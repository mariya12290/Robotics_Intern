#include <algorithm>
#include <std_msgs/Bool.h>
#include <kira_msgs/Docking.h>
#include "kira_lib/utils/utils.hpp"
#include "docking_station.hpp"

namespace Sim {
using namespace kira_lib;

DockingStation::DockingStation(ros::NodeHandle& nh) :
    ComponentVirtualBase(nh, UPDATE_RATE), m_nh{nh}
{
    m_baseRefHandle = simGetObjectHandle("base_ref");
    m_robotBaseHandle = simGetObjectHandle("robot_base");
    Utils::LoadRosParam(m_nh, std::string{NAME} + "/y_position_threshold", m_positionThreshold);
    Utils::LoadRosParam(m_nh, std::string{NAME} + "/orientation_threshold", m_orientationThreshold);

    std::string dockingStationPath{""};

    Utils::LoadRosParam(m_nh, std::string{NAME} + "/docking_pose", m_dockingPoseArray);
    Utils::LoadRosParam(m_nh, std::string{NAME} + "/docking_station_path", dockingStationPath);

    m_dockingStationHandle = AddDockingStation(dockingStationPath);
    if (m_dockingStationHandle != -1)
    {
        m_dockingStationPlugin = simGetObjectHandle("Docking_Station_Plugin");
        m_dockingStationCollision = simGetObjectHandle("Docking_Station_Collision");
        ComponentHandleExtraction(
            std::vector<std::string_view>{"Marker", "Part2", "Part3", "Part1"});
        m_dockingServer =
            m_nh.advertiseService("docking_service", &DockingStation::DockingService, this);
    }
    m_dockedPublisher = m_nh.advertise<std_msgs::Bool>("is_docked", 10);
}

int DockingStation::AddDockingStation(const std::string& dockingStationPath)
{
    auto dockingHandle = simLoadModel(dockingStationPath.c_str());
    if (dockingHandle == -1)
    {
        ROS_WARN_STREAM_COND(dockingHandle == -1, "could not load docking station model");
        return -1;
    }
    /* the offset for the model from the floor */
    static constexpr float DOCKING_Z_OFFSET = 0.0;

    sim_vector_t position{m_dockingPoseArray[0], m_dockingPoseArray[1], DOCKING_Z_OFFSET};
    simSetObjectPosition(dockingHandle, -1, position.data());

    sim_vector_t orientation{-0.0f, 0.0f, m_dockingPoseArray[2]};
    simSetObjectOrientation(dockingHandle, -1, orientation.data());
    return dockingHandle;
}

void DockingStation::ComponentHandleExtraction(
    std::vector<std::string_view> const& componentPrefixName)
{
    int objectCount{0};

    SmartBuffer<simInt> handles =
        simGetObjectsInTree(m_dockingStationHandle, sim_handle_all, 0, &objectCount);
    for (auto i{0}; i < objectCount; i++)
    {

        SmartBuffer<char> nameBuffer = simGetObjectName(handles[i]);
        std::string componentName{nameBuffer.data};

        if (std::any_of(componentPrefixName.begin(), componentPrefixName.end(),
                        [&](std::string_view prefix) {
                            return (componentName.find(prefix) != std::string_view::npos);
                        }))
        {
            continue;
        }
        m_handles.emplace_back(handles[i]);
    }
}

const std::string_view DockingStation::Name() const
{
    return NAME;
}

bool DockingStation::DockingService(kira_msgs::Docking::Request& req,
                                    kira_msgs::Docking::Response& res)
{
    if (!m_isdocked || req.command == kira_msgs::Docking::Request::DOCKING_COMMAND_STOP)

    {
        m_pulse = false;
        res.error_code = kira_msgs::Docking::Response::DOCKING_ERROR;
        return true;
    }
    else
    {
        m_pulse = true;
        res.error_code = kira_msgs::Docking::Response::DOCKING_NO_ERROR;
        return true;
    }
}

void DockingStation::RunOnce(unsigned, unsigned)
{
    CheckDocked();
    UpdateDockingColor();
}

void DockingStation::Shutdown()
{
    m_dockingServer.shutdown();
}

void DockingStation::UpdateDockingColor()
{

    if (m_pulse && m_isdocked)
    {
        m_tickCount += 1;

        switch (m_tickCount)
        {
        case 5:
            std::for_each(m_handles.begin(), m_handles.end(), [&](int value) {
                simSetShapeColor(value, nullptr, sim_colorcomponent_ambient_diffuse,
                                 m_colorBlack.data());
            });
            break;
        case 10:
            std::for_each(m_handles.begin(), m_handles.end(), [](int value) {
                simSetShapeColor(value, nullptr, sim_colorcomponent_ambient_diffuse,
                                 COLOR_BLUE.data());
            });
            m_tickCount = 0;
            break;
        }
    }
    else if (m_isdocked)
    {
        std::for_each(m_handles.begin(), m_handles.end(), [](int value) {
            simSetShapeColor(value, nullptr, sim_colorcomponent_ambient_diffuse, COLOR_BLUE.data());
        });
    }
    else
    {
        std::for_each(m_handles.begin(), m_handles.end(), [&](int value) {
            simSetShapeColor(value, nullptr, sim_colorcomponent_ambient_diffuse,
                             m_colorBlack.data());
        });
    }
}

void DockingStation::CheckDocked()
{
    std_msgs::Bool docked;
    docked.data = false;
    m_isdocked = false;
    if (CheckCollision() && CheckIsInBound() && CheckOrientation())
    {
        m_isdocked = true;
        docked.data = true;
    }
    m_dockedPublisher.publish(docked);
}

bool DockingStation::CheckCollision()
{
    return (simCheckCollision(m_robotBaseHandle, m_dockingStationCollision));
}

bool DockingStation::CheckOrientation()
{
    /** we can adjust the threshold according to our requirements in the launch file */
    simGetObjectOrientation(m_baseRefHandle, m_dockingStationPlugin, m_baseOrientation.data());
    return ((m_baseOrientation[2] >= -m_orientationThreshold) &&
            (m_baseOrientation[2] <= m_orientationThreshold));
}

bool DockingStation::CheckIsInBound()
{
    /** we can adjust the threshold according to our requirements in the launch file */

    /** when the robot reaches the start pose for docking, linear.x of robot will be decreasing
     * relative to docking station plugin, similarly linear.y of robot should be decreasing linearly
     * with provided threshold, which means position bound will be set for new position everytime
     * w.r.t x position */

    /** reference  frame of a shape located in the middle of the shape.  y_position_threshold will
     * be in cm from the centre of docking station plugin in both direction + & - */

    simGetObjectPosition(m_baseRefHandle, m_dockingStationPlugin, m_basePos.data());
    return (((m_basePos[0] + POSITION_DIFF) - m_positionThreshold) <= m_basePos[1] &&
            (-((m_basePos[0] + POSITION_DIFF) - m_positionThreshold) >=
             m_basePos[1])); // PositionDifference + threshold
}
} // namespace Sim
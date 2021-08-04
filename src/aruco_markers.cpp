#include "aruco_marker.hpp"
#include "kira_lib/utils/utils.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <sys/stat.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "XmlRpcException.h"

namespace Sim {

using namespace kira_lib;

ArucoMarker::ArucoMarker(ros::NodeHandle& nh) : m_nh(nh)
{
    CreateAndPlaceMarker();
}

void ArucoMarker::LoadMarkerDescriptionFromRos()
{
    XmlRpc::XmlRpcValue markers;
    Marker arCode;
    if (not Utils::LoadRosParam(m_nh, "marker_size", m_size) or
        not Utils::LoadRosParam(m_nh, "markers", markers, ::ros::console::Level::Debug))
    {
        /* when no markers are set they are not needed, silently continue */
        return;
    }
    try
    {
        for (auto i = 0; i < markers.size(); ++i)
        {
            XmlRpc::XmlRpcValue& sublist = markers[i];
            arCode.id = sublist["code"];
            arCode.pos[0] = static_cast<float>(static_cast<double>(sublist["xyz"][0]));
            arCode.pos[1] = static_cast<float>(static_cast<double>(sublist["xyz"][1]));
            arCode.pos[2] = static_cast<float>(static_cast<double>(sublist["xyz"][2]));
            arCode.orientation[0] = static_cast<float>(static_cast<double>(sublist["rpy"][0]));
            arCode.orientation[1] = static_cast<float>(static_cast<double>(sublist["rpy"][1]));
            arCode.orientation[2] = static_cast<float>(static_cast<double>(sublist["rpy"][2]));
            m_markers.emplace_back(arCode);
        }
    }
    catch (const XmlRpc::XmlRpcException&)
    {
        m_markers.clear();
        ROS_WARN("could not load ArUco marker descriptions");
    }
}

void ArucoMarker::GenerateImages(const std::string& outputDirectory) const
{
    auto borderBits = 1; /**Number of bits in marker borders*/
    auto markerSize = 100; /**Marker size in pixels */
    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);

    auto result = mkdir(outputDirectory.c_str(), 0777);
    if ((result == -1) && (errno != EEXIST))
    {
        ROS_WARN("output directory creation for Aruco codes  failed");
    }
    else
    {
        for (const auto& marker : m_markers)
        {
            cv::Mat markerImg;
            cv::aruco::drawMarker(dictionary, marker.id, markerSize, markerImg, borderBits);
            imwrite((outputDirectory + "/" + "aruco_" + std::to_string(marker.id) + ".jpg").c_str(),
                    markerImg);
        }
    }
}

void ArucoMarker::CreateAndPlaceMarker()
{
    LoadMarkerDescriptionFromRos();
    GenerateImages(m_outputDirectory);
    PlaceMarkersIntoSim(m_outputDirectory);
}

void ArucoMarker::PlaceMarkersIntoSim(const std::string& outputDirectory) const
{
    auto arucoHandle = simCreateDummy(0, nullptr);
    simSetObjectName(arucoHandle, "aruco_makers");
    for (const auto& marker : m_markers)
    {
        auto handle = simCreateTexture(
            (outputDirectory + "/" + "aruco_" + std::to_string(marker.id) + ".jpg").c_str(), 0,
            m_size.data(), nullptr, nullptr, 0, nullptr, nullptr, nullptr);
        if (handle == -1)
        {
            ROS_WARN(" couldn't load aruco marker");
        }
        else
        {
            simSetObjectName(handle, ("aruco_" + std::to_string(marker.id)).c_str());
            auto resultPos = simSetObjectPosition(handle, -1, marker.pos.data());
            ROS_WARN_STREAM_COND(resultPos == -1, "could not set the object position");

            tf2::Quaternion quat;
            quat.setRPY(static_cast<double>(marker.orientation[0]),
                        static_cast<double>(marker.orientation[1]),
                        static_cast<double>(marker.orientation[2]));
            sim_quaternion_t rot{static_cast<float>(quat.x()), static_cast<float>(quat.y()),
                                 static_cast<float>(quat.z()), static_cast<float>(quat.w())};
            auto resultOrientation = simSetObjectQuaternion(handle, -1, rot.data());
            ROS_WARN_STREAM_COND(resultOrientation == -1, "could not set object orientation");
            SetComponentParent(arucoHandle, handle);
        }
    }
}
} // namespace Sim

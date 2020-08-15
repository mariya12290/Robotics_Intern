#include "sim/camera.hpp"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl_ros/transforms.h"
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include "kira_lib/utils/utils.hpp"
#include "simLib.h"
#include <tf2_ros/static_transform_broadcaster.h>

namespace Sim {

using namespace kira_lib;

Camera::Camera(ros::NodeHandle& nh) : ComponentVirtualBase(nh, UPDATE_RATE)
{
    std::vector<std::string> cameraList;
    /* get the camera names from the rosparam list in the launch file */

    ros::NodeHandle cameraNodeHandle(nh, NAME);
    Utils::LoadRosParam(cameraNodeHandle, "cameras", cameraList);

    for (const auto& cameraName : cameraList)
    {
        m_cameraSensors.emplace_back(std::make_unique<SingleCamera>(cameraNodeHandle, cameraName));
    }
}

void Camera::RunOnce(unsigned t, unsigned)
{
    for (const auto& camera : m_cameraSensors)
    {
        sensor_msgs::PointCloud2 pointCloud;
        sensor_msgs::Image image;
        if (camera->m_enableColor)
        {
            image = camera->ExtractCameraColorData();
        }
        if (camera->m_enableDepth)
        {
            pointCloud = camera->ExtractCameraDepthData();
        }
        camera->AddStampToMeasurementMessagesAndPublish(t, image, pointCloud);
        camera->PublishCameraInfo(TimeStampToRosTime(t));
    }
}

bool Camera::SingleCamera::VerifyPointCloudData(sensor_msgs::PointCloud2& pointCloud) const
{
    if ((pointCloud.width * pointCloud.height) > 0)
    {
        return true;
    }
    return false;
}

void Camera::Shutdown()
{
    for (const auto& camera : m_cameraSensors)
    {
        camera->ShutDownPublishers();
    }
    simResetVisionSensor(sim_handle_all);
}
std::vector<std::string> Camera::RequiredFrames()
{
    std::vector<std::string> frames;
    frames.reserve(m_cameraSensors.size());
    std::transform(m_cameraSensors.begin(), m_cameraSensors.end(), std::back_inserter(frames),
                   [](const auto& cameraSensor) { return cameraSensor->Frame(); });
    return frames;
}
void Camera::SetTransformations(const std::vector<geometry_msgs::TransformStamped>& transforms)
{
    if (transforms.size() != m_cameraSensors.size())
    {
        ROS_WARN("Wrong number of transforms for camera sensors, skipping tf correction");
        return;
    }

    for (auto i = 0u; i < transforms.size(); ++i)
    {
        m_cameraSensors[i]->ApplyTransform(transforms[i]);
    }
}

Camera::SingleCamera::SingleCamera(ros::NodeHandle parentNodeHandle, std::string cameraName) :
    m_nh{ros::NodeHandle(parentNodeHandle, cameraName)}, m_sensorName(std::move(cameraName))
{
    Utils::LoadRosParam(parentNodeHandle, "enable_color", m_enableColor);
    Utils::LoadRosParam(parentNodeHandle, "enable_depth", m_enableDepth);

    // overwriting flags for individual camera
    Utils::LoadRosParam(m_nh, "enable_color", m_enableColor);
    Utils::LoadRosParam(m_nh, "enable_depth", m_enableDepth);
    ROS_INFO_STREAM(m_sensorName << ": {\"enable_color\": " << m_enableColor
                                 << " , \"enable_depth\": " << m_enableDepth << "}");

    float viewAngleDepth{};
    resolution_vector_t resolutionDepth{};

    float viewAngleColor{};
    resolution_vector_t resolutionColor{};
    if (m_enableColor && m_enableDepth)
    {
        m_colorPointCloud = false;
        std::tie(resolutionDepth, viewAngleDepth) = LoadDepthParameters(parentNodeHandle, m_nh);
        std::tie(resolutionColor, viewAngleColor) = LoadColorParameters(parentNodeHandle, m_nh);
    }
    else if (m_enableColor)
    {
        m_colorPointCloud = false;
        std::tie(resolutionColor, viewAngleColor) = LoadColorParameters(parentNodeHandle, m_nh);
    }
    else if (m_enableDepth)
    {
        m_colorPointCloud = true;
        std::tie(resolutionDepth, viewAngleDepth) = LoadDepthParameters(parentNodeHandle, m_nh);
        std::tie(resolutionColor, viewAngleColor) = LoadColorParameters(parentNodeHandle, m_nh);
    }

    Utils::LoadRosParam(parentNodeHandle, "camera_path", m_cameraPath,
                        ::ros::console::Level::Debug);
    Utils::LoadRosParam(m_nh, "camera_path", m_cameraPath, ::ros::console::Level::Debug);

    sim_vector_t pos;
    sim_vector_t orientation;

    Utils::LoadRosParam(parentNodeHandle, "xyz", pos, ::ros::console::Level::Debug);
    Utils::LoadRosParam(m_nh, "xyz", pos, ::ros::console::Level::Debug);
    Utils::LoadRosParam(parentNodeHandle, "rpy", orientation, ::ros::console::Level::Debug);
    Utils::LoadRosParam(m_nh, "rpy", orientation, ::ros::console::Level::Debug);

    std::tie(m_sensorHandleRGB, m_sensorHandleDepth) =
        AddCamera(m_cameraPath, m_sensorName, NAME, pos, orientation);

    if ((m_sensorHandleRGB == -1) && (m_sensorHandleDepth == -1))
    {
        ROS_FATAL_STREAM("Simulator shutdown as camera couldn't be added ");
        simQuitSimulator(true);
    }
    else
    {
        ROS_INFO_STREAM("camera: " << m_sensorName << " Added");
    }

    SetCameraResolution(m_sensorHandleRGB, resolutionColor);
    SetCameraFieldOfView(m_sensorHandleRGB, viewAngleColor);

    m_colorInfo = ComputeCameraInfo(m_sensorHandleRGB);

    if (m_enableDepth)
    {
        Utils::LoadRosParam(parentNodeHandle, "near_and_far_clip_depth", m_nearAndFarClipDepth,
                            ::ros::console::Level::Debug);
        Utils::LoadRosParam(m_nh, "near_and_far_clip_depth", m_nearAndFarClipDepth,
                            ::ros::console::Level::Debug);
        SetCameraResolution(m_sensorHandleDepth, resolutionDepth);
        SetCameraFieldOfView(m_sensorHandleDepth, viewAngleDepth);

        m_depthInfo = ComputeCameraInfo(m_sensorHandleDepth);
        SetNfClipping(m_sensorHandleDepth, m_nearAndFarClipDepth);
    }
}

std::pair<Camera::SingleCamera::resolution_vector_t, float>
Camera::SingleCamera::LoadDepthParameters(ros::NodeHandle parentNodeHandle,
                                          ros::NodeHandle childNodeHandle)
{
    m_frameMap.insert(std::make_pair("depth", Name() + "_depth_frame"));
    m_frameMap.insert(std::make_pair("depth_optical", Name() + "_depth_optical_frame"));
    ComputeAndPublishFrameForDepth();
    m_publisherDepth =
        m_nh.advertise<sensor_msgs::PointCloud2>("/" + Name() + "/depth/color/points", 100);

    m_pubcameraDepthInfo =
        m_nh.advertise<sensor_msgs::CameraInfo>("/" + Name() + "/depth/camera_info", 100);

    resolution_vector_t resolutionDepth;
    Utils::LoadRosParam(parentNodeHandle, "resolution_depth", resolutionDepth,
                        ::ros::console::Level::Debug);
    Utils::LoadRosParam(childNodeHandle, "resolution_depth", resolutionDepth,
                        ::ros::console::Level::Debug);

    float viewAngleDepth;

    Utils::LoadRosParam(parentNodeHandle, "field_of_view_depth", viewAngleDepth,
                        ::ros::console::Level::Debug);
    Utils::LoadRosParam(childNodeHandle, "field_of_view_depth", viewAngleDepth,
                        ::ros::console::Level::Debug);
    return (std::make_pair(resolutionDepth, viewAngleDepth));
}

std::pair<Camera::SingleCamera::resolution_vector_t, float>
Camera::SingleCamera::LoadColorParameters(ros::NodeHandle parentNodeHandle,
                                          ros::NodeHandle childNodeHandle)
{
    m_frameMap.insert(std::make_pair("color", Name() + "_color_frame"));
    m_frameMap.insert(std::make_pair("color_optical", Name() + "_color_optical_frame"));
    ComputeAndPublishFrameForColor();

    if (!m_colorPointCloud)
    {
        m_publisherRGBImage =
            m_nh.advertise<sensor_msgs::Image>("/" + Name() + "/color/image_raw", 100);
        m_pubcameraRgbInfo =
            m_nh.advertise<sensor_msgs::CameraInfo>("/" + Name() + "/color/camera_info", 100);
    }

    resolution_vector_t resolutionColor;
    Utils::LoadRosParam(parentNodeHandle, "resolution_color", resolutionColor,
                        ::ros::console::Level::Debug);
    Utils::LoadRosParam(childNodeHandle, "resolution_color", resolutionColor,
                        ::ros::console::Level::Debug);

    float viewAngleColor;
    Utils::LoadRosParam(parentNodeHandle, "field_of_view_color", viewAngleColor,
                        ::ros::console::Level::Debug);
    Utils::LoadRosParam(childNodeHandle, "field_of_view_color", viewAngleColor,
                        ::ros::console::Level::Debug);

    return (std::make_pair(resolutionColor, viewAngleColor));
}

std::string Camera::SingleCamera::Frame() const
{
    return Name() + "_link";
}

std::string Camera::SingleCamera::Name() const
{
    return std::string(Camera::NAME) + "_" + m_sensorName;
}

void Camera::SingleCamera::ApplyTransform(const geometry_msgs::TransformStamped& transform)
{

    /** @attention in Coppelia Sim the used kinect models base frame has y as a measuring axis.
     * This means the camera coordinate system has a yaw angle of -90 deg relative to
     * the sensor coordinate system in the ROS TF tree
     */

    auto baseHandle = GetBaseHandle();
    tf2::Quaternion rosRotCorrection;
    rosRotCorrection.setRPY(0.0, 0.0, -M_PI / 2.0);

    std::array<float, 3> pos;
    pos[0] = static_cast<float>(transform.transform.translation.x);
    pos[1] = static_cast<float>(transform.transform.translation.y);
    pos[2] = static_cast<float>(transform.transform.translation.z);

    auto objectHandle = simGetObjectHandle((std::string(NAME) + "_" + m_sensorName).c_str());
    simSetObjectPosition(objectHandle, baseHandle, pos.data());

    tf2::Quaternion tfQuat;
    tf2::fromMsg(transform.transform.rotation, tfQuat);
    auto rotQuat = tfQuat * rosRotCorrection;
    std::array<float, 4> rot{static_cast<float>(rotQuat.x()), static_cast<float>(rotQuat.y()),
                             static_cast<float>(rotQuat.z()), static_cast<float>(rotQuat.w())};
    simSetObjectQuaternion(objectHandle, baseHandle, rot.data());
}

void Camera::SingleCamera::PublishStaticFrame(std::array<double, 3>& trans,
                                              const tf2::Quaternion& q, const std::string& from,
                                              const std::string& to)
{
    static tf2_ros::StaticTransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = from;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.child_frame_id = to;
    transformStamped.transform.translation.x = trans[0];
    transformStamped.transform.translation.y = trans[1];
    transformStamped.transform.translation.z = trans[2];
    transformStamped.transform.rotation.w = q.getW();
    transformStamped.transform.rotation.x = q.getX();
    transformStamped.transform.rotation.y = q.getY();
    transformStamped.transform.rotation.z = q.getZ();
    broadcaster.sendTransform(transformStamped);
}

void Camera::SingleCamera::SetCameraResolution(int cameraHandle,
                                               const resolution_vector_t& resolution)
{

    auto res_x =
        simSetObjectIntParameter(cameraHandle, sim_visionintparam_resolution_x, resolution[0]);
    ROS_INFO_COND(res_x == -1, "couldn't set resolution_x ");
    auto res_y =
        simSetObjectIntParameter(cameraHandle, sim_visionintparam_resolution_y, resolution[1]);
    ROS_INFO_COND(res_y == -1, "couldn't set resolution_y ");
}

void Camera::SingleCamera::SetNfClipping(
    int cameraHandle, const Camera::SingleCamera::nf_clipping_vector_t& clippingPlaneDistance)
{
    auto resNear = simSetObjectFloatParameter(cameraHandle, sim_visionfloatparam_far_clipping,
                                              clippingPlaneDistance[0]);
    ROS_INFO_COND(resNear == -1, "couldn't set near clipping plane distance ");
    auto resFar = simSetObjectFloatParameter(cameraHandle, sim_visionfloatparam_far_clipping,
                                             clippingPlaneDistance[1]);
    ROS_INFO_COND(resFar == -1, "couldn't set far clipping plane distance ");
}

void Camera::SingleCamera::SetCameraFieldOfView(int cameraHandle, float angle)
{

    auto res =
        simSetObjectFloatParameter(cameraHandle, sim_visionfloatparam_perspective_angle, angle);
    ROS_INFO_COND(res == -1, "couldn't set FOV angle");
}

void Camera::SingleCamera::ComputeAndPublishFrameForColor()
{
    std::array<double, 3> trans{0, 0, 0};
    trans.at(1) = 0.01488;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    PublishStaticFrame(trans, q, Frame(), m_frameMap.at("color"));
    trans = {0};
    q.setRPY((M_PI / 2), M_PI, (M_PI / 2));
    PublishStaticFrame(trans, q, m_frameMap.at("color"), m_frameMap.at("color_optical"));
}

void Camera::SingleCamera::ComputeAndPublishFrameForDepth()
{
    std::array<double, 3> trans{0, 0, 0};
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    PublishStaticFrame(trans, q, Frame(), m_frameMap.at("depth"));
    q.setRPY((M_PI / 2), M_PI, (M_PI / 2));
    PublishStaticFrame(trans, q, m_frameMap.at("depth"), m_frameMap.at("depth_optical"));
}

void Camera::SingleCamera::ShutDownPublishers()
{
    if (m_enableColor)
    {
        m_publisherRGBImage.shutdown();
        m_pubcameraRgbInfo.shutdown();
    }
    if (m_enableDepth)
    {
        m_publisherDepth.shutdown();
        m_pubcameraDepthInfo.shutdown();
    }
}

std::pair<int, int> Camera::SingleCamera::AddCamera(const std::string& cameraPath,
                                                    const std::string& cameraName,
                                                    const std::string& componentName,
                                                    const sim_vector_t& pos,
                                                    const sim_vector_t& orientation)
{

    auto cameraHandle = simLoadModel(cameraPath.c_str());
    if (cameraHandle == -1)
    {
        ROS_WARN_STREAM_COND(cameraHandle == -1, "could not load camera module");
        return std::make_pair(-1, -1);
    }
    RenameComponentAndAllChildren(cameraHandle, componentName, cameraName);

    auto sensorHandleRgb = simGetObjectHandle((componentName + "_" + cameraName + "_rgb").c_str());
    auto sensorHandleDepth =
        simGetObjectHandle((componentName + "_" + cameraName + "_depth").c_str());
    SetComponentPosition(cameraHandle, pos, orientation);
    auto robo = simGetObjectHandle("kira_b50");
    bool res = SetComponentParent(robo, cameraHandle);
    ROS_WARN_STREAM_COND(res == false, " camera could not attach to  its parent");
    return std::make_pair(sensorHandleRgb, sensorHandleDepth);
}

void Camera::SingleCamera::AddStampToMeasurementMessagesAndPublish(
    uint32_t ibrTimestamp, sensor_msgs::Image& imgmessage, sensor_msgs::PointCloud2& pcmessage)
{

    imgmessage.header.stamp = TimeStampToRosTime(ibrTimestamp);
    if (m_enableColor)
    {
        std::stringstream rgbLinkName;

        imgmessage.header.frame_id = m_frameMap.at("color_optical");
        m_publisherRGBImage.publish(imgmessage);
    }
    if (m_enableDepth)
    {
        std::stringstream depthLinkName;
        pcmessage.header.frame_id = m_frameMap.at("depth_optical");

        pcmessage.header.stamp.sec = imgmessage.header.stamp.sec;
        pcmessage.header.stamp.nsec = imgmessage.header.stamp.nsec;
        if (VerifyPointCloudData(pcmessage))
        {
            m_publisherDepth.publish(pcmessage);
        }
    }
}

sensor_msgs::CameraInfo Camera::SingleCamera::ComputeCameraInfo(int cameraHandle)
{
    sensor_msgs::CameraInfo msg;
    resolution_vector_t cameraResolution;
    if (simGetVisionSensorResolution(cameraHandle, cameraResolution.data()) != -1)
    {

        msg.width = static_cast<uint>(cameraResolution[0]);
        msg.height = static_cast<uint>(cameraResolution[1]);
        float view_angle;
        simGetObjectFloatParameter(cameraHandle, sim_visionfloatparam_perspective_angle,
                                   &view_angle);
        double f_x = (msg.width / 2.) / tan(view_angle / 2.);
        double f_y = f_x;
        msg.D.resize(5);
        msg.distortion_model = "plumb_bob";

        for (auto i = 0u; i < 5; i++)
        {
            msg.D[i] = 0;
        }
        msg.K[0] = f_x;
        msg.K[1] = 0;
        msg.K[2] = msg.width / 2;
        msg.K[3] = 0;
        msg.K[4] = f_y;
        msg.K[5] = msg.height / 2;
        msg.K[6] = 0;
        msg.K[7] = 0;
        msg.K[8] = 1;

        msg.R[0] = 1;
        msg.R[1] = 0;
        msg.R[2] = 0;
        msg.R[3] = 0;
        msg.R[4] = 1;
        msg.R[5] = 0;
        msg.R[6] = 0;
        msg.R[7] = 0;
        msg.R[8] = 1;

        msg.P[0] = msg.K[0];
        msg.P[1] = 0;
        msg.P[2] = msg.K[2];
        msg.P[3] = 0;
        msg.P[4] = 0;
        msg.P[5] = msg.K[4];
        msg.P[6] = msg.K[5];
        msg.P[7] = 0;
        msg.P[8] = 0;
        msg.P[9] = 0;
        msg.P[10] = 1;
        msg.P[11] = 0;
    }

    return msg;
}

void Camera::SingleCamera::AddHeaderToMessage(ros::Time t, sensor_msgs::CameraInfo& message,
                                              std::string frameName)
{
    message.header.stamp = t;
    message.header.frame_id = frameName;
}

void Camera::SingleCamera::PublishCameraInfo(ros::Time t)
{
    if (!m_colorPointCloud && m_enableColor)
    {
        AddHeaderToMessage(t, m_colorInfo, m_frameMap.at("color_optical"));
        m_pubcameraRgbInfo.publish(m_colorInfo);
    }
    if (m_enableDepth)
    {
        AddHeaderToMessage(t, m_depthInfo, m_frameMap.at("depth_optical"));
        m_pubcameraDepthInfo.publish(m_depthInfo);
    }
}

sensor_msgs::Image Camera::SingleCamera::ExtractCameraColorData() const
{

    sensor_msgs::Image imgmessage;

    /* Self defined class to fit the API function to avoid memory leak*/
    SmartBuffer<float> auxValuesRGB;
    SmartBuffer<int> auxValuesRGBCount;
    SmartBuffer<simUChar> rgbValues;

    std::array<int, 2> cameraResolutionRGB;

    /* Invoke rgb vision sensor */
    int isRGBCameraDataSucceed =
        simHandleVisionSensor(m_sensorHandleRGB, &auxValuesRGB.data, &auxValuesRGBCount.data);

    bool sensorHandledRGB{(isRGBCameraDataSucceed >= 0)};
    if (sensorHandledRGB)
    {
        /* Get the used camera resolution of the vision sensor */
        const int isGetResolutionSucceed =
            simGetVisionSensorResolution(m_sensorHandleRGB, cameraResolutionRGB.data());

        if (isGetResolutionSucceed > 0)
        {
            rgbValues.data = simGetVisionSensorCharImage(m_sensorHandleRGB, &cameraResolutionRGB[0],
                                                         &cameraResolutionRGB[1]);
            /* Flip the rgb values around since Coppelia Sim uses another coordinate
            system
               bit0 set (1): the provided image is rgba, otherwise it is rgb
               bit1 set (2): the image will be flipped on its x-axis
               bit2 set (4): the image will be flipped on its y-axis */
            int checkTransform = simTransformImage(rgbValues.data, cameraResolutionRGB.data(), 2,
                                                   nullptr, nullptr, nullptr);
            if (checkTransform < 0)
            {
                ROS_WARN_STREAM(
                    "Couldn't flip RGB-Image! Color values might be off in pointcloud!");
            }

            /* Flip the rgb values around since Coppelia Sim uses another coordinate
            system
              bit0 set (1): the provided image is rgba, otherwise it is rgb
              bit1 set (2): the image will be flipped on its x-axis
              bit2 set (4): the image will be flipped on its y-axis
              Because of different coordinate system conventions the imgs must be flipped
              around the y-axis for a proper rviz presentation */
            int checkTransform2 = simTransformImage(rgbValues.data, cameraResolutionRGB.data(), 2,
                                                    nullptr, nullptr, nullptr);
            int checkTransform3 = simTransformImage(rgbValues.data, cameraResolutionRGB.data(), 4,
                                                    nullptr, nullptr, nullptr);

            if (checkTransform2 < 0 || checkTransform3 < 0)
            {
                ROS_WARN("Couldn't flip RGB-Image! Image messages might be upside down in rviz");
            }
            imgmessage.encoding = "rgb8";
            imgmessage.is_bigendian = 0;
            imgmessage.width = static_cast<uint>(cameraResolutionRGB[0]);
            imgmessage.height = static_cast<uint>(cameraResolutionRGB[1]);
            imgmessage.step = static_cast<uint>(cameraResolutionRGB[0] * AMOUNT_RGB_CHANNELS);

            /* Insert entire RGB buffer into ROS data message */
            imgmessage.data.insert(imgmessage.data.end(), rgbValues.begin(),
                                   rgbValues.begin() + (imgmessage.height * imgmessage.step));
        }
    }
    else
    {
        ROS_ERROR_STREAM_COND(!sensorHandledRGB, "Could not extract RGB data");
    }
    return imgmessage;
}

sensor_msgs::PointCloud2 Camera::SingleCamera::ExtractCameraDepthData() const
{
    sensor_msgs::PointCloud2 pcMessageTransformed;

    /* Self defined class to fit the API function to avoid memory leak*/
    SmartBuffer<float> auxValuesDepth;
    SmartBuffer<int> auxValuesDepthCount;
    SmartBuffer<simUChar> rgbValues;

    /* Invoke depth vision sensor and get the 3D coordinates relative to the camera from the
     * sensor
     */
    int isDepthCameraDataSucceed =
        simHandleVisionSensor(m_sensorHandleDepth, &auxValuesDepth.data, &auxValuesDepthCount.data);

    bool sensorHandledDepth{(isDepthCameraDataSucceed >= 0)};

    std::array<int, 2> cameraResolutionRGB;

    if (sensorHandledDepth && (auxValuesDepthCount.data[0] == 2))
    {
        /* Get the used camera resolution of the vision sensor */
        const int isGetResolutionSucceed =
            simGetVisionSensorResolution(m_sensorHandleRGB, cameraResolutionRGB.data());
        if (isGetResolutionSucceed > 0)
        {
            /* Get the rgb value of each pixel for colorization of the pointcloud */
            rgbValues.data = simGetVisionSensorCharImage(m_sensorHandleRGB, &cameraResolutionRGB[0],
                                                         &cameraResolutionRGB[1]);
            /* Flip the rgb values around since Coppelia Sim uses another coordinate
            system
               bit0 set (1): the provided image is rgba, otherwise it is rgb
               bit1 set (2): the image will be flipped on its x-axis
               bit2 set (4): the image will be flipped on its y-axis */
            int checkTransform = simTransformImage(rgbValues.data, cameraResolutionRGB.data(), 2,
                                                   nullptr, nullptr, nullptr);
            if (checkTransform < 0)
            {
                ROS_WARN_STREAM(
                    "Couldn't flip RGB-Image! Color values might be off in pointcloud!");
            }
        }
        /* Create Quaternions for neccesary rotations */
        tf::Quaternion rotateQuat1, rotateQuat2;
        geometry_msgs::Quaternion rotateQuatMsg;

        /* Transform to remove the offset while publishing pointcloud in Rviz*/
        rotateQuat1.setRPY(-M_PI, 0, 0);
        rotateQuat2.setRPY(0, M_PI, 0);

        rotateQuat1.normalize();
        rotateQuat2.normalize();

        tf::Transform msgTransformation1(rotateQuat1);
        tf::Transform msgTransformation2(rotateQuat2);
        const int sizePackage1 = auxValuesDepthCount[1]; /* Package 1 holds additional information
                                                            about e.g. intensities */
        const int sizePackage2 = auxValuesDepthCount[2]; /* Package 2 contains the actual depth
                                                            values gathered from the v-rep filter */
        pcl::PointCloud<pcl::PointXYZRGB> rawPC;
        rawPC.reserve(static_cast<size_t>(sizePackage2));

        int rgbcounter = 0;
        pcl::PointXYZRGB singlePoint;

        /* Loop through all the depth buffer data points and give each point in pointcloud
          the correct euclidean coordinates and rgb values. The buffer values are the
          following: Value 1: Points in x direction Value 2: Points in y direction Value 3:
          Euclidean coord x Point1 Value 4: Euclidean coord y Point1 Value 5: Euclidean
          coord z Point1 Value 6: Distance Point 1 Value 7: Euclidean coord x Point2
          ...                                */
        for (int counter = sizePackage1 + 2; counter < sizePackage1 + sizePackage2; counter += 4)
        {
            /* Check for max and min clip distances and only add points to PointCloud2
             * representation that are within this range */
            if ((auxValuesDepth.data[counter + 3] < m_nearAndFarClipDepth[1]) &&
                (auxValuesDepth.data[counter + 3] > m_nearAndFarClipDepth[0]))
            {
                singlePoint.x = auxValuesDepth.data[counter];
                singlePoint.y = auxValuesDepth.data[counter + 1];
                singlePoint.z = auxValuesDepth.data[counter + 2];

                if (isGetResolutionSucceed > 0)
                {
                    singlePoint.r = rgbValues.data[rgbcounter];
                    singlePoint.g = rgbValues.data[rgbcounter + 1];
                    singlePoint.b = rgbValues.data[rgbcounter + 2];
                }
                rawPC.push_back(singlePoint);
            }
            rgbcounter += AMOUNT_RGB_CHANNELS;
        }

        /* Create neccesary Pointclouds from pcl library */
        pcl::PointCloud<pcl::PointXYZRGB> transformedPC1;
        pcl::PointCloud<pcl::PointXYZRGB> transformedPC2;
        /* Apply rotations around x and z axis to fit ros coordiate system conventions */
        pcl_ros::transformPointCloud(rawPC, transformedPC1, msgTransformation1);
        pcl_ros::transformPointCloud(transformedPC1, transformedPC2, msgTransformation2);
        pcl::toROSMsg(transformedPC2, pcMessageTransformed);
    }
    else
    {
        ROS_ERROR_STREAM_COND(!sensorHandledDepth, "Could not extract point cloud data");
    }
    return pcMessageTransformed; // named return value optimization
}

} // namespace Sim

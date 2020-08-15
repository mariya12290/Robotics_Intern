/**
 * @file sim/camera_sensor.hpp
 * @brief provides a simple implematation template for vision sensors including
 * ROS msgs
 * @author Aralapura Mariyappa, Surendra Kumar
 * SurendraKumar.AralapuraMariyappa@de.kaercher.com
 * @copyright 2019 Alfred Kaercher GmbH & Co. KG
 */

#ifndef SIM_CAMERA_SENSOR_HPP
#define SIM_CAMERA_SENSOR_HPP

#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "sim/component_virtual_base.hpp"
#include "sim/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <sensor_msgs/CameraInfo.h>

namespace Sim {

/**
 * @brief The Camera class
 * @details Wrapper class for the different depth vision cameras of the kira_b50
 * into the Coppelia Sim simulation. The virtual cameras provides img and
 * PointCloud2 ROS messages and all neccesary transforamations, for faster
 * Prototyping and seemless integration in the later overall system.
 */
class Camera : public ComponentVirtualBase {
public:
  /** @brief Constructor.
   * @param ROS node to register the several camera components.
   * @details The Constructor will register a camera component. The several used
   * cameras(front_top side_left, etc.) are saved as Objects in a member list
   * and are created by using the unique sennsor position as ID **/
  Camera(ros::NodeHandle &nh);

  /** @brief main simulation work runs here, which invokes every single camera
   * in the member list.
   * @param t current simulation time [ms]
   */
  void RunOnce(unsigned t, unsigned) override;

  /** @brief shutdown all camera publishers and realease all vision handles.*/
  void Shutdown() override;

  const std::string_view Name() const override { return NAME; }

  /**
   * @brief List of the camera sensor frames
   * @return names of the sensors with the suffix _link
   */
  std::vector<std::string> RequiredFrames() override;

  /**
   * @brief Apply the transformations for the camera sensors
   */
  void SetTransformations(
      const std::vector<geometry_msgs::TransformStamped> &transforms) override;

  static constexpr auto NAME = "camera"; /*!< name of the component */

private:
  /**
   * @brief The SingleCamera class
   * @details This class implements a single depth vision camera and publishes
   * the according Img and PointCloud2 messages in ROS.
   */
  class SingleCamera {
  public:
    using resolution_vector_t = std::array<int, 2>;
    using nf_clipping_vector_t = std::array<float, 2>;
    /** @brief Constructor for a single camera object.
     * @param ROS node to register the RGB and Pointcloud publishers.
     * @details The Constructor will register a camera sensor with its name and
     * then get its handle. After that the publishers will be registerd. **/
    SingleCamera(ros::NodeHandle parentNodeHandle, std::string cameraName);
    SingleCamera(SingleCamera &&cameraSensor) = delete;
    SingleCamera(const SingleCamera &cameraSensor) = delete;
    ~SingleCamera() = default; /* Default destructor */
    std::string Frame() const;
    std::string Name() const;
    void ApplyTransform(const geometry_msgs::TransformStamped &transform);
    void PublishStaticFrame(std::array<double, 3> &trans,
                            const tf2::Quaternion &q, const std::string &from,
                            const std::string &to);
    void SetCameraResolution(int cameraHandle,
                             const resolution_vector_t &resolution);
    void SetNfClipping(int cameraHandle,
                       const nf_clipping_vector_t &clippingPlaneDistance);
    void SetCameraFieldOfView(int cameraHandle, float angle);

    /* @brief  sets the values of different frames and publishes static frames.
     * @details this function will provide the TF frames for cameras similar to
     * camera manager.
     */
    void ComputeAndPublishFrameForColor();

    void ComputeAndPublishFrameForDepth();

    /** @brief remove the garbage point cloud data in the beginning. */
    bool VerifyPointCloudData(sensor_msgs::PointCloud2 &pointCloud) const;

    /**@brief get the pointCloud data to be published.
     * @return PointCloud to be published, can be shown in rviz.
     * @details this function will firstly get the sensor data from Coppelia Sim
     * by calling its Api, and generate a pointCloud with such data. Then this
     * pointCloud will be transfored from local camera sensor coordinate to
     * global coordinate. After that, the points in the cloud will be colored
     * based on their respective RGB values. */
    sensor_msgs::PointCloud2 ExtractCameraDepthData() const;

    /**@brief get the Image msg to be published.
     * @return Image to be published, can be shown in rviz.
     * @details this function will firstly get the sensor data from Coppelia Sim
     * by calling
     * its Api. The RGB- Values will be published through the passed image
     * messsage reference */
    sensor_msgs::Image ExtractCameraColorData() const;

    /** @brief publish the img and pointcloud ros msgs
     * @details this function will update the headers of the given rosmsgs and
     * published them
     * **/
    void AddStampToMeasurementMessagesAndPublish(
        uint32_t timestamp, sensor_msgs::Image &imgmessage,
        sensor_msgs::PointCloud2 &pointCloud);
    /**
     * @brief  construct the corresponding camera info message
     * @param laserHandle handle of the corresponding camera sensor
     * @param msg resulting message
     */
    sensor_msgs::CameraInfo ComputeCameraInfo(int cameraHandle);

    /**
     * @brief get the camera info Msg to be published.
     *  @param time stamp from ros node
     *  @param msg resulting message to be published
     *  @param frameName the corresponding frame in which the message is to be
     * published
     */
    void AddHeaderToMessage(ros::Time t, sensor_msgs::CameraInfo &message,
                            std::string frameName);
    /**
     *  @brief publish the camera info message
     */
    void PublishCameraInfo(ros::Time t);

    /* @brief shut down the img and pointcloud2 publishers */
    void ShutDownPublishers();

    /**
     * @brief loading of depth related parameters
     * @param Nodes
     * @return resoultion and view angle for depth
     */
    std::pair<resolution_vector_t, float>
    LoadDepthParameters(ros::NodeHandle parentNodeHandle,
                        ros::NodeHandle childNodeHandle);

    /**
     * @brief loading of image related parameters
     * @param Nodes
     * @return resoultion and view angle for image
     */
    std::pair<resolution_vector_t, float>
    LoadColorParameters(ros::NodeHandle parentNodeHandle,
                        ros::NodeHandle childNodeHandle);

    /** flag for enabling the depth and image of individual camera */
    bool m_enableDepth{};
    bool m_enableColor{};

  private:
    /**
     * @brief AddCamera - loads a Coppelia Sim camera sensor from the
     * corresponding .ttm file
     * @details Insert the camera into the Coppelia Sim at a position, renames
     * it and attaches it to the parent object.
     * @param cameraPath the path of camera.ttm file
     * @param cameraName  all subcomponents of the loaded model will be prefixed
     * with this name
     * @param componentName, all subcomponents of the loaded model will be
     * prefixed with this component name
     * @param pos (x,y,z) [m]
     * @param orientation (r,p,y) [rad]
     * @return pair of handles to the RGB and depth sensors
     */
    std::pair<int, int> AddCamera(const std::string &cameraPath,
                                  const std::string &cameraName,
                                  const std::string &componentName,
                                  const sim_vector_t &pos,
                                  const sim_vector_t &orientation);
    ros::NodeHandle m_nh;
    std::string m_sensorName; /* m_sensorname as unique ID of the camera using
                                 position as name (ex. camera_front) */
    nf_clipping_vector_t m_nearAndFarClipDepth; /* Near and far  clipping
                                                   distance for camera [m] */

    int m_sensorHandleRGB; /* Coppelia Sim handle to the given vision sensor */
    int m_sensorHandleDepth; /* Coppelia Sim handle to the given vision sensor
                              */
    ros::Publisher m_publisherRGBImage; /* Publisher for the RGB ros msg */
    ros::Publisher m_publisherDepth; /* Publisher for the PointCLoud2 ros msg */
    ros::Publisher m_pubcameraRgbInfo;
    ros::Publisher m_pubcameraDepthInfo;
    static constexpr int AMOUNT_RGB_CHANNELS = 3;
    std::string m_cameraPath = "";
    std::map<std::string, std::string>
        m_frameMap; /* to store different frames */
    sensor_msgs::CameraInfo m_colorInfo;
    sensor_msgs::CameraInfo m_depthInfo;

    /** flag for coloring the point clouds */
    bool m_colorPointCloud{false};
  };

  /** list of used cameras, names are given through rosparam list */
  std::vector<std::unique_ptr<SingleCamera>> m_cameraSensors;
  /** in [ms]. Has to be a multiple of 10. With 60ms we get roughly 15FPS */
  static constexpr auto UPDATE_RATE = 60;
};
} // namespace Sim

#endif // SIM_CAMERA_SENSOR_HPP

/**
 * @file sim/docking_station.hpp
 * @brief This file checks is_docked or not state and docking is possible are
 * not?
 * @author Surendra Kumar Aralapura Mariyappa
 * SurendraKumar.AralapuraMariyappa@de.kaercher.com
 * @copyright 2020 Alfred Kaercher SE & Co. KG
 */

#ifndef DOCKING_STATION_HPP
#define DOCKING_STATION_HPP

#include "component_virtual_base.hpp"
#include "sim/utils.hpp"
#include <kira_msgs/Docking.h>
#include <ros/ros.h>

namespace Sim {

/**
 * @brief The Docking Station class
 * @details This component provides low level interface to the docking service.
 * The position and orientation of kira will be checked for each tick relative
 * to docking station plugin and color of docking station will be changed when
 * kira is docked. When there is a docking service call, pulsing of docking
 * station color will be taken place.
 */

class DockingStation : public ComponentVirtualBase {
public:
  /** @brief Constructor.
   *  @param ROS node to register the Service. */
  DockingStation(ros::NodeHandle &nh);

  static constexpr auto NAME{"docking_station"};
  static constexpr auto UPDATE_RATE{10};
  static constexpr sim_vector_t COLOR_BLUE{0, 0.5f, 1};
  // difference between front of kira and base_ref in linear.x
  static constexpr auto POSITION_DIFF{0.60417938232422};

  void RunOnce(unsigned t, unsigned dt) override;

  void Shutdown() override;

  const std::string_view Name() const override;

  /** @brief callback for docking serivce call
   *  @param provided service call request and service call response
   */
  bool DockingService(kira_msgs::Docking::Request &req,
                      kira_msgs::Docking::Response &res);

  /** @brief check the collision between docking station part and kira base
   */
  bool CheckCollision();

  /** @brief check the state of the docking
   */
  void CheckDocked();

  /** @brief check orientation of kira along z axis relative to docking station
   * plugin, i.e yaw
   */
  bool CheckOrientation();

  /** @brief Check the y position bound  of kira relative to docking station
   * plugin
   */
  bool CheckIsInBound();

  /** @brief Change the color of docking station when docking service is
   * called/kira is docked
   */
  void UpdateDockingColor();

  /** @brief Extraction of docking station part handles for color change
   * @param vector of string prefix
   */
  void ComponentHandleExtraction(
      std::vector<std::string_view> const &componentPrefixName);

  /** @brief load and place the docking station at the gievn position and
   * orientation into the simulation scene
   * @param path to the docking station model, given position and orientation of
   * docking station relative to kira base ref.
   */
  int AddDockingStation(const std::string &dockingStationPath);

private:
  ros::NodeHandle &m_nh;
  ros::ServiceServer m_dockingServer;
  ros::Publisher m_dockedPublisher;

  // thresholds
  float m_positionThreshold{};    // Unit is in meters
  float m_orientationThreshold{}; // unit is in radians

  /* handles from Coppelia Sim */
  int m_baseRefHandle{-1};
  int m_robotBaseHandle{-1};
  int m_tickCount{0};

  /** docking station handles */
  int m_dockingStationHandle{-1};
  int m_dockingStationPlugin{-1};
  int m_dockingStationCollision{-1};

  std::vector<int> m_handles;

  bool m_isdocked{false};
  bool m_pulse{false};

  sim_vector_t m_basePos{};
  sim_vector_t m_baseOrientation{};
  sim_vector_t m_colorBlack{};
  sim_vector_t m_dockingPoseArray{};
};
} // namespace Sim
#endif // DOCKING_STATION_HPP

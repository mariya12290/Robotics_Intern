/**
 * @file sim/cleaning.hpp
 * @brief This files creates and place the polygon into simulation
<<<<<<< HEAD
 * @author Surendra Kumar Aralapura Mariyappa
 * SurendraKumar.AralapuraMariyappa@de.kaercher.com
=======
 * @author Surendra Kumar Aralapura Mariyappa  SurendraKumar.AralapuraMariyappa@de.kaercher.com
 * @author Fedor Vlasov fedor.vlasov@de.kaercher.com
>>>>>>> bf44554f5290b6de569185c41a1bc285e50d1210
 * @copyright 2020 Alfred Kaercher SE & Co. KG
 */

#ifndef CLEANING_HPP
#define CLEANING_HPP

<<<<<<< HEAD
#include "sim/component_virtual_base.hpp"
#include "sim/utils.hpp"
#include <chrono>
#include <kira_msgs/Cleaning.h>
#include <kira_msgs/TankRinse.h>
#include <ros/ros.h>
=======
#include <ros/ros.h>
#include <chrono>
#include <kira_msgs/Cleaning.h>
#include <kira_msgs/LiftDriveControl.h>
#include <kira_msgs/TankRinse.h>
#include "sim/component_virtual_base.hpp"
#include "sim/utils.hpp"
>>>>>>> bf44554f5290b6de569185c41a1bc285e50d1210

namespace Sim {
/**
 * @brief The Cleaning class
<<<<<<< HEAD
 * @details Feature class for the visualization of cleaning service of KIRA 50
 * for different user input values.
 */

class Cleaning : public ComponentVirtualBase {
public:
  /** @brief Constructor.
   *  @param ROS node to register the Service. */
  Cleaning(ros::NodeHandle &nh);

  static constexpr auto NAME{"cleaning"};
  static constexpr auto UPDATE_RATE{10};
  static constexpr auto MAX_MOTOR_SPEED{0.08081f};
  static constexpr auto POLYGON_NAME{"water_polygon"};

  static constexpr auto VALUE_X{0.01919};
  static constexpr auto VALUE_Y{0.0025};
  static constexpr auto VALUE_Z{0.12};
  static constexpr sim_vector_t COLOR{0, 0, 1};
  static constexpr auto TIME{1000};

  /* To set the height of the added polygon just above the floor */
  static constexpr auto POLYGON_HEIGHT{0.0100035405159};
  static constexpr auto MASS{0};
  static constexpr sim_vector_t SIZE{0.8, 0.8, 0};
  static constexpr uint_fast8_t FLAGS{0b00010000};

  void RunOnce(unsigned t, unsigned dt) override;

  void Shutdown() override;

  const std::string_view Name() const override;

  /** @brief callback for cleaning serivce call
   *  @param provided service call request and service call response
   */
  bool CleaningService(kira_msgs::Cleaning::Request &req,
                       kira_msgs::Cleaning::Response &res);

  /** @brief callback for tank rinse serivce call
   *  @param provided service call request and service call response
   */
  bool TankRinseService(kira_msgs::TankRinse::Request &req,
                        kira_msgs::TankRinse::Response &res);

  /** @brief flashing of robot color for tank rinse service call
   */
  void TankRinse();

  /** @brief create a polygon for water and detergent, place at given position
   */
  void CreateAndPlacePolygon();

  /** @brief Activate cleaning gadgets when there is a cleaning serivce request
   *  @param cleaning gadgets (water, detergents,command, brush, broom) values
   */
  void EnableCleaning(const kira_msgs::CleaningParams &params);

  /** @brief Deactiavte all cleaning gadegts when there is cleaning serivce off
   * request
   */
  void DisableCleaning();

  /** @brief turn on and set the broom motor speed depending upon given value
   *  @param rotational speed of the broom joint
   */
  void TurnOnBroom(uint broom);

  /** @brief turn on and set the brush motor speed depending upon the given
   * value
   *  @param power of brush joint
   */
  void TurnOnBrush(uint brush);

  /** @brief Change the polygon opacity/transparency depending upon water
   *  @param percentage of water
   */
  void ColorWater(uint amount);

  /** @brief Change the polygon opacity/transparency depending upon detergent
   *  @param percentage of detergent
   */
  void ColorDetergent(float ratio);

  /** @brief transformation of  polygon depending upon detergent and water value
   *  @param percentage of detergent and water
   */
  void TransformPolygon(uint amount, float ratio);

  /** @brief Choosing the  water and detergent depending upon detergent and
   * water value
   *  @param percentage of detergent and water
   */
  void HandlePolygon(uint amount, float ratio);

  /** @brief To set up the brush joints speed
   *  @param provided speed
   */
  void SpeedUpBrushJoint(float speed);

  /** @brief Renaming the polygon
   */
  void RenamePolygon();

  /** @brief coloring polygon
   *  @param percentage of detergent and water
   */
  void ColorPolygon(uint amount, float ratio);

private:
  /** @brief
   *  @param  */
  ros::ServiceServer m_cleaningServer;
  ros::ServiceServer m_tankRinseServer;
  ros::ServiceServer m_liftDriveServer;

  ros::NodeHandle &m_nh;

  /* handles from Coppelia Sim */
  int m_broomMotorHandle{-1};
  int m_waterPolygonHandle{-1};
  int m_brushMotorHandleA{-1};
  int m_brushMotorHandleB{-1};
  int m_robotBaseHandle{-1};

  /** simulation time step  */
  int m_dt{};
  int m_tickCount{};
  bool m_clientCall{false};
  int m_tick{};

  sim_vector_t m_colorYellow{};
  /*properties of polygon */
  struct Polygon {
    float opacity{0}; /* for transparency */
    float speedBroom{0};
    float speedBrush{0};
    uint previousRatio{0};
    uint previousAmount{0};
    sim_vector_t color{0, 0, 1};
    sim_vector_t pos;
  } m_shape;
=======
 * @details Feature class for the visualization of cleaning service of KIRA 50 for different user
 * input values.
 */

class Cleaning : public ComponentVirtualBase
{
public:
    /** @brief Constructor.
     *  @param ROS node to register the Service. */
    Cleaning(ros::NodeHandle& nh);

    static constexpr auto NAME{"cleaning"};
    static constexpr auto UPDATE_RATE{10};
    static constexpr auto MAX_MOTOR_SPEED{0.08081f};
    static constexpr auto POLYGON_NAME{"water_polygon"};

    static constexpr auto VALUE_X{0.01919};
    static constexpr auto VALUE_Y{0.0025};
    static constexpr auto VALUE_Z{0.12};
    static constexpr sim_vector_t COLOR{0, 0, 1};
    static constexpr auto TIME{1000};

    /* To set the height of the added polygon just above the floor */
    static constexpr auto POLYGON_HEIGHT{0.0100035405159};
    static constexpr auto MASS{0};
    static constexpr sim_vector_t SIZE{0.8, 0.8, 0};
    static constexpr uint_fast8_t FLAGS{0b00010000};

    void RunOnce(unsigned t, unsigned dt) override;

    void Shutdown() override;

    const std::string_view Name() const override;

    /** @brief callback for cleaning serivce call
     *  @param provided service call request and service call response
     */
    bool CleaningService(kira_msgs::Cleaning::Request& req, kira_msgs::Cleaning::Response& res);

    /** @brief callback for lift drive control serivce call
     *  @param provided service call request and service call response
     */
    bool LiftDriveService(kira_msgs::LiftDriveControl::Request& req,
                          kira_msgs::LiftDriveControl::Response& res);

    /** @brief function to perfrom SqueeGee drive action
     *  @param provided service call request
     */
    void SqueeGeeDrive(const kira_msgs::LiftDriveControl::Request& req);

    /** @brief function to perfrom Brush drive action
     *  @param provided service call request
     */
    void BrushDrive(const kira_msgs::LiftDriveControl::Request& req);

    /** @brief function to perfrom Broom drive action
     *  @param provided service call request
     */
    void BroomDrive(const kira_msgs::LiftDriveControl::Request& req);

    /** @brief callback for tank rinse serivce call
     *  @param provided service call request and service call response
     */
    bool TankRinseService(kira_msgs::TankRinse::Request& req, kira_msgs::TankRinse::Response& res);

    /** @brief flashing of robot color for tank rinse service call
     */
    void TankRinse();

    /** @brief create a polygon for water and detergent, place at given position
     */
    void CreateAndPlacePolygon();

    /** @brief Activate cleaning gadgets when there is a cleaning serivce request
     *  @param cleaning gadgets (water, detergents,command, brush, broom) values
     */
    void EnableCleaning(const kira_msgs::CleaningParams& params);

    /** @brief Deactiavte all cleaning gadegts when there is cleaning serivce off request
     */
    void DisableCleaning();

    /** @brief turn on and set the broom motor speed depending upon given value
     *  @param rotational speed of the broom joint
     */
    void TurnOnBroom(uint broom);

    /** @brief turn on and set the brush motor speed depending upon the given value
     *  @param power of brush joint
     */
    void TurnOnBrush(uint brush);

    /** @brief Change the polygon opacity/transparency depending upon water
     *  @param percentage of water
     */
    void ColorWater(uint amount);

    /** @brief Change the polygon opacity/transparency depending upon detergent
     *  @param percentage of detergent
     */
    void ColorDetergent(float ratio);

    /** @brief transformation of  polygon depending upon detergent and water value
     *  @param percentage of detergent and water
     */
    void TransformPolygon(uint amount, float ratio);

    /** @brief Choosing the  water and detergent depending upon detergent and water value
     *  @param percentage of detergent and water
     */
    void HandlePolygon(uint amount, float ratio);

    /** @brief To set up the brush joints speed
     *  @param provided speed
     */
    void SpeedUpBrushJoint(float speed);

    /** @brief Renaming the polygon
     */
    void RenamePolygon();

    /** @brief coloring polygon
     *  @param percentage of detergent and water
     */
    void ColorPolygon(uint amount, float ratio);

private:
    /** @brief
     *  @param  */
    ros::ServiceServer m_cleaningServer;
    ros::ServiceServer m_tankRinseServer;
    ros::ServiceServer m_liftDriveServer;

    ros::NodeHandle& m_nh;

    /* handles from Coppelia Sim */
    int m_broomMotorHandle{-1};
    int m_waterPolygonHandle{-1};
    int m_brushMotorHandleA{-1};
    int m_brushMotorHandleB{-1};
    int m_robotBaseHandle{-1};

    /** simulation time step  */
    int m_dt{};
    int m_tickCount{};
    bool m_clientCall{false};
    int m_tick{};

    sim_vector_t m_colorYellow{};
    /*properties of polygon */
    struct Polygon
    {
        float opacity{0}; /* for transparency */
        float speedBroom{0};
        float speedBrush{0};
        uint previousRatio{0};
        uint previousAmount{0};
        sim_vector_t color{0, 0, 1};
        sim_vector_t pos;
    } m_shape;
>>>>>>> bf44554f5290b6de569185c41a1bc285e50d1210
};
} // namespace Sim
#endif // CLEANING_HPP

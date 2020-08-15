#include "sim/cleaning.hpp"
#include "sim/utils.hpp"
#include <kira_msgs/Cleaning.h>
#include <ros/ros.h>
#include <simLib.h>

namespace Sim {
Cleaning::Cleaning(ros::NodeHandle &nh)
    : ComponentVirtualBase(nh, UPDATE_RATE), m_nh{nh} {
  /** need to change the TIME here if needed */
  m_dt = static_cast<int>(simGetSimulationTimeStep() * TIME);
  m_robotBaseHandle = simGetObjectHandle("robot_base");
  m_broomMotorHandle = simGetObjectHandle("motor_broom");
  m_brushMotorHandleA = simGetObjectHandle("motor_brush_A");
  m_brushMotorHandleB = simGetObjectHandle("motor_brush_B");

  simGetShapeColor(m_robotBaseHandle, nullptr,
                   sim_colorcomponent_ambient_diffuse, m_colorYellow.data());
  m_cleaningServer = m_nh.advertiseService("cleaning_command",
                                           &Cleaning::CleaningService, this);
  m_tankRinseServer =
      m_nh.advertiseService("tank_rinse", &Cleaning::TankRinseService, this);
}

bool Cleaning::TankRinseService(kira_msgs::TankRinse::Request &req,
                                kira_msgs::TankRinse::Response &res) {
  std::chrono::duration<float> time{req.duration};
  std::chrono::duration<float, std::milli> timeSpan{time};
  auto duration_count = timeSpan.count();
  m_tick = static_cast<int>(duration_count / m_dt);
  m_clientCall = true;
  res.error_code = kira_msgs::TankRinse::Response::RINSE_NO_ERROR;
  return true;
}

void Cleaning::TankRinse() {
  if (m_clientCall) {
    m_tickCount += 1;
    if ((m_tickCount % 20) == 0) {
      simSetShapeColor(m_robotBaseHandle, nullptr,
                       sim_colorcomponent_ambient_diffuse, COLOR.data());
    } else if ((m_tickCount % 10) == 0) {

      simSetShapeColor(m_robotBaseHandle, nullptr,
                       sim_colorcomponent_ambient_diffuse,
                       m_colorYellow.data());
    }
  }
  if (m_tickCount == m_tick) {
    m_tickCount = 0;
    m_clientCall = false;
    simSetShapeColor(m_robotBaseHandle, nullptr,
                     sim_colorcomponent_ambient_diffuse, m_colorYellow.data());
    return;
  }
  return;
}

void Cleaning::RunOnce(unsigned, unsigned) { TankRinse(); }
const std::string_view Cleaning::Name() const { return NAME; }

bool Cleaning::CleaningService(kira_msgs::Cleaning::Request &req,
                               kira_msgs::Cleaning::Response &res) {
  if (req.command == kira_msgs::Cleaning::Request::CLEANING_OFF) {
    DisableCleaning();
  } else /* turn cleaning gadgets on with params */
  {
    EnableCleaning(req.params);
  }
  res.error_code = kira_msgs::Cleaning::Response::CLEANING_NO_ERROR;
  return true;
}

void Cleaning::EnableCleaning(const kira_msgs::CleaningParams &params) {
  HandlePolygon(params.water, params.detergent);
  TurnOnBroom(params.sidebroom);
  TurnOnBrush(params.brush);
}

void Cleaning::DisableCleaning() {
  TurnOnBrush(0u);
  TurnOnBroom(0u);
  HandlePolygon(0u, 0);
}

void Cleaning::Shutdown() {
  m_cleaningServer.shutdown();
  m_tankRinseServer.shutdown();
  m_liftDriveServer.shutdown();
}

void Cleaning::TurnOnBroom(uint broom) {
  m_shape.speedBroom = 0;
  if (broom) { /* turn on broom motor only when user requested */

    m_shape.speedBroom =
        MAX_MOTOR_SPEED + VALUE_X * static_cast<float>(std::min(broom, 100u));

    simSetJointTargetVelocity(m_broomMotorHandle, -m_shape.speedBroom);
  } else {

    simSetJointTargetVelocity(m_broomMotorHandle, m_shape.speedBroom);
  }
}
void Cleaning::TurnOnBrush(uint brush) {
  m_shape.speedBrush = 0;
  if (brush) { /* turn on brush motor only when user requested */

    m_shape.speedBrush =
        MAX_MOTOR_SPEED + VALUE_X * static_cast<float>(std::min(brush, 100u));

    SpeedUpBrushJoint(m_shape.speedBrush);
  } else {
    SpeedUpBrushJoint(m_shape.speedBrush);
  }
}
void Cleaning::CreateAndPlacePolygon() {
  m_waterPolygonHandle = simCreatePureShape(sim_pure_primitive_none, FLAGS,
                                            SIZE.data(), MASS, nullptr);
  simGetObjectPosition(GetBaseHandle(), -1, m_shape.pos.data());

  /* getting the position of base_ref dummy object in the simulation and
   * placing the created polygon at the same position of base_ref with reduced
   * height(z value). Finally estabilishing the parent and child relationship
   * between base_ref and added polygon.
   */
  m_shape.pos[2] = POLYGON_HEIGHT;
  m_shape.pos[0] = std::ceil(m_shape.pos[0]);
  simSetObjectPosition(m_waterPolygonHandle, -1, m_shape.pos.data());
  simSetObjectParent(m_waterPolygonHandle, GetBaseHandle(), true);
}

void Cleaning::ColorWater(uint amount) {
  // changing the polygon opacity when water is added

  m_shape.opacity = (static_cast<float>(std::min(amount, 100u))) * VALUE_Y;
  simSetShapeColor(m_waterPolygonHandle, nullptr,
                   sim_colorcomponent_transparency, &m_shape.opacity);
}

void Cleaning::ColorDetergent(float ratio) {
  // changing the polygon opacity when water is added

  m_shape.opacity = static_cast<float>(std::min(ratio, 2.5f) * VALUE_Z);
  simSetShapeColor(m_waterPolygonHandle, nullptr,
                   sim_colorcomponent_transparency, &m_shape.opacity);
}

void Cleaning::TransformPolygon(uint amount, float ratio) {
  if (amount && ratio) {
    /* transformation of polygon is relevant to both detergent and water value
     */
    if (!(m_shape.previousAmount == amount) &&
        !(m_shape.previousRatio == ratio)) {
      ColorWater(amount);
      m_shape.previousAmount = amount;
      ColorDetergent(ratio);
      m_shape.previousRatio = ratio;
    } else if (!(m_shape.previousRatio == ratio)) {
      ColorDetergent(ratio);
      m_shape.previousRatio = ratio;
    } else if (!(m_shape.previousAmount == amount)) {
      ColorWater(amount);
      m_shape.previousAmount = amount;
    }
  } else if (amount) {
    /* transformation of polygon is relevant to water value */
    ColorWater(amount);
    m_shape.previousAmount = amount;
  } else if (ratio) {
    /* transformation of polygon is relevant to detergent value */
    ColorDetergent(ratio);
    m_shape.previousRatio = ratio;
  }
}

void Cleaning::HandlePolygon(uint amount, float ratio) {
  if (amount || ratio) {
    if (not static_cast<bool>(simIsHandleValid(m_waterPolygonHandle, -1))) {
      CreateAndPlacePolygon();
      RenamePolygon();
    }
    ColorPolygon(amount, ratio);
    TransformPolygon(amount, ratio);
  } else {
    if (static_cast<bool>(simIsHandleValid(m_waterPolygonHandle, -1))) {
      simRemoveObject(m_waterPolygonHandle);
    }
  }
}

void Cleaning::ColorPolygon(uint amount, float ratio) {
  if (amount && ratio) {
    m_shape.color.at(0) = 0.6;
    m_shape.color.at(1) = 0.2;

    simSetShapeColor(m_waterPolygonHandle, nullptr,
                     sim_colorcomponent_ambient_diffuse, m_shape.color.data());
  } else if (amount) {
    m_shape.color.at(0) = 0;
    m_shape.color.at(1) = 0;
    simSetShapeColor(m_waterPolygonHandle, nullptr,
                     sim_colorcomponent_ambient_diffuse, m_shape.color.data());
  } else {
    m_shape.color.at(0) = 0.8;
    m_shape.color.at(1) = 0.2;
    simSetShapeColor(m_waterPolygonHandle, nullptr,
                     sim_colorcomponent_ambient_diffuse, m_shape.color.data());
  }
}

void Cleaning::SpeedUpBrushJoint(float speed) {
  simSetJointTargetVelocity(m_brushMotorHandleA, speed);
  simSetJointTargetVelocity(m_brushMotorHandleB, speed);
}

void Cleaning::RenamePolygon() {
  simSetObjectName(m_waterPolygonHandle, POLYGON_NAME);
}
} // namespace Sim

/**
 * @file sim/light_control_module
 * @brief This files provides simple light cotrol module implementation
 * @author Surendra Kumar Aralapura Mariyappa
 * SurendraKumar.AralapuraMariyappa@de.kaercher.com
 * @copyright 2020 Alfred Kaercher SE & Co. KG
 */

#ifndef LIGHT_CONTROL_MODULE_HPP
#define LIGHT_CONTROL_MODULE_HPP

#include "sim/component_virtual_base.hpp"
#include "sim/utils.hpp"
#include <chrono>
#include <kira_msgs/LedAnimation.h>
#include <ros/ros.h>

namespace Sim {
using sim_vector_l = std::array<int, 6>;
/**
 * @brief The Light control module class
 * @details light control module service API for allowing the client to call the
 * service according to operating modes. Operating modes are autonomous
 * mode(driving forward and backward, turning left and right), localization,
 * docking, emergency, standard mode, cleaning, obstacle detection, Leds on and
 * off.
 */
class LightControlModule : public ComponentVirtualBase {
public:
  /** @brief Constructor.
   *  @param ROS node to register the Service. */
  LightControlModule(ros::NodeHandle &nh);

  static constexpr auto NAME{"light_control_module"};
  static constexpr auto UPDATE_RATE{10};
  static constexpr uint_fast8_t FLAGS{0b000010000};
  static constexpr uint_fast8_t SHAPE{0b000000010};
  static constexpr auto MASS{0};
  static constexpr auto TIME{1000};

  /** we can add as many colors as we want, implementation is flexible */
  std::unordered_map<std::string, int> COLOR_OPTION{
      {"red", 1},    {"green", 2}, {"blue", 3},
      {"yellow", 4}, {"white", 5}, {"orange", 6}};

  void RunOnce(unsigned t, unsigned dt) override;

  void Shutdown() override;

  const std::string_view Name() const override;

  /** @brief member function to process the client request and to respond with
   * response after processing the request
   *  @param client request and response
   */
  bool AnimationServiceCallBack(kira_msgs::LedAnimation::Request &req,
                                kira_msgs::LedAnimation::Response &res);

  /** @brief creation of Leds in coppeliasim */
  void CreateAndPlaceLeds();

  /** @brief load the position and orientation for created leds in coppeliasim
   * from .yaml file */
  void LoadLedsDescription();

private:
  /** Led animation parameters */
  struct Animation {
    /** array of given leds */
    sim_vector_l leds{};
    /** given function either snap for blinking and color change and linear for
     * pulsing */
    std::string function{};
    /** given offset to synchronization  */
    uint offset{};
    /** RGB values for given leds states/colors */
    std::vector<std::array<float, 3>> colorValue{};
    /** given duration for leds state transition */
    uint tick{};
    /** flag used to change the state of leds */
    bool stateFlag{true};
    /** timer to count the ticks */
    uint tickCount{};
    /** helper variables used to change the state of leds in pulsing */
    uint colorIndex1{};
    uint colorIndex2{};
    /** helper variable used in count the half time for state transition */
    uint halfCount{};
  };

  struct Led {
    int id{};
    sim_vector_t pos{};
    sim_vector_t orientation{};
  };
  ros::NodeHandle &m_nh;
  ros::ServiceServer m_ledAnimationServer;

  bool m_clientCall{false};
  int m_callCount{};

  /** handles from coppeliasim */
  sim_vector_l m_ledsHandle{};

  /** simulation time step  */
  uint m_dt{};

  /** a dictionary to hold created animation/client call objects */
  /** the reason why map is used instead of unordered_map with compromising
   * efficiency, bcoz we need to remove the leds from previous client call
   * objects, if the led is present in new client call */
  std::map<std::string, Animation> m_serviceCall{};
  int m_tickCount{};
  sim_vector_t m_size{}; /**Leds size in m */
  std::array<Led, 6> m_leds{};

  /** @brief Extraction of RGB value from client given color name
   *  @param client given color name
   * @return RGB value of color
   */
  std::vector<std::array<float, 3>>
  ColorValueExtraction(std::vector<std::string> &colorTransition);

  /** @brief main function runs for each tick for the light control module
   */
  void LedAnimation();

  /** @brief instantiation of animation/client call object for each client
   * request
   *  @param client request parameters
   * @return animation/client call object
   */
  Animation AnimationObject(const kira_msgs::LedAnimationParams &params);

  /** @brief creation of animation objects and also deletion of former animation
   * objects if number of Leds in object is zero
   *  @param request parameters
   */
  void CreateAnimationObjects(const kira_msgs::LedAnimationParams &params);

  /** @brief function which will be executed when the client request is snap or
   * for flashing/blinking
   *  @param created animation or client call object and LEDs handle in
   * coppeliasim
   */
  void LedFlash(LightControlModule::Animation &animation,
                sim_vector_l ledsHandle);

  /** @brief function which will be executed when the client request is linear
   * or for pulsing
   *  @param created animation or client call object and LEDs handle in
   * coppeliasim
   */
  void LedPulse(LightControlModule::Animation &animation,
                sim_vector_l ledsHandle);

  /** @brief function which will be executed when the client request is snap or
   * just for color change
   *  @param created animation or client call object and LEDs handle in
   * coppeliasim
   */
  void LedColorChange(const LightControlModule::Animation &animation,
                      sim_vector_l ledsHandle);

  /** @brief a function to make the synchronization between two flash calls
   * @param animation object */
  bool Delay();
};
} // namespace Sim
#endif // LIGHT_CONTROL_MODULE_HPP

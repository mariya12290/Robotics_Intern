#include <boost/algorithm/string.hpp>
#include <numeric>
#include "light_control_module.hpp"
#include "XmlRpcException.h"
#include "kira_lib/utils/utils.hpp"

namespace Sim {

using namespace kira_lib;

LightControlModule::LightControlModule(ros::NodeHandle& nh) :
    ComponentVirtualBase(nh, UPDATE_RATE), m_nh{nh}
{
    m_dt = static_cast<unsigned>(simGetSimulationTimeStep() * TIME);
    m_ledsHandle.fill(-1);
    LoadLedsDescription();
    CreateAndPlaceLeds();
    m_ledAnimationServer =
        m_nh.advertiseService("light_control", &LightControlModule::AnimationServiceCallBack, this);
}

const std::string_view LightControlModule::Name() const
{
    return NAME;
}
void LightControlModule::Shutdown()
{
    m_ledAnimationServer.shutdown();
}

bool LightControlModule::AnimationServiceCallBack(kira_msgs::LedAnimation::Request& req,
                                                  kira_msgs::LedAnimation::Response& res)
{
    m_clientCall = true;
    m_callCount += 1;
    CreateAnimationObjects(req.animation);
    res.error_code = kira_msgs::LedAnimation::Response::ANIMATION_NO_ERROR;
    return true;
}

void LightControlModule::CreateAnimationObjects(const kira_msgs::LedAnimationParams& params)
{
    if (m_callCount == 1)
    {
        Animation animation{AnimationObject(params)};
        std::pair<std::string, Animation> callObject("call" + std::to_string(m_callCount),
                                                     animation);
        m_serviceCall.insert(callObject);
        return;
    }
    else
    { // remove leds from old client call object if it is present in new client request call
        for (auto& i : m_serviceCall)
        {
            for (const auto& j : params.leds)
            {
                auto result = std::find(i.second.leds.begin(), i.second.leds.end(), j);
                if (result != i.second.leds.end())
                {
                    std::replace(i.second.leds.begin(), i.second.leds.end(), *result, 0);
                }
                else
                {
                    continue;
                }
            }
        }
    }
    // delete animation object, if it contains no leds or zero leds
    for (auto it = m_serviceCall.begin(); it != m_serviceCall.end();)
    {
        if (std::all_of(it->second.leds.begin(), it->second.leds.end(),
                        [&](int value) { return value == 0; }))

        {
            it = m_serviceCall.erase(it);
        }
        else
        {
            ++it;
        }
    }
    Animation animation{AnimationObject(params)};
    std::pair<std::string, Animation> callObject("call" + std::to_string(m_callCount), animation);
    m_serviceCall.insert(callObject);
    return;
}

LightControlModule::Animation
LightControlModule::AnimationObject(const kira_msgs::LedAnimationParams& params)
{
    Animation animation;

    // input data validation is not implemented
    /** for an example, if the length of the leds is greater than 6 or if the specified leds number
     * is greater than 6. */
    std::copy(params.leds.begin(), params.leds.end(), animation.leds.begin());
    std::vector<std::string> color_transition{};
    color_transition.resize(params.state.size());
    std::copy(params.state.begin(), params.state.end(), color_transition.begin());
    std::transform(color_transition.begin(), color_transition.end(), color_transition.begin(),
                   [](std::string str) { return str; });
    animation.function = params.function;

    /** we can use directly duration and offset in seconds or in milliseconds. Since in simulation
     * we consider time step, duration and offset are converted into timestep */
    std::chrono::duration<float> time{params.duration};
    std::chrono::duration<float> offset{params.offset};

    std::chrono::duration<float, std::milli> timeSpan{time};
    auto duration_count = timeSpan.count();
    animation.tick = static_cast<int>(duration_count / m_dt);

    std::chrono::duration<float, std::milli> timeSpanOffset{offset};
    auto offset_count = timeSpanOffset.count();
    animation.offset = static_cast<int>(offset_count / m_dt);

    boost::to_lower(animation.function);

    animation.colorValue = ColorValueExtraction(color_transition);
    animation.colorValue.shrink_to_fit();
    return animation;
}

std::vector<std::array<float, 3>>
LightControlModule::ColorValueExtraction(std::vector<std::string>& colorTransition)
{
    std::vector<std::array<float, 3>> color{};
    auto option = [&](std::string str) {
        return (COLOR_OPTION.find(str) != COLOR_OPTION.end() ? (COLOR_OPTION.find(str)->second)
                                                             : 0);
    };
    for (auto i{0u}; i < colorTransition.size(); i++)
    {
        switch (option(colorTransition[i]))
        {
        case 1:
            color.push_back({1, 0, 0}); // In vector of arrays, we can not use emplace_back
            break;
        case 2:
            color.push_back({0, 1, 0});
            break;
        case 3:
            color.push_back({0, 0, 1});
            break;
        case 4:
            color.push_back({1, 1, 0});
            break;
        case 5:
            color.push_back({1, 1, 1});
            break;
        case 6:
            color.push_back({1, 0.5, 0});
            break;
        default:
            color.push_back({0, 0, 0});
            ROS_WARN("RGB value for given color is not defined/ state transition is not given ");
            break;
        }
    }
    return color;
}

void LightControlModule::RunOnce(unsigned, unsigned)
{
    LedAnimation();
}

void LightControlModule::LedAnimation()
{
    for (auto& i : m_serviceCall)
    {
        LedColorChange(i.second, m_ledsHandle);
        LedFlash(i.second, m_ledsHandle);
        LedPulse(i.second, m_ledsHandle);
    }
}
/** new flashing/blinking client call will always synchronize with the previous flashing/blinking
client call */
bool LightControlModule::Delay()
{
    /** it is also possible to synchronize with second color instead of first */
    return m_tickCount == 0;
}

void LightControlModule::LedFlash(LightControlModule::Animation& animation, sim_vector_l ledsHandle)
{
    if (animation.offset && animation.function == "snap" && animation.colorValue.size() == 2 &&
        m_clientCall)
    {
        animation.tickCount += 1;
        if (animation.tickCount == animation.offset)
        {
            animation.stateFlag = false;
            animation.offset = 0;
            if (!Delay())
            {
                animation.stateFlag = true;
                return;
            }
        }
        else
        {
            return;
        }
    }
    else if (animation.function == "snap" && animation.stateFlag)
    {
        if (Delay())
        {
            animation.stateFlag = false;
            animation.tickCount = 0;
        }
        else
        {
            return;
        }
    }

    /**  increasing the duration count of previous blinking client calls is also possible in oder to
     * synchronize with the new blinking client call through creating separate container for
     * blinking client call objects and check the size of the container for everytick, if size
     * increased which means, duration count of previous blinking client objects should be increased
     * for the synchronization, while increasing duration count we need to take offset of new
     * blinking client call into account */

    if (animation.colorValue.size() == 2 && m_clientCall && animation.function == "snap")
    {
        auto result =
            static_cast<float>(animation.tick) / static_cast<float>(animation.colorValue.size());
        if (animation.tickCount == animation.tick || animation.tickCount == 0)
        {
            animation.colorIndex1 = 0;
            animation.tickCount = 0;
            animation.halfCount = 0; 
        }
        else if ((animation.halfCount % static_cast<int>(std::ceil(result))) == 0)
        {
            animation.colorIndex1 = 1;
            animation.halfCount = 0;
        }

        std::for_each(animation.leds.begin(), animation.leds.end(), [&](int value) {
            simSetShapeColor(ledsHandle[value - 1], nullptr, sim_colorcomponent_ambient_diffuse,
                             animation.colorValue[animation.colorIndex1].data());
        });
        m_tickCount = animation.tickCount;
        animation.tickCount += 1;
        animation.halfCount += 1;
    }
}

void LightControlModule::LedColorChange(const LightControlModule::Animation& animation,
                                        sim_vector_l ledsHandle)
{
    if (animation.colorValue.size() == 1 && m_clientCall && animation.function == "snap")
    {
        {
            std::for_each(animation.leds.begin(), animation.leds.end(), [&](int value) {
                simSetShapeColor(ledsHandle[value - 1], nullptr, sim_colorcomponent_ambient_diffuse,
                                 animation.colorValue[0].data());
            });
        }
    }
}

/** It is also possible to use the offset in plusing, which means to say wait for certain offset and
 * then start */
void LightControlModule::LedPulse(Animation& animation, sim_vector_l ledsHandle)
{
    if (animation.function == "linear" && m_clientCall)
    {
        auto result =
            static_cast<float>(animation.tick) / static_cast<float>(animation.colorValue.size());
        if (animation.tickCount == animation.tick || animation.tickCount == 0)
        {
            animation.colorIndex1 = 0;
            animation.tickCount = 0;
            animation.halfCount = 0;
            animation.colorIndex2 = animation.colorIndex1 + 1;
        }
        else if (animation.tickCount != animation.tick)
        {
            if (static_cast<int>(animation.halfCount % static_cast<int>(std::ceil(result))) == 0)
            {
                animation.colorIndex1 += 1;
                animation.colorIndex2 = animation.colorIndex1 + 1;
                animation.halfCount = 0;
                if (animation.colorIndex1 >= (animation.colorValue.size() - 1))
                {
                    animation.colorIndex2 = 0;
                }
            }
        }
        animation.tickCount += 1;
        animation.halfCount += 1; 

        auto timeFraction = static_cast<float>(animation.halfCount) / static_cast<float>(result);

        sim_vector_t rgb;

        /** leds state transition in plusing for an example ['red','green','blue'] ->
         * red->green->blue->red->continuation according to CSS framework*/

        rgb[0] = animation.colorValue[animation.colorIndex1][0] * (1.0 - timeFraction) +
                 (animation.colorValue[animation.colorIndex2][0] * timeFraction);

        rgb[1] = animation.colorValue[animation.colorIndex1][1] * (1.0 - timeFraction) +
                 (animation.colorValue[animation.colorIndex2][1] * timeFraction);

        rgb[2] = animation.colorValue[animation.colorIndex1][2] * (1.0 - timeFraction) +
                 (animation.colorValue[animation.colorIndex2][2] * timeFraction);
        std::for_each(animation.leds.begin(), animation.leds.end(), [&](int value) {
            simSetShapeColor(ledsHandle[value - 1], nullptr, sim_colorcomponent_ambient_diffuse,
                             rgb.data());
        });
    }
}

void LightControlModule::LoadLedsDescription()
{
    XmlRpc::XmlRpcValue leds;
    Led led;
    if (not Utils::LoadRosParam(m_nh, "led_size", m_size) or
        not Utils::LoadRosParam(m_nh, "leds", leds, ::ros::console::Level::Debug))
    {
        /* when no leds are set they are not needed, silently continue */
        return;
    }
    try
    {
        for (auto i{0}; i < leds.size(); ++i)
        {
            XmlRpc::XmlRpcValue& sublist = leds[i];
            led.id = sublist["code"];
            led.pos[0] = static_cast<float>(static_cast<double>(sublist["xyz"][0]));
            led.pos[1] = static_cast<float>(static_cast<double>(sublist["xyz"][1]));
            led.pos[2] = static_cast<float>(static_cast<double>(sublist["xyz"][2]));
            led.orientation[0] = static_cast<float>(static_cast<double>(sublist["rpy"][0]));
            led.orientation[1] = static_cast<float>(static_cast<double>(sublist["rpy"][1]));
            led.orientation[2] = static_cast<float>(static_cast<double>(sublist["rpy"][2]));
            m_leds[i] = led;
        }
    }
    catch (const XmlRpc::XmlRpcException&)
    {
        ROS_WARN("could not load leds position and orientation ");
    }
}

void LightControlModule::CreateAndPlaceLeds()
{
    auto ledHandle = simCreateDummy(0, nullptr);
    if (ledHandle == -1)
    {
        ROS_WARN(" couldn't create dummy shape ");
        return;
    }
    else
    {
        sim_vector_t pos;
        simGetObjectPosition(GetBaseHandle(), -1, pos.data());
        simSetObjectPosition(ledHandle, -1, pos.data());
        simSetObjectName(ledHandle, "Leds");
        simSetObjectParent(ledHandle, simGetObjectHandle("kira_b50"), 1);
    }

    for (uint i = 0; i < m_leds.size(); ++i)
    {
        m_ledsHandle[i] = simCreatePureShape(SHAPE, FLAGS, m_size.data(), MASS, nullptr);
        if (m_ledsHandle[i] == -1)
        {
            ROS_WARN(" couldn't load leds description file");
        }
        else
        {
            simSetObjectName(m_ledsHandle[i], ("led" + std::to_string(m_leds[i].id)).c_str());
            simSetObjectPosition(m_ledsHandle[i], GetBaseHandle(), m_leds[i].pos.data());
            simSetObjectOrientation(m_ledsHandle[i], GetBaseHandle(), m_leds[i].orientation.data());
            simSetObjectParent(m_ledsHandle[i], GetBaseHandle(), true);
        }
    }
}
} // namespace Sim

#pragma once

// main XBot2 include
#include <xbot2/xbot2.h>
#include <matlogger2/matlogger2.h>

#include <xbot2/ros/ros2_support.h>
#include <std_srvs/srv/set_bool.hpp>

#include <atomic>

#include <sander_device.h>


using namespace XBot;

class SanderControl : public ControlPlugin
{

public:

    // we don't do anything special inside the
    // constructor, so just inherit the base class
    // implementation
    using ControlPlugin::ControlPlugin;

    // initialization method; the plugin won't be run
    // if this returns 'false'
    bool on_initialize() override;

    // callback for switching to the 'Starting' state
    void on_start() override;

    // callback for 'Run' state
    void run() override;
    

private:
    
    XBot::MatLogger2::Ptr _logger;

    Ros2Support::UniquePtr _ros;

    bool trig_srv_handler(const std_srvs::srv::SetBool::Request& req,
        std_srvs::srv::SetBool::Response& res);

    CallbackQueue _queue;

    ServiceServerPtr<std_srvs::srv::SetBool::Request,
                     std_srvs::srv::SetBool::Response> _trig_srv;

    bool _sander_activated = false;
    std::atomic<bool> _sander_control;

    XBot::Hal::SanderBase* _sander;

};


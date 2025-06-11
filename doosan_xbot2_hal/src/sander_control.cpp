#include "sander_control.h"


bool SanderControl::trig_srv_handler(const std_srvs::srv::SetBool::Request & req,
    std_srvs::srv::SetBool::Response & res) {

    // print service handling message with cyan color for visibility
    jhigh().jprint(fmt::fg(fmt::terminal_color::cyan),
    "handling SetBool service call..\n");

    // set appropriate response fields
    res.message = "gripper control ok";
    res.success = true;

    _sander_control.store((bool)req.data);

    return true;
}


bool SanderControl::on_initialize()
{
    /* Create logger */
    XBot::MatLogger2::Options logger_opt;
    logger_opt.default_buffer_size = 1e6;
    _logger = XBot::MatLogger2::MakeLogger("/tmp/sander_control_log", logger_opt);
    _logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

    // ros2 
    auto node = Ros2Support::get_main_node()->create_sub_node(getName());
    _ros = std::make_unique<Ros2Support>(node);

    int queue_size = getParamOr<int>("~queue_size", 1);

    _trig_srv = _ros->advertiseService<std_srvs::srv::SetBool>("activate_gripper",
        &SanderControl::trig_srv_handler,
        this,
        &_queue);

    _sander_control.store(_sander_activated);

    _sander = _robot->getDeviceInstance<XBot::Hal::SanderBase>("sander_0");

    //setDefaultControlMode(ControlMode::Idle());

    return true;
}

void SanderControl::on_start()
{

}


void SanderControl::run()
{
    _queue.run();

    if(_sander_activated != _sander_control.load()) {
        _sander->activate_sander(_sander_control.load());

        _sander_activated = _sander_control.load();

        _sander->move();
    }

}

XBOT2_REGISTER_PLUGIN(SanderControl, sander_control)

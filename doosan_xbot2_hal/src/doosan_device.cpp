#include <xbot2/journal/journal.h>

#include <doosan_device_client.h>
#include <doosan_device_driver.h>

#define DEG_TO_RAD(deg) (deg * (2 * M_PI) / 360.0) // From deg to rad
#define RAD_TO_DEG(rad) (rad * 360.0 / (2 * M_PI)) // From rad to deg

#define DOOSAN_IP "192.168.137.100" 

//bool XBot::Hal::DoosanDriverContainer::g_bHasControlAuthority = true;
//DRAFramework::CDRFLEx XBot::Hal::DoosanDriverContainer::_drfl;

float XBot::Hal::DoosanDriver::_doosan_q[JOINTS] = {0.0, };
float XBot::Hal::DoosanDriver::_doosan_qref[JOINTS] = {0.0, };
float XBot::Hal::DoosanDriver::_doosan_qref_prev[JOINTS] = { 0.0, };

/*
 * Client side implementation (i.e. used by plugin implementers)
 */

// JointBase
double
XBot::Hal::DoosanClient::get_link_pos() const
{
    return _rx.position;
}

double XBot::Hal::DoosanClient::get_motor_pos() const
{
    return _rx.position;
}

double XBot::Hal::DoosanClient::get_link_vel() const
{
    return _rx.velocity;
}

double XBot::Hal::DoosanClient::get_motor_vel() const
{
    return _rx.velocity;
}

double XBot::Hal::DoosanClient::get_tor() const
{
    return _rx.torque;
}

double XBot::Hal::DoosanClient::get_stiffness() const
{
    return 0.0;
}

double XBot::Hal::DoosanClient::get_damping() const
{
    return 0.0;
}

double XBot::Hal::DoosanClient::get_temp() const
{
    return 0.0;
}

double XBot::Hal::DoosanClient::get_pos_ref() const
{
    return _rx.position_ref;
}

double XBot::Hal::DoosanClient::get_vel_ref() const
{
    return 0.0;
}

double XBot::Hal::DoosanClient::get_tor_ref() const
{
    return 0.0;
}

double XBot::Hal::DoosanClient::get_stiffness_ref() const
{
    return 0.0; // TBD ref values here
}

double XBot::Hal::DoosanClient::get_damping_ref() const
{
    return 0.0; // TBD ref values here
}

void XBot::Hal::DoosanClient::set_pos_ref(double q)
{
    _tx.position_ref = q;
}

void XBot::Hal::DoosanClient::set_vel_ref(double q)
{
    _tx.velocity_ref = q;
}

void XBot::Hal::DoosanClient::set_tor_ref(double q)
{
    _tx.torque_ref = q;
}

void XBot::Hal::DoosanClient::set_stiffness_ref(double q)
{
    // TBD set _tx
}

void XBot::Hal::DoosanClient::set_damping_ref(double q)
{
    // TBD set _tx
}

/*
void XBot::Hal::DoosanDriverContainer::OnLogAlarm(LPLOG_ALARM tLog)
{
    cout << "Alarm Info: "
         << "group(" << (unsigned int)tLog->_iGroup << "), index("
         << tLog->_iIndex << "), param(" << tLog->_szParam[0] << "), param("
         << tLog->_szParam[1] << "), param(" << tLog->_szParam[2] << ")" << endl;
}*/

/*
void XBot::Hal::DoosanDriverContainer::OnMonitoringStateCB(const ROBOT_STATE eState)
{
    cout << "current state: " << (int)eState << endl;

    switch ((unsigned char)eState)
    {

    case STATE_EMERGENCY_STOP:
        // popup
        break;
    case STATE_STANDBY:
    case STATE_MOVING:
    case STATE_TEACHING:
        break;
    case STATE_SAFE_STOP:
        if (g_bHasControlAuthority)
        {
            _drfl.SetSafeStopResetType(SAFE_STOP_RESET_TYPE_DEFAULT);
            _drfl.SetRobotControl(CONTROL_RESET_SAFET_STOP);
        }
        break;
    case STATE_SAFE_OFF:
        cout << "STATE_SAFE_OFF1" << endl;
        if (g_bHasControlAuthority)
        {
            cout << "STATE_SAFE_OFF2" << endl;
            _drfl.SetRobotControl(CONTROL_SERVO_ON);
        }
        break;
    case STATE_SAFE_STOP2:
        if (g_bHasControlAuthority)
            _drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_STOP);
        break;
    case STATE_SAFE_OFF2:
        if (g_bHasControlAuthority)
        {
            _drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_OFF);
        }
        break;
    case STATE_RECOVERY:
        _drfl.SetRobotControl(CONTROL_RESET_RECOVERY);
        break;
    default:
        break;
    }
    return;
}*/

/*
 * Driver side implementation (i.e. interfacing with the Doosan)
 */
XBot::Hal::DoosanDriverContainer::DoosanDriverContainer(std::vector<DeviceInfo> devinfo,
                                                        const Device::CommonParams &p) : DeviceContainer(devinfo, p)
{
    // TBD register callback and start connection if needed
    //_drfl.set_on_monitoring_state(XBot::Hal::DoosanDriverContainer::OnMonitoringStateCB);
    //_drfl.set_on_log_alarm(XBot::Hal::DoosanDriverContainer::OnLogAlarm);


    Context().journal().jhigh().jinfo("Ready to go! \n");

    // DRFL initialization
    bool connect_ok = _drfl.open_connection(DOOSAN_IP);

    Context().journal().jhigh().jinfo("Connection Open! \n");


    // obtain the control from the external PC
    bool external_control_ok = _drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);

    _drfl.setup_monitoring_version(1);
    Context().journal().jhigh().jinfo("Library version: {}\n", _drfl.get_library_version());

    // servo on
    _drfl.set_robot_control(CONTROL_SERVO_ON);
    //_drfl.set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX_10, TRUE);

    // TBD proper management of the doosan state machine
    while ((_drfl.get_robot_state() != STATE_STANDBY) /*|| !g_bHasControlAuthority*/) {

        Context().journal().jhigh().jinfo("Connect is ok? {} - state: {} - External control? {}\n", 
            connect_ok, 
            _drfl.get_robot_state(),
            external_control_ok);
        this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    // robot mode MANUAL on REAL doosan
    _drfl.set_robot_mode(ROBOT_MODE_MANUAL);
    _drfl.set_robot_system(ROBOT_SYSTEM_REAL);



    Context().journal().jhigh().jinfo("Found {} devices from config file\n", devinfo.size());

    if (devinfo.empty())
    {
        // create devices TBD generalize it from config file
        for (size_t i = 1; i <= JOINTS; i++)
        {
            // TBD autodetection
            DeviceInfo di;
            di.id = i;
            di.name = "joint" + std::to_string(i);
            di.type = "doosan_device";

            Context().journal().jhigh().jok("detected {} '{}' with id {} \n",
                                            di.type, di.name, di.id);

            auto d = std::make_shared<DoosanDriver>(di, p);

            addDevice(d);

            // pass the drfl static object to the driver
            d->initialize(_drfl);
        }
    }


    // connect to RT control
    _drfl.connect_rt_control();
    //sleep(1);

    // info for rt control
    cout << _drfl.get_rt_control_input_data_list(_drfl.get_rt_control_input_version_list()) << endl;
    cout << _drfl.get_rt_control_output_data_list(_drfl.get_rt_control_output_version_list()) << endl;
    
    //sleep(1);
    //_drfl.set_on_rt_monitoring_data(OnRTMonitoringData);
    //sleep(1);

    // rt control configuration
    _drfl.set_rt_control_input("v1.0", 0.01, 10);
    _drfl.set_rt_control_output("v1.0", 0.01, 10);
    //sleep(1);

    // start RT control
    _drfl.start_rt_control();

}

XBot::Hal::DoosanDriver::DoosanDriver(DeviceInfo di,
                                      const CommonParams &params) : DeviceDriverTpl(di, params),
                                                                    _safety(di, get_period_sec(), JointSafety::force_safety_flag::safety_required)
{

    // get param manager
    auto &pm = Context().paramManager();

    // do initialization

    for (int i = 0; i < NUMBER_OF_JOINT; i++)
    {
        _q_dot_d[i] = -10000;
        _q_ddot_d[i] = -10000;
    }

    // register the device for Position, Velocity and Effort control modes
    register_resource(JointBase::Resource::Position,
                      JointBase::Resource::Mask::Position);
    register_resource(JointBase::Resource::Velocity,
                      JointBase::Resource::Mask::Velocity);
    register_resource(JointBase::Resource::Effort,
                      JointBase::Resource::Mask::Effort);

    Context().journal().jhigh().jok("connected to {} with name '{}' and period '{} s' \n", di.type, di.name, get_period_sec());
}

void XBot::Hal::DoosanDriver::initialize(DRAFramework::CDRFLEx& drfl) {
    _drfl = drfl;
}

bool XBot::Hal::DoosanDriver::sense_impl()
{
    // receive from DRFL TBD optimize from the container
    bool recv_ok = true;
    memcpy(_doosan_q, _drfl.read_data_rt()->actual_joint_position, sizeof(float) * JOINTS);

    // transfer the data from Doosan to XBot packet TBD!
    _rx.position = DEG_TO_RAD(_doosan_q[get_id() - 1]);

    // init tx values only at the first valid recv
    if (recv_ok && !_tx_initialized)
    {

        _tx.position_ref = _rx.position;
        _tx.velocity_ref = 0.0;
        _tx.torque_ref = 0.0;

        // HACK do a copy to xbot joint to use safety
        _rx_xbot.link_pos = _rx.position;
        _rx_xbot.motor_pos = _rx.position;
        _rx_xbot.link_vel = _rx.velocity;
        _rx_xbot.motor_vel = _rx.velocity;
        _rx_xbot.torque = _rx.torque;

        _rx_xbot.pos_ref = _tx.position_ref;
        _rx_xbot.vel_ref = _tx.velocity_ref;
        _rx_xbot.tor_ref = _tx.torque_ref;
        _safety.initialize(_rx_xbot);

        _tx_initialized = true;
    }

    // get references on rx
    _rx.position_ref = _tx.position_ref;
    _rx.velocity_ref = _tx.velocity_ref;
    _rx.torque_ref = _tx.torque_ref;

    return recv_ok;
}

bool XBot::Hal::DoosanDriver::move_impl()
{

    // check safety copying to xbot tx data structure
    _tx_xbot.pos_ref = _tx.position_ref; // TBD check it
    _tx_xbot.vel_ref = 0.0;              // TBD check it
    _tx_xbot.tor_ref = 0.0;              // TBD check it

    _tx_xbot.mask = 1; // TBD check the mask and use the callback on_rx_recv

    if (!_safety.enforce(_tx_xbot, _tx_xbot_safe))
    {
        // if enforce is false means that we need to use the safe tx
        _tx.position_ref = _tx_xbot_safe.pos_ref;
        _tx.velocity_ref = _tx_xbot_safe.vel_ref;
        _tx.torque_ref = _tx_xbot_safe.tor_ref;
    }

    // transfer the data from safe XBot packet (_tx_xbot_safe) to Doosan
    _doosan_qref[get_id() - 1] = RAD_TO_DEG(_tx_xbot_safe.pos_ref);

    bool send_ok = true;
    // send to doosan
    if (get_id() == 6){
        send_ok = _drfl.servoj_rt(_doosan_qref, _q_dot_d, _q_ddot_d, 0.0);
    }

    return send_ok;
}

XBOT2_REGISTER_DEVICE(XBot::Hal::DoosanDriverContainer,
                      XBot::Hal::DeviceContainer<XBot::Hal::DoosanClient>,
                      doosan)

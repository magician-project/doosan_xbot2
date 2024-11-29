#include <xbot2/journal/journal.h>

#include <doosan_device_client.h>
#include <doosan_device_driver.h>

#define DEG_TO_RAD(deg) (deg * (2 * M_PI) / 360.0) // From deg to rad
#define RAD_TO_DEG(rad) (rad * 360.0 / (2 * M_PI)) // From rad to deg

#define DOOSAN_IP "192.168.137.100"

XBot::Hal::DoosanDriverContainer *XBot::Hal::DoosanDriverContainer::_instance = nullptr;

/*
 * Client side implementation (i.e. used by plugin implementers)
 */

// JointBase
double XBot::Hal::DoosanClient::get_link_pos() const
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
    return _rx.stiffness;
}

double XBot::Hal::DoosanClient::get_damping() const
{
    return _rx.damping;
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
    return _rx.velocity_ref;
}

double XBot::Hal::DoosanClient::get_tor_ref() const
{
    return _rx.torque_ref;
}

double XBot::Hal::DoosanClient::get_stiffness_ref() const
{
    return _rx.stiffness; 
}

double XBot::Hal::DoosanClient::get_damping_ref() const
{
    return _rx.damping; 
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
    _tx.stiffness_ref = q;
}

void XBot::Hal::DoosanClient::set_damping_ref(double q)
{
    _tx.damping_ref = q;
}

void XBot::Hal::DoosanDriverContainer::StaticLogAlarmCB(LPLOG_ALARM tLog)
{
    if (_instance)
    {
        _instance->OnLogAlarm(tLog);
    }
}

void XBot::Hal::DoosanDriverContainer::StaticMonitoringStateCB(const ROBOT_STATE eState)
{
    if (_instance)
    {
        _instance->OnMonitoringStateCB(eState);
    }
}

void XBot::Hal::DoosanDriverContainer::StaticMonitroingAccessControlCB(
    const MONITORING_ACCESS_CONTROL eTrasnsitControl)
{
    if (_instance)
    {
        _instance->OnMonitroingAccessControlCB(eTrasnsitControl);
    }
}

void XBot::Hal::DoosanDriverContainer::StaticTpInitializingCompleted()
{
    if (_instance)
    {
        _instance->OnTpInitializingCompleted();
    }
}

void XBot::Hal::DoosanDriverContainer::StaticRTMonitoringData(LPRT_OUTPUT_DATA_LIST tData)
{
    if (_instance)
    {
        _instance->OnRTMonitoringData(tData);
    }
}

void XBot::Hal::DoosanDriverContainer::OnLogAlarm(LPLOG_ALARM tLog)
{
    cout << "Alarm Info: "
         << "group(" << (unsigned int)tLog->_iGroup << "), index("
         << tLog->_iIndex << "), param(" << tLog->_szParam[0] << "), param("
         << tLog->_szParam[1] << "), param(" << tLog->_szParam[2] << ")" << endl;
}

void XBot::Hal::DoosanDriverContainer::OnMonitoringStateCB(const ROBOT_STATE eState)
{
    //cout << "current state: " << (int)eState << endl;

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
        //cout << "STATE_SAFE_OFF1" << endl;
        if (g_bHasControlAuthority)
        {
            //cout << "STATE_SAFE_OFF2" << endl;
            //_drfl.SetRobotControl(CONTROL_SERVO_ON); // TBD check if really needed
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
        //_drfl.SetRobotControl(CONTROL_RESET_RECOVERY);
        break;
    default:
        break;
    }
    return;
}

void XBot::Hal::DoosanDriverContainer::OnMonitroingAccessControlCB(
    const MONITORING_ACCESS_CONTROL eTrasnsitControl)
{
    std::cout << eTrasnsitControl << std::endl; // TBD lose time to have control
    //sleep(1);
    switch (eTrasnsitControl)
    {
    case MONITORING_ACCESS_CONTROL_REQUEST:
        _drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_NO);
        // Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_YES);
        break;
    case MONITORING_ACCESS_CONTROL_GRANT:
        g_bHasControlAuthority = TRUE;
        // cout << "GRANT1" << endl;
        // cout << "MONITORINGCB : " << (int)Drfl.GetRobotState() << endl;
        OnMonitoringStateCB(_drfl.GetRobotState());
        // cout << "GRANT2" << endl;
        break;
    case MONITORING_ACCESS_CONTROL_DENY:
    case MONITORING_ACCESS_CONTROL_LOSS:
        g_bHasControlAuthority = FALSE;
        if (g_TpInitailizingComplted)
        {
            // assert(Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_REQUEST));
            _drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
        }
        break;
    default:
        break;
    }
}

void XBot::Hal::DoosanDriverContainer::OnTpInitializingCompleted()
{
    g_TpInitailizingComplted = TRUE;
    _drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
}

void XBot::Hal::DoosanDriverContainer::OnRTMonitoringData(LPRT_OUTPUT_DATA_LIST tData)
{
    /*
    printf("timestamp : %.3f\n", tData->time_stamp);
    printf("actual_joint_position : %f %f %f %f %f %f\n", tData->actual_joint_position[0], tData->actual_joint_position[1], tData->actual_joint_position[2], tData->actual_joint_position[3], tData->actual_joint_position[4], tData->actual_joint_position[5]);
    printf("actual_motor_torque : %f %f %f %f %f %f\n", tData->actual_motor_torque[0], tData->actual_motor_torque[1], tData->actual_motor_torque[2], tData->actual_motor_torque[3], tData->actual_motor_torque[4], tData->actual_motor_torque[5]);
    printf("actual_grav_torque : %f %f %f %f %f %f\n", tData->gravity_torque[0], tData->gravity_torque[1], tData->gravity_torque[2], tData->gravity_torque[3], tData->gravity_torque[4], tData->gravity_torque[5]);
    printf("target torque : %f %f %f %f %f %f\n", tData->target_motor_torque[0], tData->target_motor_torque[1], tData->target_motor_torque[2], tData->target_motor_torque[3], tData->target_motor_torque[4], tData->target_motor_torque[5]);
    */
}

/*
 * Driver side implementation (i.e. interfacing with the Doosan)
 */
XBot::Hal::DoosanDriverContainer::DoosanDriverContainer(std::vector<DeviceInfo> devinfo,
                                                        const Device::CommonParams &p) : DeviceContainer(devinfo, p)
{
    if (_instance == nullptr)
    {
        _instance = this;
    }

    // register callback and start connection if needed
    _drfl.set_on_monitoring_state(XBot::Hal::DoosanDriverContainer::StaticMonitoringStateCB);
    _drfl.set_on_monitoring_access_control(XBot::Hal::DoosanDriverContainer::StaticMonitroingAccessControlCB);
    _drfl.set_on_tp_initializing_completed(XBot::Hal::DoosanDriverContainer::StaticTpInitializingCompleted);
    _drfl.set_on_log_alarm(XBot::Hal::DoosanDriverContainer::StaticLogAlarmCB);
    _drfl.set_on_rt_monitoring_data(XBot::Hal::DoosanDriverContainer::StaticRTMonitoringData);

    Context().journal().jhigh().jinfo("DoosanDriverContainer ready to setup!\n");

    // DRFL open connection
    bool connect_ok = _drfl.open_connection(DOOSAN_IP);

    Context().journal().jhigh().jinfo("Connection with Doosan open!\n");

    // obtain the control from the external PC
    bool external_control_ok = _drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);

    _drfl.setup_monitoring_version(1);
    Context().journal().jhigh().jinfo("Library version: {}\n", _drfl.get_library_version());

    // servo on
    _drfl.set_robot_control(CONTROL_SERVO_ON);
    _drfl.set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX_10, TRUE);

    while ((_drfl.get_robot_state() != STATE_STANDBY) || !g_bHasControlAuthority)
    {

        Context().journal().jhigh().jinfo("Connect is ok? {} - state: {} - External control? {}\n",
                                          connect_ok,
                                          _drfl.get_robot_state(),
                                          external_control_ok);
        this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    Context().journal().jhigh().jinfo("Found {} devices from config file\n", devinfo.size());


    // init container variables
    _container_rx.init();
    _container_tx.init();

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
            
            // init driver param         
            //_q_dot_d[i-1] = -10000; // -10000 treated as none
            //_q_ddot_d[i-1] = -10000; // -10000 treated as none

            _kp[i-1] = 1000.0;
            _kd[i-1] = 80.0;

            _container_rx.stiffness = _kp[i-1];
            _container_rx.damping = _kd[i-1];
            get_device(i)->set_rx(_container_rx);

        }
    }


    // robot mode MANUAL on REAL doosan
    _drfl.set_robot_mode(ROBOT_MODE_MANUAL);
    _drfl.set_robot_system(ROBOT_SYSTEM_REAL);

    // connect to RT control
    _drfl.connect_rt_control();

    // info for rt control
    Context().journal().jhigh().jinfo("RT Doosan Input {}\n",
                                      _drfl.get_rt_control_input_data_list(_drfl.get_rt_control_input_version_list()));
    Context().journal().jhigh().jinfo("RT Doosan Output {}\n",
                                      _drfl.get_rt_control_output_data_list(_drfl.get_rt_control_output_version_list()));

    // rt control configuration
    _drfl.set_rt_control_output("v1.0", 0.001, 10);
    //_drfl.set_rt_control_input("v1.0", 0.001, 10); //TX from xbot2 to doosan

    // start RT control
    _drfl.start_rt_control();

    _drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_ENTER);
    //sleep(1);
    _drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);

}

bool XBot::Hal::DoosanDriverContainer::sense_all()
{
    bool sense_ok = true;

    _doosan_data = _drfl.read_data_rt(); // TBD check what is needed in terms of memcpy
    memcpy(_doosan_q, _doosan_data->actual_joint_position, sizeof(float) * JOINTS);
    memcpy(_doosan_q_dot, _doosan_data->actual_joint_velocity, sizeof(float) * JOINTS);
    memcpy(_doosan_torque, _doosan_data->actual_joint_torque, sizeof(float) * JOINTS);
    memcpy(_doosan_gravity_torque, _doosan_data->gravity_torque, sizeof(float) * JOINTS);

    memcpy(_doosan_qref, _doosan_data->target_joint_position, sizeof(float) * JOINTS);

    //Context().journal().jhigh().jok("_doosan_qref {}", _doosan_qref[0]);
    //Context().journal().jhigh().jinfo("timestamp {}", _doosan_data->time_stamp);

    for (size_t i = 1; i <= JOINTS; i++) {
        _doosan_q[i - 1] = DEG_TO_RAD(_doosan_q[i - 1]);
        _doosan_q_dot[i - 1] = DEG_TO_RAD(_doosan_q_dot[i - 1]);

        _container_rx.position = _doosan_q[i - 1];
        _container_rx.velocity = _doosan_q_dot[i - 1];
        _container_rx.torque = _doosan_torque[i - 1];

        _container_rx.stiffness = _kp[i-1];
        _container_rx.damping = _kd[i-1];

        _doosan_qref[i - 1] = DEG_TO_RAD(_doosan_qref[i - 1]);  // TBD do an utils function
        _container_rx.position_ref = _doosan_qref[i - 1];

        get_device(i)->set_rx(_container_rx);
    }

    return DeviceContainer::sense_all() && sense_ok;
}

bool XBot::Hal::DoosanDriverContainer::move_all()
{
    bool move_ok = true;

    for (size_t i = 1; i <= JOINTS; i++) {
        //_doosan_qref[i-1] = RAD_TO_DEG(_doosan_qref[i-1]); // TBD do an utils function

        get_device(i)->get_tx(_container_tx);
        //_container_tx.position_ref = RAD_TO_DEG(_container_tx.position_ref);

        _doosan_qref[i-1] = _container_tx.position_ref;
        //Context().journal().jhigh().jok("_doosan_qref {}", _doosan_qref[i-1]);
        //Context().journal().jhigh().jok("_doosan_gravity_torque {}", _doosan_gravity_torque[i-1]);
        //Context().journal().jhigh().jok("_doosan_q {}", _doosan_q[i-1]);
        //Context().journal().jhigh().jok("kp {}", _kp[i-1] * (_doosan_qref[i-1] - _doosan_q[i - 1] ) );

        _doosan_torque_ref[i-1] = _doosan_gravity_torque[i-1] + 
                                  _kp[i-1] * (_doosan_qref[i-1] - _doosan_q[i - 1] ) + 
                                  _kd[i-1] * (_q_dot_d[i-1] - _doosan_q_dot[i-1]);

        //_container_tx.torque_ref = _doosan_torque_ref[i-1];


        
        //Context().journal().jhigh().jok("{} : _container_tx stiffness {}", i, _container_tx.stiffness_ref);


    }

    move_ok = _drfl.torque_rt(_doosan_torque_ref, 0);
    
    ////move_ok = _drfl.servoj_rt(_doosan_qref, _q_dot_d, _q_ddot_d, 0.001*20); // SHAKYYYYY
    

    
    //Context().journal().jhigh().jinfo("moveall !");
    //Context().journal().jhigh().jok("_doosan_qref {}", _doosan_qref[4]);


    return DeviceContainer::move_all() && move_ok;
}

XBot::Hal::DoosanDriver::DoosanDriver( DeviceInfo di,
                                       const CommonParams &params) : DeviceDriverTpl(di, params),
                                                                    _safety(di, get_period_sec(), JointSafety::force_safety_flag::safety_not_required)
{

    // get param manager
    auto &pm = Context().paramManager();

    // register the device for Position, Velocity and Effort control modes
    register_resource(JointBase::Resource::Position,
                      JointBase::Resource::Mask::Position);
    register_resource(JointBase::Resource::Velocity,
                      JointBase::Resource::Mask::Velocity);
    register_resource(JointBase::Resource::Effort,
                      JointBase::Resource::Mask::Effort);

    register_resource(JointBase::Resource::Stiffness,
                      JointBase::Resource::Mask::Stiffness);
    register_resource(JointBase::Resource::Damping,
                      JointBase::Resource::Mask::Damping);

    register_resource(JointBase::Resource::Impedance,
                      JointBase::Resource::Mask::Impedance);


    Context().journal().jhigh().jok("connected to {} with name '{}' and period '{} s' \n", di.type, di.name, get_period_sec());
}

bool XBot::Hal::DoosanDriver::sense_impl()
{
    // receive from DRFL TBD optimize from the container
    bool recv_ok = true;

    // init tx values only at the first valid recv
    if (recv_ok && !_tx_initialized)
    {


        Context().journal().jhigh().jok("INIT TX REF");
        _tx.position_ref = _rx.position;
        _tx.velocity_ref = 0.0;
        _tx.torque_ref = 0.0;
        _tx.stiffness_ref = _rx.stiffness;
        _tx.damping_ref = _rx.damping;

        // HACK do a copy to xbot joint to use safety
        _rx_xbot.link_pos = _rx.position;
        _rx_xbot.motor_pos = _rx.position;
        _rx_xbot.link_vel = _rx.velocity;
        _rx_xbot.motor_vel = _rx.velocity;
        _rx_xbot.torque = _rx.torque;
        _rx_xbot.gain_kp = _rx.stiffness;
        _rx_xbot.gain_kd = _rx.damping;

        _rx_xbot.pos_ref = _tx.position_ref;
        _rx_xbot.vel_ref = _tx.velocity_ref;
        _rx_xbot.tor_ref = _tx.torque_ref;
        _rx_xbot.gain_kp = _tx.stiffness_ref;
        _rx_xbot.gain_kd = _tx.damping_ref;

        _safety.initialize(_rx_xbot);
        _tx_initialized = true;
    }

    // get references on rx
    _rx.position_ref = _tx.position_ref;
    _rx.velocity_ref = _tx.velocity_ref;
    _rx.torque_ref = _tx.torque_ref;
    _rx.stiffness = _tx.stiffness_ref;
    _rx.damping = _tx.damping_ref;

    return recv_ok;
}

bool XBot::Hal::DoosanDriver::move_impl()
{

    /*
    // check safety copying to xbot tx data structure
    _tx_xbot.pos_ref = _tx.position_ref; // TBD check it
    _tx_xbot.vel_ref = 0.0; //_tx.velocity_ref; // TBD check it
    _tx_xbot.tor_ref = _tx.torque_ref; //_tx.torque_ref; // TBD check it

    _tx_xbot.mask = 7; // TBD check the mask and use the callback on_rx_recv

    if (!_safety.enforce(_tx_xbot, _tx_xbot_safe))
    {
        // if enforce is false means that we need to use the safe tx
        _tx.position_ref = _tx_xbot_safe.pos_ref;
        _tx.velocity_ref = _tx_xbot_safe.vel_ref;
        _tx.torque_ref = _tx_xbot_safe.tor_ref;
    }
    /*



    /*
    // transfer the data from safe XBot packet (_tx_xbot_safe) to Doosan
    _doosan_qref[get_id() - 1] = RAD_TO_DEG(_tx_xbot_safe.pos_ref);

    bool send_ok = true;
    // send to doosan
    if (get_id() == 6)
    {
        send_ok = _drfl.servoj_rt(_doosan_qref, _q_dot_d, _q_ddot_d, 0.02);
        //Context().journal().jhigh().jok("send_ok {}", send_ok);
    }
    */

    //Context().journal().jhigh().jok("move impl");

    return true;
}

XBOT2_REGISTER_DEVICE(XBot::Hal::DoosanDriverContainer,
                      XBot::Hal::DeviceContainer<XBot::Hal::DoosanClient>,
                      doosan)

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
    return _rx.link_pos;
}

double XBot::Hal::DoosanClient::get_motor_pos() const
{
    return _rx.motor_pos;
}

double XBot::Hal::DoosanClient::get_link_vel() const
{
    return _rx.link_vel;
}

double XBot::Hal::DoosanClient::get_motor_vel() const
{
    return _rx.motor_vel;
}

double XBot::Hal::DoosanClient::get_tor() const
{
    return _rx.torque;
}

double XBot::Hal::DoosanClient::get_stiffness() const
{
    return _rx.gain_kp;
}

double XBot::Hal::DoosanClient::get_damping() const
{
    return _rx.gain_kd;
}

double XBot::Hal::DoosanClient::get_temp() const
{
    return 0.0;
}

double XBot::Hal::DoosanClient::get_pos_ref() const
{
    return _rx.pos_ref;
}

double XBot::Hal::DoosanClient::get_vel_ref() const
{
    return _rx.vel_ref;
}

double XBot::Hal::DoosanClient::get_tor_ref() const
{
    return _rx.tor_ref;
}

double XBot::Hal::DoosanClient::get_stiffness_ref() const
{
    return _rx.gain_kp; 
}

double XBot::Hal::DoosanClient::get_damping_ref() const
{
    return _rx.gain_kd; 
}

void XBot::Hal::DoosanClient::set_pos_ref(double q)
{
    _tx.pos_ref = q;
    _tx.mask |= 1;
}

void XBot::Hal::DoosanClient::set_vel_ref(double q)
{
    _tx.vel_ref = q;
    _tx.mask |= 2;
}

void XBot::Hal::DoosanClient::set_tor_ref(double q)
{
    _tx.tor_ref = q;
    _tx.mask |= 4;
}

void XBot::Hal::DoosanClient::set_stiffness_ref(double q)
{
    _tx.gain_kp = q;
    _tx.mask |= 8;
}

void XBot::Hal::DoosanClient::set_damping_ref(double q)
{
    _tx.gain_kd = q;
    _tx.mask |= 16;
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
    //std::cout << eTrasnsitControl << std::endl; // TBD lose time to have control
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
    sleep(1); // IMPORTANT TO WAIT HERE!

    _drfl.setup_monitoring_version(1);
    Context().journal().jhigh().jinfo("Library version: {}\n", _drfl.get_library_version());

    // servo on
    _drfl.set_robot_control(CONTROL_SERVO_ON);
    
    // grinder control
    //_drfl.set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX_1, TRUE);
    //_drfl.set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX_1, FALSE);

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
    _container_rx = joint_rx();
    _container_tx = joint_tx();

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

            // initialize the stiffness and damping of our controller
            // TBD put it in the config file of the device
            _kp[i-1] = 1200.0;
            _kd[i-1] = 75.0;

            _container_rx.gain_kp = _kp[i-1];
            _container_rx.gain_kd = _kd[i-1];
            get_device(i)->set_rx(_container_rx);

            _container_tx.gain_kp = _kp[i-1];
            _container_tx.gain_kd = _kd[i-1];
            get_device(i)->set_tx(_container_tx);

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
    //_drfl.set_rt_control_input("v1.0", 0.001, 10); // TX from xbot2 (embedded pc connected devices) to doosan controller

    // start RT control
    _drfl.start_rt_control();

    // set the safety level to mode the doosan (unsafe to approach)
    _drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_ENTER);
    _drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);

}

bool XBot::Hal::DoosanDriverContainer::sense_all()
{
    bool sense_ok = true;

    _doosan_data = _drfl.read_data_rt(); // TBD check what is needed in terms of memcpy

    // RX
    memcpy(_doosan_q, _doosan_data->actual_joint_position, sizeof(float) * JOINTS);
    memcpy(_doosan_q_dot, _doosan_data->actual_joint_velocity, sizeof(float) * JOINTS);
    memcpy(_doosan_torque, _doosan_data->actual_joint_torque, sizeof(float) * JOINTS);
    memcpy(_doosan_gravity_torque, _doosan_data->gravity_torque, sizeof(float) * JOINTS);

    // TX
    memcpy(_doosan_qref, _doosan_data->target_joint_position, sizeof(float) * JOINTS);
    memcpy(_doosan_qdotref, _doosan_data->target_joint_velocity, sizeof(float) * JOINTS);
    memcpy(_doosan_torque_ref, _doosan_data->target_motor_torque, sizeof(float) * JOINTS);

    for (size_t i = 1; i <= JOINTS; i++) {
        // RX
        _doosan_q[i - 1] = DEG_TO_RAD(_doosan_q[i - 1]);
        _doosan_q_dot[i - 1] = DEG_TO_RAD(_doosan_q_dot[i - 1]);

        _container_rx.motor_pos = _doosan_q[i - 1];
        _container_rx.link_pos = _container_rx.motor_pos; // TBD fix it
        _container_rx.motor_vel = _doosan_q_dot[i - 1];
        _container_rx.link_vel = _container_rx.motor_vel; // TBD fix it
        _container_rx.torque = _doosan_torque[i - 1];

        _container_rx.gain_kp = _kp[i-1];
        _container_rx.gain_kd = _kd[i-1];

        // TX
        _doosan_qref[i - 1] = DEG_TO_RAD(_doosan_qref[i - 1]); 
        _container_rx.pos_ref = _doosan_qref[i - 1];

        _doosan_qdotref[i - 1] = DEG_TO_RAD(_doosan_qdotref[i - 1]); 
        _container_rx.vel_ref = _doosan_qdotref[i - 1];
        
        _container_rx.tor_ref = _doosan_torque_ref[i - 1];

        // TBD update impedance in reading

        // torque estimation of the second joint given our amazing passive spring identification
        if(i == 2){
            //Context().journal().jhigh().jok("_doosan_data->actual_motor_torque {}", _doosan_data->actual_motor_torque[1]);
            //Context().journal().jhigh().jok("_doosan_data->gravity_torque {}", _doosan_data->gravity_torque[1]);
            //Context().journal().jhigh().jok("*** _doosan_data->actual_joint_torque {}", _doosan_data->actual_joint_torque[1]);
        
            _container_rx.torque = _doosan_torque[i - 1] - (256 * _doosan_q[i-1]) - (89 * _doosan_q[i-1] * _doosan_q[i-1]);
        }

        // set the XBot2 RX with the above data
        get_device(i)->set_rx(_container_rx);
    }

    return DeviceContainer::sense_all() && sense_ok;
}

bool XBot::Hal::DoosanDriverContainer::move_all()
{
    bool move_ok = true;

    for (size_t i = 1; i <= JOINTS; i++) {

        get_device(i)->get_tx(_container_tx);
        
        // position ref from XBot2
        _doosan_qref[i-1] = _container_tx.pos_ref;
        // velocity ref from XBot2
        _doosan_qdotref[i-1] = _container_tx.vel_ref;

        // torque ref from XBot2
        // we transform it back taking into account the passive element in the ref
        if(i == 2){

            // Context().journal().jhigh().jok("_doosan_qref {}", _doosan_qref[i-1] );
            // Context().journal().jhigh().jok("_doosan_data->gravity_torque {}", _doosan_data->gravity_torque[i-1]);
            // Context().journal().jhigh().jok("_doosan_torque_ref CONDA {}", _container_tx.torque_ref);
            // Context().journal().jhigh().jok("{} : _container_tx position_ref ref {}", i, _container_tx.pos_ref);
            // Context().journal().jhigh().jok("{} : _container_tx velocity_ref {}", i, _container_tx.vel_ref);
            // Context().journal().jhigh().jok("{} : _container_tx torque_ref {}", i, _container_tx.tor_ref);
            // Context().journal().jhigh().jok("{} : _container_tx stiffness_ref {}", i, _container_tx.gain_kp);
            // Context().journal().jhigh().jok("{} : _container_tx damping_ref {}", i, _container_tx.gain_kd);

            _doosan_torque_ref[i - 1] = _container_tx.tor_ref + (256 * _doosan_q[i-1]) + (89 * _doosan_q[i-1] * _doosan_q[i-1]);

            //Context().journal().jhigh().jok("_doosan_torque_ref SPRING {}", _doosan_torque_ref[i-1]);
        
        } else {

            _doosan_torque_ref[i - 1] = _container_tx.tor_ref;
        }

        // Our centralized joint impedence controller plus feed-forward
        _doosan_torque_ref[i - 1] = _doosan_gravity_torque[i - 1] +
                                    _container_tx.gain_kp * (_doosan_qref[i - 1] - _doosan_q[i - 1]) +
                                    _container_tx.gain_kd * (_doosan_qdotref[i - 1] - _doosan_q_dot[i - 1]) +
                                    _doosan_torque_ref[i - 1];

        // should we update hte xbot2 tor ref with the data we send to the doosan, including spring ? (TBD)
        _container_tx.tor_ref = _doosan_torque_ref[i - 1];
        

        //Context().journal().jhigh().jok("_doosan_qref {}", _doosan_qref[i-1]);
        //Context().journal().jhigh().jok("_doosan_gravity_torque {}", _doosan_gravity_torque[i-1]);
        //Context().journal().jhigh().jok("_doosan_q {}", _doosan_q[i-1]);
        //Context().journal().jhigh().jok("kp {}", _kp[i-1] * (_doosan_qref[i-1] - _doosan_q[i - 1] ) );
        //Context().journal().jhigh().jok("{} : _container_rx stiffness {}", i, _container_rx.stiffness);
        //Context().journal().jhigh().jok("{} : _container_tx stiffness {}", i, _container_tx.stiffness_ref);
        //Context().journal().jhigh().jok("_doosan_torque_ref {}", _doosan_torque_ref[i-1]);
    }

    // send the torque to the doosan
    move_ok = _drfl.torque_rt(_doosan_torque_ref, 0);

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
        _tx.reset(_rx);
        _tx.vel_ref = 0.0;
        _tx.tor_ref = 0.0;

        _safety.initialize(_rx);
        _tx_initialized = true;
    }

    // get references on rx
    _rx.pos_ref = _tx.pos_ref;
    _rx.vel_ref = _tx.vel_ref;
    _rx.tor_ref = _tx.tor_ref;
    _rx.gain_kp = _tx.gain_kp;
    _rx.gain_kd = _tx.gain_kd;

    return recv_ok;
}

bool XBot::Hal::DoosanDriver::move_impl()
{
    //Context().journal().jhigh().jok("DoosanDriver position_ref ref {}", _tx.pos_ref);
    // Context().journal().jhigh().jok("DoosanDriver velocity_ref {}", _tx.velocity_ref);
    // Context().journal().jhigh().jok("DoosanDriver torque_ref {}", _tx.torque_ref);
    // Context().journal().jhigh().jok("DoosanDriver stiffness_ref {}", _tx.stiffness_ref);
    // Context().journal().jhigh().jok("DoosanDriver damping_ref {}", _tx.damping_ref);
    // Context().journal().jhigh().jok("DoosanDriver mask {}", _tx.mask);

    //TBD rest mask
    //_tx.mask = 0;
    
    return true;
}

void XBot::Hal::DoosanDriver::on_tx_recv(const joint_tx& msg)
{
   
    // this applies valid fields and updates mask
    _tx.apply(msg);

    // Context().journal().jhigh().jinfo("received msg..: mask = {:b}&{:b}, tx = \n{}", msg.mask, msg.resource_mask, msg);
    // Context().journal().jhigh().jinfo("received pos_ref from msg = {}", msg.pos_ref);
    // Context().journal().jhigh().jinfo("msg mask {}", msg.mask);
    // Context().journal().jhigh().jinfo("updated tx {}", _tx.pos_ref);
    // Context().journal().jhigh().jinfo("updated tx....: mask = {:b}, tx = \n{}", _tx.mask, _tx);
}

XBOT2_REGISTER_DEVICE(XBot::Hal::DoosanDriverContainer,
                      XBot::Hal::DeviceContainer<XBot::Hal::DoosanClient>,
                      doosan)

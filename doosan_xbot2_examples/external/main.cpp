// DRFTWin32.cpp : �ܼ� ���� ���α׷��� ���� �������� �����մϴ�.
//

#ifdef __XENO__
#include <rtdk.h>
#include <native/task.h>
#include <native/timer.h>
#else
//#include "stdafx.h"
//#include <Windows.h>
//#include <process.h>
//#include <conio.h>
#endif // __XENO__
//#include <stdio.h>
#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>

#include "../../doosan_xbot2_hal/include/DRFLEx.h"
using namespace DRAFramework;

#undef NDEBUG
#include <assert.h>

CDRFLEx Drfl;
bool g_bHasControlAuthority = FALSE;
bool g_TpInitailizingComplted = FALSE;
bool g_mStat = FALSE;
bool g_Stop = FALSE;
bool moving = FALSE;
string strDrl =
    "\r\n\
loop = 0\r\n\
while loop < 1003:\r\n\
 movej(posj(10,10.10,10,10.10), vel=60, acc=60)\r\n\
 movej(posj(00,00.00,00,00.00), vel=60, acc=60)\r\n\
 loop+=1\r\n";

bool bAlterFlag = FALSE;

int linux_kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int getch()
{
    int c;
    struct termios oldattr, newattr;

    tcgetattr(STDIN_FILENO, &oldattr); // ���� �͹̳� ���� ����
    newattr = oldattr;
    newattr.c_lflag &= ~(ICANON | ECHO);        // CANONICAL�� ECHO ��
    newattr.c_cc[VMIN] = 1;                     // �ּ� �Է� ���� ���� 1�� ����
    newattr.c_cc[VTIME] = 0;                    // �ּ� �б� ��� �ð��� 0���� ����
    tcsetattr(STDIN_FILENO, TCSANOW, &newattr); // �͹̳ο� ���� �Է�
    c = getchar();                              // Ű���� �Է� ����
    tcsetattr(STDIN_FILENO, TCSANOW, &oldattr); // ������ �������� ����
    return c;
}

void OnTpInitializingCompleted()
{
    // Tp �ʱ�ȭ ���� ����� ��û.
    g_TpInitailizingComplted = TRUE;
    Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
}

void OnHommingCompleted()
{
    // 50msec �̳� �۾��� ������ ��.
    cout << "homming completed" << endl;
}

void OnProgramStopped(const PROGRAM_STOP_CAUSE)
{
    assert(Drfl.PlayDrlStop(STOP_TYPE_SLOW));
    // 50msec �̳� �۾��� ������ ��.
    // assert(Drfl.SetRobotMode(ROBOT_MODE_MANUAL));
    cout << "program stopped" << endl;
}

void OnMonitoringDataCB(const LPMONITORING_DATA pData)
{
    // 50msec �̳� �۾��� ������ ��.

    return;
    cout << "# monitoring 0 data " << pData->_tCtrl._tTask._fActualPos[0][0]
         << pData->_tCtrl._tTask._fActualPos[0][1]
         << pData->_tCtrl._tTask._fActualPos[0][2]
         << pData->_tCtrl._tTask._fActualPos[0][3]
         << pData->_tCtrl._tTask._fActualPos[0][4]
         << pData->_tCtrl._tTask._fActualPos[0][5] << endl;
}

void OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData)
{
    return;
    cout << "# monitoring 1 data " << pData->_tCtrl._tWorld._fTargetPos[0]
         << pData->_tCtrl._tWorld._fTargetPos[1]
         << pData->_tCtrl._tWorld._fTargetPos[2]
         << pData->_tCtrl._tWorld._fTargetPos[3]
         << pData->_tCtrl._tWorld._fTargetPos[4]
         << pData->_tCtrl._tWorld._fTargetPos[5] << endl;
}

void OnMonitoringCtrlIOCB(const LPMONITORING_CTRLIO pData)
{
    return;
    cout << "# monitoring ctrl 0 data" << endl;
    for (int i = 0; i < 16; i++)
    {
        cout << (int)pData->_tInput._iActualDI[i] << endl;
    }
}

void OnMonitoringCtrlIOExCB(const LPMONITORING_CTRLIO_EX pData)
{
    return;
    cout << "# monitoring ctrl 1 data" << endl;
    for (int i = 0; i < 16; i++)
    {
        cout << (int)pData->_tInput._iActualDI[i] << endl;
    }
    for (int i = 0; i < 16; i++)
    {
        cout << (int)pData->_tOutput._iTargetDO[i] << endl;
    }
}

void OnMonitoringStateCB(const ROBOT_STATE eState)
{
    cout << "current state: " << (int)eState << endl;
    // 50msec �̳� �۾��� ������ ��.
    switch ((unsigned char)eState)
    {
#if 0 // TP �ʱ�ȭ�� ����ϴ� ���������� API ���������� ������� ����.(TP���� \
      // �ܵ� ����� ���, ���)
    case STATE_NOT_READY:
        if (g_bHasControlAuthority) Drfl.SetRobotControl(CONTROL_INIT_CONFIG);
        break;
    case STATE_INITIALIZING:
        // add initalizing logic
        if (g_bHasControlAuthority) Drfl.SetRobotControl(CONTROL_ENABLE_OPERATION);
        break;
#endif
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
            Drfl.SetSafeStopResetType(SAFE_STOP_RESET_TYPE_DEFAULT);
            Drfl.SetRobotControl(CONTROL_RESET_SAFET_STOP);
        }
        break;
    case STATE_SAFE_OFF:
        cout << "STATE_SAFE_OFF1" << endl;
        if (g_bHasControlAuthority)
        {
            cout << "STATE_SAFE_OFF2" << endl;
            //Drfl.SetRobotControl(CONTROL_SERVO_ON);
        }
        break;
    case STATE_SAFE_STOP2:
        if (g_bHasControlAuthority)
            Drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_STOP);
        break;
    case STATE_SAFE_OFF2:
        if (g_bHasControlAuthority)
        {
            Drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_OFF);
        }
        break;
    case STATE_RECOVERY:
        // Drfl.SetRobotControl(CONTROL_RESET_RECOVERY);
        break;
    default:
        break;
    }
    return;
}

void OnMonitroingAccessControlCB(
    const MONITORING_ACCESS_CONTROL eTrasnsitControl)
{
    // 50msec �̳� �۾��� ������ ��.

    switch (eTrasnsitControl)
    {
    case MONITORING_ACCESS_CONTROL_REQUEST:
        assert(Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_NO));
        // Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_YES);
        break;
    case MONITORING_ACCESS_CONTROL_GRANT:
        g_bHasControlAuthority = TRUE;
        // cout << "GRANT1" << endl;
        // cout << "MONITORINGCB : " << (int)Drfl.GetRobotState() << endl;
        OnMonitoringStateCB(Drfl.GetRobotState());
        // cout << "GRANT2" << endl;
        break;
    case MONITORING_ACCESS_CONTROL_DENY:
    case MONITORING_ACCESS_CONTROL_LOSS:
        g_bHasControlAuthority = FALSE;
        if (g_TpInitailizingComplted)
        {
            // assert(Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_REQUEST));
            Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
        }
        break;
    default:
        break;
    }
}

void OnLogAlarm(LPLOG_ALARM tLog)
{
    g_mStat = true;
    cout << "Alarm Info: "
         << "group(" << (unsigned int)tLog->_iGroup << "), index("
         << tLog->_iIndex << "), param(" << tLog->_szParam[0] << "), param("
         << tLog->_szParam[1] << "), param(" << tLog->_szParam[2] << ")" << endl;
}

void OnTpPopup(LPMESSAGE_POPUP tPopup)
{
    cout << "Popup Message: " << tPopup->_szText << endl;
    cout << "Message Level: " << tPopup->_iLevel << endl;
    cout << "Button Type: " << tPopup->_iBtnType << endl;
}

void OnTpLog(const char *strLog) { cout << "Log Message: " << strLog << endl; }

void OnTpProgress(LPMESSAGE_PROGRESS tProgress)
{
    cout << "Progress cnt : " << (int)tProgress->_iTotalCount << endl;
    cout << "Current cnt : " << (int)tProgress->_iCurrentCount << endl;
}

void OnTpGetuserInput(LPMESSAGE_INPUT tInput)
{
    cout << "User Input : " << tInput->_szText << endl;
    cout << "Data Type : " << (int)tInput->_iType << endl;
}

void OnRTMonitoringData(LPRT_OUTPUT_DATA_LIST tData)
{

    static int td = 0;
    //printf("td -> %d\n", td);
    if (td++ >= 0)
    {
        td = 0;
        /*
        printf("timestamp : %.3f\n", tData->time_stamp);
        printf("joint : %f %f %f %f %f %f\n", tData->actual_joint_position[0], tData->actual_joint_position[1], tData->actual_joint_position[2], tData->actual_joint_position[3], tData->actual_joint_position[4], tData->actual_joint_position[5]);
        printf("q = %7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f\n",
               tData->actual_joint_position[0], tData->actual_joint_position[1], tData->actual_joint_position[2],
               tData->actual_joint_position[3], tData->actual_joint_position[4], tData->actual_joint_position[5]);
        printf("q_dot = %7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f\n",
               tData->actual_joint_velocity[0], tData->actual_joint_velocity[1], tData->actual_joint_velocity[2],
               tData->actual_joint_velocity[3], tData->actual_joint_velocity[4], tData->actual_joint_velocity[5]);
        printf("trq_g = %7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f\n",
               tData->gravity_torque[0], tData->gravity_torque[1], tData->gravity_torque[2],
               tData->gravity_torque[3], tData->gravity_torque[4], tData->gravity_torque[5]);
        */
    }
}

uint32_t ThreadFunc(void *arg)
{
    printf("start ThreadFunc\n");

    while (true)
    {
        if (linux_kbhit())
        {
            char ch = getch();
            switch (ch)
            {
            case 's':
            {
                printf("Stop!\n");
                g_Stop = true;
                Drfl.MoveStop(STOP_TYPE_SLOW);
            }
            break;
            case 'p':
            {
                printf("Pause!\n");
                Drfl.MovePause();
            }
            break;
            case 'r':
            {
                printf("Resume!\n");
                Drfl.MoveResume();
            }
            break;
            }
        }

        //Sleep(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "exit ThreadFunc" << std::endl;

    return 0;
}

void OnDisConnected()
{
    while (!Drfl.open_connection("192.168.137.100"))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

struct PlanParam
{
    float time;

    float ps[6];
    float vs[6];
    float as[6];
    float pf[6];
    float vf[6];
    float af[6];

    float A0[6];
    float A1[6];
    float A2[6];
    float A3[6];
    float A4[6];
    float A5[6];
};

struct TraParam
{
    float time;

    float pos[6];
    float vel[6];
    float acc[6];
};

void TrajectoryPlan(PlanParam *plan)
{
    float ps[6], vs[6], as[6];
    float pf[6], vf[6], af[6];
    float tf;

    tf = plan->time;

    for (int i = 0; i < 6; i++)
    {
        ps[i] = plan->ps[i];
        vs[i] = plan->vs[i];
        as[i] = plan->as[i];
        pf[i] = plan->pf[i];
        vf[i] = plan->vf[i];
        af[i] = plan->af[i];
    }

    for (int i = 0; i < 6; i++)
    {
        plan->A0[i] = ps[i];
        plan->A1[i] = vs[i];
        plan->A2[i] = as[i] / 2;
        plan->A3[i] = (20 * pf[i] - 20 * ps[i] - (8 * vf[i] + 12 * vs[i]) * tf - (3 * as[i] - af[i]) * tf * tf) / (2 * tf * tf * tf);
        plan->A4[i] = (30 * ps[i] - 30 * pf[i] + (14 * vf[i] + 16 * vs[i]) * tf + (3 * as[i] - 2 * af[i]) * tf * tf) / (2 * tf * tf * tf * tf);
        plan->A5[i] = (12 * pf[i] - 12 * ps[i] - (6 * vf[i] + 6 * vs[i]) * tf - (as[i] - af[i]) * tf * tf) / (2 * tf * tf * tf * tf * tf);
    }
}

void TrajectoryGenerator(PlanParam *plan, TraParam *tra)
{
    double A0[6], A1[6], A2[6], A3[6], A4[6], A5[6];
    double t = tra->time;

    for (int i = 0; i < 6; i++)
    {
        A0[i] = plan->A0[i];
        A1[i] = plan->A1[i];
        A2[i] = plan->A2[i];
        A3[i] = plan->A3[i];
        A4[i] = plan->A4[i];
        A5[i] = plan->A5[i];
    }

    for (int i = 0; i < 6; i++)
    {
        tra->pos[i] = A0[i] + A1[i] * t + A2[i] * t * t + A3[i] * t * t * t + A4[i] * t * t * t * t + A5[i] * t * t * t * t * t;
        tra->vel[i] = A1[i] + 2 * A2[i] * t + 3 * A3[i] * t * t + 4 * A4[i] * t * t * t + 5 * A5[i] * t * t * t * t;
        tra->acc[i] = 2 * A2[i] + 6 * A3[i] * t + 12 * A4[i] * t * t + 20 * A5[i] * t * t * t;
    }
}

int main(int argc, char **argv)
{

    // �ݹ� ���(// �ݹ� �Լ� �������� 50msec �̳� �۾��� ������ ��)
    Drfl.set_on_homming_completed(OnHommingCompleted);
    Drfl.set_on_monitoring_data(OnMonitoringDataCB);
    Drfl.set_on_monitoring_data_ex(OnMonitoringDataExCB);
    Drfl.set_on_monitoring_ctrl_io(OnMonitoringCtrlIOCB);
    Drfl.set_on_monitoring_ctrl_io_ex(OnMonitoringCtrlIOExCB);
    Drfl.set_on_monitoring_state(OnMonitoringStateCB);
    Drfl.set_on_monitoring_access_control(OnMonitroingAccessControlCB);
    Drfl.set_on_tp_initializing_completed(OnTpInitializingCompleted);
    Drfl.set_on_log_alarm(OnLogAlarm);
    Drfl.set_on_tp_popup(OnTpPopup);
    Drfl.set_on_tp_log(OnTpLog);
    Drfl.set_on_tp_progress(OnTpProgress);
    Drfl.set_on_tp_get_user_input(OnTpGetuserInput);
    Drfl.set_on_rt_monitoring_data(OnRTMonitoringData);

    Drfl.set_on_program_stopped(OnProgramStopped);
    Drfl.set_on_disconnected(OnDisConnected);

    // ���� ����
    assert(Drfl.open_connection("192.168.137.100"));

    // ���� ���� ȹ��
    SYSTEM_VERSION tSysVerion = {
        '\0',
    };
    Drfl.get_system_version(&tSysVerion);
    // ����͸� ������ ���� ����
    Drfl.setup_monitoring_version(1);

    Drfl.set_robot_control(CONTROL_SERVO_ON);
    Drfl.set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX_10, TRUE);
    cout << "System version: " << tSysVerion._szController << endl;
    cout << "Library version: " << Drfl.get_library_version() << endl;

    while ((Drfl.get_robot_state() != STATE_STANDBY) || !g_bHasControlAuthority)
        // Sleep(1000);
        this_thread::sleep_for(std::chrono::milliseconds(1000));

    // ���� ��� ����

    assert(Drfl.set_robot_mode(ROBOT_MODE_MANUAL));
    assert(Drfl.set_robot_system(ROBOT_SYSTEM_REAL));

    bool collision = Drfl.change_collision_sensitivity(1.0f);
    if (collision)
    {
        cout << "interaction enabled" << endl;
    }
    else
    {
        cout << "error in change_collision_sensitivity" << endl;
    }

    // Drfl.ConfigCreateModbus("mr1", "192.168.137.70", 552,
    // MODBUS_REGISTER_TYPE_HOLDING_REGISTER, 3, 5);

    typedef enum
    {
        EXAMPLE_JOG,
        EXAMPLE_HOME,
        EXAMPLE_MOVEJ_ASYNC,
        EXAMPLE_MOVEL_SYNC,
        EXAMPLE_MOVEJ_SYNC,
        EXAMPLE_DRL_PROGRAM,
        EXAMPLE_GPIO,
        EXAMPLE_MODBUS,
        EXAMPLE_LAST,
        EXAMPLE_SERVO_OFF
    } EXAMPLE;

    EXAMPLE eExample = EXAMPLE_HOME;
    float q0_test[6] = {0, -30, 120, 0, 90, 0};
    float time_test = 5;

    float time = 0.0;
    int count = 0;
    float q[NUMBER_OF_JOINT] = {
        0.0,
    };
    float q_dot[NUMBER_OF_JOINT] = {
        0.0,
    };
    float q_d[NUMBER_OF_JOINT] = {
        0.0,
    };
    float q_dot_d[NUMBER_OF_JOINT] = {
        0.0,
    };
    float trq_g[NUMBER_OF_JOINT] = {
        0.0,
    };
    float trq_d[NUMBER_OF_JOINT] = {
        0.0,
    };
    float kp[NUMBER_OF_JOINT] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // have to tune
    float kd[NUMBER_OF_JOINT] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // have to tune

    float stx[6] = {0, 0, 0, 0, 0, 0};
    float stx2[6] = {1, 1, 1, 1, 1, 1};

    float fd[6] = {0, 0, 0, 0, 0, 0};
    unsigned char fctrl_dir[6] = {1, 1, 1, 1, 1, 1};

    float aux_6[6];

    string version = "v1.0";
    float period = 0.01; // 10 msec
    int losscount = 10;

    bool bLoop = TRUE;
    while (bLoop)
    {
        g_mStat = false;
        g_Stop = false;
#ifdef __XENO__
        unsigned long overrun = 0;
        const double tick = 1000000; // 1ms
        rt_task_set_periodic(nullptr, TM_NOW, tick);
        if (rt_task_wait_period(&overrun) == -ETIMEDOUT)
        {
            std::cout << __func__ << ": \x1B[37m\x1B[41mover-runs: " << overrun
                      << "\x1B[0m\x1B[0K" << std::endl;
        }
#else
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
#endif // __XENO__
#if 0
        static char ch = '0';
        if (ch == '7') ch = '0';
        else if (ch == '0') ch = '7';
#else
        cout << "\ninput key : ";
        // char ch = _getch();
        char ch;
        cin >> ch;
        cout << ch << endl;
#endif
        switch (ch)
        {
        case 'q':
            bLoop = FALSE;
            break;
        case '0':
        {
            switch ((int)eExample)
            {
            case EXAMPLE_JOG:
                //assert(Drfl.Jog(JOG_AXIS_JOINT_4, MOVE_REFERENCE_BASE, 20.0f));
                //sleep(1);
                cout << "jog stop" << endl;
                //Drfl.jog(JOG_AXIS_JOINT_4, MOVE_REFERENCE_BASE, 0.f);
                break;
            case EXAMPLE_HOME:
                //Drfl.move_home();
                //cout << "home stop" << endl;
                //Drfl.move_home(MOVE_HOME_MECHANIC, (unsigned char)0);

                sleep(1);

                Drfl.movej(q0_test, 0.0f, 0.0f, time_test);

                cout << "starting compliance control" << endl;
                Drfl.task_compliance_ctrl(stx, COORDINATE_SYSTEM_TOOL, 1.0f);
                collision = Drfl.change_collision_sensitivity(1.0f);
                if (collision)
                {
                    cout << "interaction enabled" << endl;
                }
                else
                {
                    cout << "error in change_collision_sensitivity" << endl;
                }

                //cout << Drfl.get_robot_mode() << endl;
                //sleep(10);
                //Drfl.set_stiffnessx(stx2);

                Drfl.set_desired_force(fd, fctrl_dir);
                //sleep(10);

                //Drfl.release_compliance_ctrl();

                break;
            case EXAMPLE_MOVEJ_ASYNC:
                assert(Drfl.MoveStop(STOP_TYPE_SLOW));
                cout << "movej async stop" << endl;
                break;
            case EXAMPLE_MOVEL_SYNC:
            case EXAMPLE_MOVEJ_SYNC:
                break;
            case EXAMPLE_DRL_PROGRAM:
                assert(Drfl.PlayDrlStop(STOP_TYPE_SLOW));
                // assert(Drfl.SetRobotMode(ROBOT_MODE_MANUAL));
                // assert(Drfl.SetRobotSystem(ROBOT_SYSTEM_REAL));
                cout << "drl player stop" << endl;
                break;
            case EXAMPLE_GPIO:
                cout << "reset gpio" << endl;
                for (int i = 0; i < NUM_DIGITAL; i++)
                {
                    assert(Drfl.SetCtrlBoxDigitalOutput((GPIO_CTRLBOX_DIGITAL_INDEX)i,
                                                        FALSE));
                }
                break;
            case EXAMPLE_MODBUS:
                cout << "reset modbus" << endl;
                assert(Drfl.SetModbusValue("mr1", 0));
                break;
            default:
                cout << "Command code " << (int)eExample << " not recognized!" << endl;
                break;
            }
        }
        break;
        case '1':
        {
            //Drfl.connect_rt_control("127.0.0.1", 12348);
            Drfl.connect_rt_control();
            sleep(1);
            cout << Drfl.get_rt_control_input_data_list(Drfl.get_rt_control_input_version_list()) << endl;
            cout << Drfl.get_rt_control_output_data_list(Drfl.get_rt_control_output_version_list()) << endl;
            sleep(1);
            Drfl.set_on_rt_monitoring_data(OnRTMonitoringData);
            sleep(1);
            Drfl.set_rt_control_input(version, period, losscount);
            Drfl.set_rt_control_output(version, period, losscount);
            //sleep(1);
            Drfl.start_rt_control();
            //sleep(1);
            cout << "gcomp" << endl;
            while (1)
            {

                time = (++count) * period;
                // get current state
                //memcpy(q, Drfl.read_data_rt()->actual_joint_position, sizeof(float) * 6);
                //memcpy(q_dot, Drfl.read_data_rt()->actual_joint_velocity, sizeof(float) * 6);
                memcpy(trq_g, Drfl.read_data_rt()->gravity_torque, sizeof(float) * 6);

                // gravity compensation + pd control
                for (int i = 0; i < 6; i++)
                {
                    trq_d[i] = trq_g[i] + kp[i] * (q_d[i] - q[i]) + kd[i] * (q_dot_d[i] - q_dot[i]);
                }
                //cout << trq_d[0] << " " << trq_d[1] << " " << trq_d[2] << " " << trq_d[3] << " " << trq_d[4] << " " << trq_d[5] << " " << endl;

                Drfl.torque_rt(trq_d, 0);

                //memcpy(aux_6, Drfl.get_tool_force()->_fForce, sizeof(float) * 6);

                //cout << aux_6[0] << " " << aux_6[1] << " " << aux_6[2] << " " << aux_6[3] << " " << aux_6[4] << " " << aux_6[5] << " " << endl;

                /*if (time > plan1.time)
                {
                    time = 0;
                    Drfl.stop(STOP_TYPE_SLOW);
                    break;
                }

                rt_task_wait_period(NULL); // RTOS function
                */

                this_thread::sleep_for(std::chrono::milliseconds(10));
                //memcpy(q, Drfl.read_data_rt()->actual_joint_position, sizeof(float) * 6);
                //cout << q[0] << endl;
            }
        }
        break;
        case '2':
        {
            string version = "v1.0";
            float period = 0.001;
            int losscount = 4;
            //              Drfl.set_rt_control_input(version, hz, losscount);
            Drfl.set_rt_control_output(version, period, losscount);
        }
        break;
        case '3':
        {
            Drfl.start_rt_control();
        }
        break;
        case '4':
        {
            Drfl.stop_rt_control();
        }
        break;
        case '5':
        {
        }

        break;
        case '6':
        {
        }
        break;
        case '7': // speedj
        {
        }
        break;
        case '8': // speedl
        {
        }
        break;
        case '9': // torque
        {
        }
        break;
        default:
            break;
        }
        // Sleep(100);
        this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    Drfl.CloseConnection();

#ifdef __XENO__
    rt_task_join(&sub_task);
#endif // __XENO__

    return 0;
}

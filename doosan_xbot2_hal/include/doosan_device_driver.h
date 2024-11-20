#pragma once

#include <memory>

#include <xbot2/hal/device.h>
#include <xbot2/hal/dev_joint_safety.h>
#include <xbot2/hal/dev_joint_packet.h>

#include <doosan_packet.h>

#include <DRFLEx.h>

#define JOINTS 6

namespace XBot
{
    namespace Hal
    {

        class DoosanDriver : public DeviceDriverTpl<doosan_rx,
                                                    doosan_tx>
        {

        public:
            XBOT2_DECLARE_SMART_PTR(DoosanDriver);

            DoosanDriver(DeviceInfo devinfo, const CommonParams &p);

            bool move_impl() override;
            bool sense_impl() override;

        private:
            bool _tx_initialized = false;

            // safety for v2.10
            XBot::Hal::JointSafety _safety;
            joint_tx _tx_xbot, _tx_xbot_safe;
            joint_rx _rx_xbot;

            float _q_dot_d[NUMBER_OF_JOINT] = {0.0, };
            float _q_ddot_d[NUMBER_OF_JOINT] = {0.0, };

        };

        class DoosanDriverContainer : public DeviceContainer<DoosanDriver>
        {

        public:
            DoosanDriverContainer(std::vector<DeviceInfo> devinfo,
                                  const Device::CommonParams &p);

            bool sense_all() override;
            bool move_all() override;

        private:

            void OnMonitoringStateCB(const ROBOT_STATE eState);
            void OnMonitroingAccessControlCB(const MONITORING_ACCESS_CONTROL eTrasnsitControl);
            void OnLogAlarm(LPLOG_ALARM tLog);
            void OnTpInitializingCompleted();
            void OnRTMonitoringData(LPRT_OUTPUT_DATA_LIST tData);

            // Static proxy functions
            static void StaticMonitoringStateCB(const ROBOT_STATE eState);
            static void StaticMonitroingAccessControlCB(const MONITORING_ACCESS_CONTROL eTrasnsitControl);
            static void StaticLogAlarmCB(LPLOG_ALARM tLog);
            static void StaticTpInitializingCompleted();
            static void StaticRTMonitoringData(LPRT_OUTPUT_DATA_LIST tData);

            static DoosanDriverContainer *_instance; // Singleton instance for static proxies

            DRAFramework::CDRFLEx _drfl;

            LPRT_OUTPUT_DATA_LIST _doosan_data;

            bool g_bHasControlAuthority = false;
            bool g_TpInitailizingComplted = false;

            float _doosan_q[JOINTS] = {
                0.0,
            };
            float _doosan_torque[JOINTS] = {
                0.0,
            };
            float _doosan_qref[JOINTS] = {
                0.0,
            };
            float _doosan_qref_prev[JOINTS] = {
                0.0,
            };

        };

    }
}

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

            void initialize(DRAFramework::CDRFLEx& drfl);

        private:
            bool _tx_initialized = false;

            // safety for v2.10
            XBot::Hal::JointSafety _safety;
            joint_tx _tx_xbot, _tx_xbot_safe;
            joint_rx _rx_xbot;

            static float _doosan_q[JOINTS];
            static float _doosan_qref[JOINTS], _doosan_qref_prev[JOINTS];

            float _q_dot_d[NUMBER_OF_JOINT] = {0.0, };
            float _q_ddot_d[NUMBER_OF_JOINT] = {0.0, };

            DRAFramework::CDRFLEx _drfl;

        };

        class DoosanDriverContainer : public DeviceContainer<DoosanDriver>
        {

        public:
            DoosanDriverContainer(std::vector<DeviceInfo> devinfo,
                                  const Device::CommonParams &p);
        private:

            //static void OnMonitoringStateCB(const ROBOT_STATE eState);
            //static void OnLogAlarm(LPLOG_ALARM tLog);

            DRAFramework::CDRFLEx _drfl;
            //static bool g_bHasControlAuthority;
        };

    }
}

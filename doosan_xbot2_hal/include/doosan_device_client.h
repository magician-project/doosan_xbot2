#pragma once

//#include <doosan_packet.h>
#include <doosan_device.h>

namespace XBot
{
    namespace Hal
    {

        class DoosanClient : public DeviceClientTpl<joint_rx, joint_tx>,
                             public virtual DoosanBase
        {

        public:
            using DeviceClientTpl::DeviceClientTpl;

            // xbot joint base

            double get_link_pos() const override;
            double get_motor_pos() const override;
            double get_link_vel() const override;
            double get_motor_vel() const override;
            double get_tor() const override;
            double get_stiffness() const override;
            double get_damping() const override;
            double get_temp() const override;
            double get_pos_ref() const override;
            double get_vel_ref() const override;
            double get_tor_ref() const override;
            double get_stiffness_ref() const override;
            double get_damping_ref() const override;
            void set_pos_ref(double q) override;
            void set_vel_ref(double q) override;
            void set_tor_ref(double q) override;
            void set_stiffness_ref(double q) override;
            void set_damping_ref(double q) override;

            // custom
        };
    }
}

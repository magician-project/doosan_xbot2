#pragma once

#include <xbot2/hal/dev_joint_packet.h>

namespace XBot
{
    namespace Hal
    {
        struct doosan_rx : joint_rx
        {
            float doosan_gravity_torque;

            doosan_rx() : joint_rx()
            {
                doosan_gravity_torque = 0.0;
            };

            const joint_rx& get_joint_rx() const
            {
                return *this;
            };

            
        };

        struct doosan_tx : joint_tx
        {
            float target_joint_acceleration;

            doosan_tx() : joint_tx()
            {
                target_joint_acceleration = 0.0;
            };

            const joint_tx& get_joint_tx() const
            {
                return *this;
            };

            void reset(const doosan_rx& rx)
            {
                joint_tx::reset(rx.get_joint_rx());
                target_joint_acceleration = 0.0;
            };

            void apply(const doosan_tx& tx, 
                       uint8_t mask = std::numeric_limits<uint8_t>::max())
            {
                joint_tx::apply(tx.get_joint_tx(), mask);
                target_joint_acceleration = tx.target_joint_acceleration;
            };

        };
    }
}

#pragma once

namespace XBot
{
    namespace Hal
    {
        struct doosan_rx
        {
            double position;
            double velocity;
            double torque;
            double stiffness;
            double damping;

            double position_ref;
            double velocity_ref;
            double torque_ref;

            void init()
            {
                position = 0;
                velocity = 0;
                torque = 0;
                stiffness = 1;
                damping = 1;

                position_ref = 0;
                velocity_ref = 0;
                torque_ref = 0;
            }
        };

        struct doosan_tx
        {
            double position_ref;
            double velocity_ref;
            double torque_ref;
            double stiffness_ref;
            double damping_ref;

            void init()
            {
                position_ref = 0;
                velocity_ref = 0;
                torque_ref = 0;
                stiffness_ref = 0;
                damping_ref = 0;
            }
        };
    }
}

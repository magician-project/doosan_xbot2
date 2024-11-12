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

            double position_ref;
            double velocity_ref;
            double torque_ref;

            void init()
            {
                position = 0;
                velocity = 0;
                torque = 0;

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

            void init()
            {
                position_ref = 0;
                velocity_ref = 0;
                torque_ref = 0;
            }
        };
    }
}

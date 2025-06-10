#pragma once

namespace XBot
{
    namespace Hal
    {
        struct sander_rx
        {
            bool sander_status;

            sander_rx()
            {
                sander_status = false;
            };
            
        };

        struct sander_tx
        {
            bool sander_control;

            sander_tx()
            {
                sander_control = false;
            };

            void reset(const sander_rx& rx)
            {
                sander_control = rx.sander_status;
            };


        };
    }
}
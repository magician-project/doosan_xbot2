#pragma once

#include <xbot2/hal/device.h>

namespace XBot
{
    namespace Hal
    {

        class SanderBase : public virtual DeviceBase
        {

        public:
            virtual bool get_status_sander() = 0;
            virtual bool activate_sander(bool on_off) = 0;
        };

    }
}
#pragma once

#include <sander_packet.h>
#include <sander_device.h>

namespace XBot
{
    namespace Hal
    {

        class SanderClient : public DeviceClientTpl<sander_rx, sander_tx>,
                             public virtual SanderBase
        {

        public:
            using DeviceClientTpl::DeviceClientTpl;

            bool get_status_sander() override;
            bool activate_sander(bool on_off) override;
        };
    }
}

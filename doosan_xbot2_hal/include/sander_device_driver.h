#pragma once

#include <memory>

#include <matlogger2/matlogger2.h>

#include <xbot2/hal/device.h>

#include <sander_packet.h>

#include <DRFLEx.h>


namespace XBot
{
    namespace Hal
    {

        class SanderDriver : public DeviceDriverTpl<sander_rx,
                                                    sander_tx>
        {

        public:
            XBOT2_DECLARE_SMART_PTR(SanderDriver);

            SanderDriver(DeviceInfo devinfo, const CommonParams &p);

            bool move_impl() override;
            bool sense_impl() override;

        private:

            XBot::MatLogger2::Ptr _logger;

            int cont = 0;

        };

        class SanderDriverContainer : public DeviceContainer<SanderDriver>
        {

        public:
            SanderDriverContainer(std::vector<DeviceInfo> devinfo,
                                  const Device::CommonParams &p);

            bool sense_all() override;
            bool move_all() override;


        };

    }
}

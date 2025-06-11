#include <xbot2/journal/journal.h>

#include <sander_device_client.h>
#include <sander_device_driver.h>

#include <global_drfl.h>

/*
 * Client side implementation (i.e. used by plugin implementers)
 */

// SanderBase

bool XBot::Hal::SanderClient::get_status_sander() 
{

    return _rx.sander_status;
}


bool XBot::Hal::SanderClient::activate_sander(bool on_off) 
{
    Context().journal().jhigh().jinfo("Activating sander {}\n", on_off);

    _tx.sander_control = on_off;
    return true;
}


/*
 * Driver side implementation (i.e. interfacing with the Doosan)
 */
XBot::Hal::SanderDriverContainer::SanderDriverContainer(std::vector<DeviceInfo> devinfo,
                                                        const Device::CommonParams &p) : DeviceContainer(devinfo, p)
{

    if (devinfo.empty())
    {
        // TBD autodetection
        DeviceInfo di;
        di.id = 0;
        di.name = "sander_" + std::to_string(di.id);
        di.type = "doosan_sander";

        Context().journal().jhigh().jok("detected {} '{}' with id {} \n",
                                        di.type, di.name, di.id);

        auto d = std::make_shared<SanderDriver>(di, p);

        addDevice(d);
    }

}

bool XBot::Hal::SanderDriverContainer::sense_all()
{

    return DeviceContainer::sense_all();
}

bool XBot::Hal::SanderDriverContainer::move_all()
{

    return DeviceContainer::move_all();
}

XBot::Hal::SanderDriver::SanderDriver( DeviceInfo di,
                                       const CommonParams &params) : DeviceDriverTpl(di, params)
{

    // get param manager
    auto &pm = Context().paramManager();

    Context().journal().jhigh().jok("connected to {} with name '{}' and period '{} s' \n", 
                                    di.type, di.name, get_period_sec());
    
    // init logger
    MatLogger2::Options logger_opt;
    logger_opt.default_buffer_size = 5e5;
    logger_opt.default_buffer_size_max_bytes = 5e8;
    
    _logger = MatLogger2::MakeLogger("/tmp/sander_device", logger_opt);
    _logger->set_buffer_mode(VariableBuffer::Mode::circular_buffer);

    _logger->create("sander_status", 1);
    _logger->create("sander_control", 1);

    std::cout << "Sander -----------------" << &_drfl << std::endl;


    _drfl.set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX_1, FALSE);

   
}

bool XBot::Hal::SanderDriver::sense_impl()
{
    bool recv_ok = true;

    // TBD READ the GPIO
   
    _logger->add("sander_status", _rx.sander_status);
    return recv_ok;
}

bool XBot::Hal::SanderDriver::move_impl()
{

    // check sander control
    if(_rx.sander_status != _tx.sander_control) {
        Context().journal().jhigh().jok("Controlling gripper {}", _tx.sander_control);
        _drfl.set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX_1, _tx.sander_control); // NOTE by default is on GPIO 1
        _rx.sander_status = _tx.sander_control;
    }

    _logger->add("sander_control", _tx.sander_control);
    return true;
}


XBOT2_REGISTER_DEVICE(XBot::Hal::SanderDriverContainer,
                      XBot::Hal::DeviceContainer<XBot::Hal::SanderClient>,
                      sander)

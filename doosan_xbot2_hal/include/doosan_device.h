#pragma once

#include <xbot2/hal/device.h>
#include <xbot2/hal/dev_joint.h>

namespace XBot
{
    namespace Hal
    {

        class DoosanBase : public virtual JointBase
        {

        public:
            XBOT2_DECLARE_SMART_PTR(DoosanBase)

            // add doosan methods
        };

    }
}
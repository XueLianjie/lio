#pragma once

#include "ros/ros.h"
#include "nav_msgs/Imu.h"
#include "sensor_msgs/Image.h"

namespace learn_vio_frame_work
{

    class ExtrinsicCaliber
    {
    public:
        ExtrinsicCaliber();
        ~ExtrinsicCaliber();
        ExtrinsicCaliber(const ExtrinsicCaliber &extrinsic_caliber) = delete;
        void operator=(const ExtrinsicCaliber &extrinsic_caliber) = delete;

    private:
    };

} // namespace learn_vio_frame_work

#pragma once

#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "cev_msgs/msg/sensor_collect.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "ros_sensor.h"
#include "estimator.h"

using namespace ckf;

namespace cev_localization {
    namespace standard_ros_sensors {

        class IMUSensor : public RosSensor<sensor_msgs::msg::Imu> {
        private:
            double pos_mod(double angle);

        protected:
            bool initialized;
            bool relative;
            double initial_yaw;
            double last_reported_yaw;
            double last_sensor_raw_yaw;
            bool use_message_covariance = true;

        public:
            IMUSensor(std::string topic, V state, M covariance,
                std::vector<std::shared_ptr<Model>> dependents,
                std::vector<std::string> state_mask = {"d2_x", "d2_y", "yaw"},
                bool use_message_covariance = true, bool relative = true);

            StatePackage msg_update(sensor_msgs::msg::Imu::SharedPtr msg);
        };

        class RawSensor : public RosSensor<cev_msgs::msg::SensorCollect> {
        protected:
            bool use_message_covariance = true;

        public:
            RawSensor(std::string topic, V state, M covariance,
                std::vector<std::shared_ptr<Model>> dependents,
                std::vector<std::string> state_mask = {"d_x", "tau"},
                bool use_message_covariance = true);

            StatePackage msg_update(cev_msgs::msg::SensorCollect::SharedPtr msg);
        };

    }
}
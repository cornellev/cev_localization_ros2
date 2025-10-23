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
            /**
             * @brief Updates the linear acceleration of the state.
             * @param msg The IMU message to update the state with.
             * @param estimate The state to update.
             */
            void update_linear_acceleration(const sensor_msgs::msg::Imu::SharedPtr& msg,
                StatePackage& estimate);
            /**
             * @brief Updates the orientation of the state.
             * @param msg The IMU message to update the state with.
             * @param estimate The state to update.
             */
            void update_orientation(const sensor_msgs::msg::Imu::SharedPtr& msg,
                StatePackage& estimate);
            /**
             * @brief Helper to accumulate an angle measurement.
             * @param raw_angle The raw angle measurement.
             * @param last_raw_angle The last raw angle measurement.
             * @param accumulated_angle The accumulated angle measurement.
             * @return The new accumulated angle measurement.
             */
            double accumulate_angle(double raw_angle, double& last_raw_angle,
                double& accumulated_angle);

        protected:
            bool initialized;
            /**
             * @brief Whether to report the orientation relative to the initial orientation.
             */
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

            /**
             * @brief Updates the state with new IMU sensor data.
             * @param msg The IMU message to update the state with.
             * @return The updated state.
             */
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

            /**
             * @brief Updates the state with new raw sensor data.
             * @param msg The raw sensor message to update the state with.
             * @return The updated state.
             */
            StatePackage msg_update(cev_msgs::msg::SensorCollect::SharedPtr msg);
        };

    }
}

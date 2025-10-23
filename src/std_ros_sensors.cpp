#include "std_ros_sensors.h"
#include <cmath>

using namespace cev_localization::standard_ros_sensors;

/* IMU SENSOR */

IMUSensor::IMUSensor(std::string topic, V state, M covariance,
    std::vector<std::shared_ptr<Model>> dependents, std::vector<std::string> state_mask,
    bool use_message_covariance, bool relative)
    : RosSensor<sensor_msgs::msg::Imu>(topic, state, covariance, dependents),
      initialized(false),
      relative(relative),
      initial_yaw(0),
      last_reported_yaw(0),
      last_sensor_raw_yaw(0) {
    multiplier = Estimator::state_mask_to_matrix(state_mask);
    this->use_message_covariance = use_message_covariance;
}

void IMUSensor::update_linear_acceleration(const sensor_msgs::msg::Imu::SharedPtr& msg,
    StatePackage& estimate) {
    // Skip if linear acceleration data invalid
    if (msg->linear_acceleration_covariance[0] == -1) {
        return;
    }

    estimate.state[ckf::state::d2_x] = msg->linear_acceleration.x;
    estimate.state[ckf::state::d2_y] = msg->linear_acceleration.y;

    if (use_message_covariance) {
        estimate.covariance(ckf::state::d2_x, ckf::state::d2_x) = msg->linear_acceleration_covariance[0];
        estimate.covariance(ckf::state::d2_x, ckf::state::d2_y) = msg->linear_acceleration_covariance[1];
        estimate.covariance(ckf::state::d2_y, ckf::state::d2_x) = msg->linear_acceleration_covariance[3];
        estimate.covariance(ckf::state::d2_y, ckf::state::d2_y) = msg->linear_acceleration_covariance[4];
    }
}

double IMUSensor::accumulate_angle(double raw_angle, double& last_raw_angle,
    double& accumulated_angle) {
    double delta = std::fmod(raw_angle - last_raw_angle, 2 * M_PI);
    if (delta > M_PI) {
        delta -= 2 * M_PI;
    } else if (delta < -M_PI) {
        delta += 2 * M_PI;
    }
    accumulated_angle += delta;
    last_raw_angle = raw_angle;
    return accumulated_angle;
}

void IMUSensor::update_orientation(const sensor_msgs::msg::Imu::SharedPtr& msg,
    StatePackage& estimate) {
    // Skip if orientation data invalid
    if (msg->orientation_covariance[0] == -1) {
        return;
    }

    // IMU message quaternion to matrix, extract pitch, yaw, roll
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll;
    double pitch;
    double yaw;

    m.getRPY(roll, pitch, yaw);

    // Update angles
    if (relative) {
        if (!initialized) {
            // Initialize angles to track, can add pitch as well.
            last_sensor_raw_yaw = yaw;
            last_reported_yaw = 0.0;
            initialized = true;

            estimate.state[ckf::state::yaw] = 0.0;
            return;
        }
        estimate.state[ckf::state::yaw] = accumulate_angle(yaw, last_sensor_raw_yaw, last_reported_yaw);
    } else {
        // Just use raw values
        last_sensor_raw_yaw = yaw;
        last_reported_yaw = yaw;
        estimate.state[ckf::state::yaw] = yaw;
    }

    if (use_message_covariance) {
        estimate.covariance(ckf::state::yaw, ckf::state::yaw) = msg->orientation_covariance[8];
    }
}

StatePackage IMUSensor::msg_update(sensor_msgs::msg::Imu::SharedPtr msg) {
    StatePackage estimate = get_internals();
    estimate.update_time = static_cast<double>(msg->header.stamp.sec)
                           + static_cast<double>(msg->header.stamp.nanosec) / 1e9;

    update_linear_acceleration(msg, estimate);
    update_orientation(msg, estimate);

    return estimate;
}

/* RAW SENSOR */

RawSensor::RawSensor(std::string topic, V state, M covariance,
    std::vector<std::shared_ptr<Model>> dependents, std::vector<std::string> state_mask,
    bool use_message_covariance)
    : RosSensor<cev_msgs::msg::SensorCollect>(topic, state, covariance, dependents) {
    multiplier = Estimator::state_mask_to_matrix(state_mask);
    this->use_message_covariance = use_message_covariance;
}

StatePackage RawSensor::msg_update(cev_msgs::msg::SensorCollect::SharedPtr msg) {
    StatePackage estimate = get_internals();
    estimate.update_time = msg->timestamp;

    estimate.state[ckf::state::d_x] = msg->velocity;
    estimate.state[ckf::state::tau] = msg->steering_angle;

    // if (this->use_message_covariance) { // TODO: Implement covariance
    //     estimate.covariance(ckf::state::d_x, ckf::state::d_x) = msg->velocity_covariance;
    //     estimate.covariance(ckf::state::tau, ckf::state::tau) = msg->steering_angle_covariance;
    // }

    return estimate;
}

#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>

#include "model.h"
#include "ros_sensor.h"
#include "config_parser.h"
#include "model.h"
#include "sensor.h"
#include "standard_models.h"
#include "std_ros_sensors.h"

#include <tf2_ros/transform_broadcaster.h>

#include <iostream>

using std::placeholders::_1;

using namespace ckf;
using namespace cev_localization;

class LocalizationNode : public rclcpp::Node {
public:
    LocalizationNode(): Node("CEVLocalizationNode") {
        RCLCPP_INFO(this->get_logger(), "Initializing CEV Localization Node");

        this->declare_parameter<std::string>("config_file", "config/ekf_real.yml");
        std::string config_file_path = this->get_parameter("config_file").as_string();

        RCLCPP_INFO(this->get_logger(), "Parsing config file at %s", config_file_path.c_str());

        // Load the YAML configurations
        config = config_parser::ConfigParser::load(config_file_path);

        std::unordered_map<std::string, std::shared_ptr<ckf::Model>> update_models;

        for (std::pair<const std::string, config_parser::UpdateModel> model: config.update_models) {
            std::string name = model.first;
            config_parser::UpdateModel& mod = model.second;

            if (update_models.find(name) != update_models.end()) {
                RCLCPP_ERROR(this->get_logger(), "Model `%s` already exists", name.c_str());
                throw std::runtime_error("Model already exists");
            }

            if (mod.type == "ACKERMANN") {
                update_models[name] = std::make_shared<ckf::standard_models::AckermannModel>(
                    V::Zero(), M::Identity() * .1, M::Identity() * .1, .185, mod.state_mask);
            } else if (mod.type == "CARTESIAN") {
                update_models[name] = std::make_shared<ckf::standard_models::CartesianModel>(
                    V::Zero(), M::Identity() * .1, M::Identity() * .1, mod.state_mask);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Unknown model type: `%s`", mod.type.c_str());
                throw std::runtime_error("Unknown model type");
            }

            if (mod.publish.active) {
                filter_publishers[name] =
                    this->create_publisher<nav_msgs::msg::Odometry>(mod.publish.topic, 1);

                ckf::Model* model_ = update_models[name].get();

                // Create a timer to publish the model
                timers[name] = this->create_wall_timer(
                    std::chrono::milliseconds((int)(1000 / mod.publish.rate)),
                    [this, model_, name]() { this->publish_model(model_, name); });
            }
        }

        for (std::pair<const std::string, config_parser::Sensor> sensor: config.sensors) {
            std::string name = sensor.first;
            config_parser::Sensor& sen = sensor.second;

            if (sensors.find(name) != sensors.end()) {
                RCLCPP_ERROR(this->get_logger(), "Sensor `%s` already exists", name.c_str());
                throw std::runtime_error("Sensor already exists");
            }

            // Create array for models to bind to
            std::vector<std::shared_ptr<ckf::Model>> models;
            for (std::string& model_name: sen.estimator_models) {
                if (update_models.find(model_name) == update_models.end()) {
                    RCLCPP_ERROR(this->get_logger(), "Model `%s` does not exist",
                        model_name.c_str());
                    throw std::runtime_error("Model does not exist");
                } else {
                    models.push_back(update_models[model_name]);
                }
            }

            if (sen.type == "IMU") {
                auto sensor = std::make_shared<standard_ros_sensors::IMUSensor>(sen.topic,
                    V::Zero(), M::Identity() * .1, models, sen.state_mask,
                    sen.use_message_covariance);

                sensors[name] = sensor;

                sensor_subscribers[name] = this->create_subscription<sensor_msgs::msg::Imu>(
                    sen.topic, 10, [sensor](const sensor_msgs::msg::Imu::SharedPtr msg) {
                        sensor->msg_handler(msg);
                    });

            } else if (sen.type == "RAW") {
                auto sensor = std::make_shared<standard_ros_sensors::RawSensor>(sen.topic,
                    V::Zero(), M::Identity() * .1, models, sen.state_mask,
                    sen.use_message_covariance);

                sensors[name] = sensor;

                sensor_subscribers[name] = this->create_subscription<cev_msgs::msg::SensorCollect>(
                    sen.topic, 10, [sensor](const cev_msgs::msg::SensorCollect::SharedPtr msg) {
                        sensor->msg_handler(msg);
                    });
            } else {
                RCLCPP_ERROR(this->get_logger(), "Unknown sensor type: `%s`", sen.type.c_str());
                throw std::runtime_error("Unknown sensor type");
            }
        }

        RCLCPP_INFO(this->get_logger(), "Configuration file parsed! Finishing initialization.");

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    config_parser::Config config;

    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> sensor_subscribers;
    std::unordered_map<std::string, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr>
        filter_publishers;
    std::unordered_map<std::string, rclcpp::TimerBase::SharedPtr> timers;

    std::unordered_map<std::string, std::shared_ptr<ckf::Model>> update_models;
    std::unordered_map<std::string, std::shared_ptr<ckf::Sensor>> sensors;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer;

    void publish_model(ckf::Model* model_, std::string model) {
        V state = model_->get_state();
        M covariance = model_->get_covariance();

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = this->config.update_models[model].publish.parent_frame;
        odom_msg.child_frame_id = this->config.update_models[model].publish.child_frame;

        odom_msg.pose.pose.position.x = state[ckf::state::x];
        odom_msg.pose.pose.position.y = state[ckf::state::y];
        odom_msg.pose.pose.position.z = 0.0;

        odom_msg.pose.covariance[0] = covariance(ckf::state::x, ckf::state::x);
        odom_msg.pose.covariance[7] = covariance(ckf::state::y, ckf::state::y);
        odom_msg.pose.covariance[35] = covariance(ckf::state::yaw, ckf::state::yaw);

        tf2::Quaternion q = tf2::Quaternion();
        q.setRPY(0, 0, state[ckf::state::yaw]);
        q = q.normalized();
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x = state[ckf::state::d_x];
        odom_msg.twist.twist.linear.y = state[ckf::state::d_y];
        odom_msg.twist.twist.angular.z = state[ckf::state::d_yaw];

        odom_msg.twist.covariance[0] = covariance(ckf::state::d_x, ckf::state::d_x);
        odom_msg.twist.covariance[7] = covariance(ckf::state::d_y, ckf::state::d_y);
        odom_msg.twist.covariance[35] = covariance(ckf::state::d_yaw, ckf::state::d_yaw);

        this->filter_publishers[model]->publish(odom_msg);

        if (this->config.update_models[model].publish.publish_tf) {
            geometry_msgs::msg::TransformStamped transformStamped;

            transformStamped.header.stamp = this->now();
            transformStamped.header.frame_id =
                this->config.update_models[model].publish.parent_frame;
            transformStamped.child_frame_id = this->config.update_models[model].publish.child_frame;

            transformStamped.transform.translation.x = state[ckf::state::x];
            transformStamped.transform.translation.y = state[ckf::state::y];
            transformStamped.transform.translation.z = state[ckf::state::z];

            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();

            this->tf_broadcaster_->sendTransform(transformStamped);
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
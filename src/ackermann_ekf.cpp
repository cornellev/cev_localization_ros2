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
#include <utility>

using std::placeholders::_1;

using namespace ckf;
using namespace cev_localization;

class LocalizationNode : public rclcpp::Node {
public:
    LocalizationNode(): Node("CEVLocalizationNode") {
        RCLCPP_INFO(this->get_logger(), "Initializing CEV Localization Node");

        // Maybe dont hardcode this lol
        this->declare_parameter<std::string>("config_file", "config/ekf_real.yml");
        std::string config_file_path = this->get_parameter("config_file").as_string();

        RCLCPP_INFO(this->get_logger(), "Parsing config file at %s", config_file_path.c_str());

        // Load the YAML configurations
        config = config_parser::ConfigParser::loadConfig(config_file_path);

        // Init estimators
        init_update_models();
        init_sensors();

        RCLCPP_INFO(this->get_logger(), "Configuration file parsed! Finishing initialization.");

        timer = this->create_wall_timer(std::chrono::milliseconds((int)(config.time_step * 1000.0)),
            std::bind(&LocalizationNode::timer_callback, this));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(config.odometry_topic, 1);
    }

private:
    config_parser::Config config;

    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> sensor_subscribers;
    std::unordered_map<std::string, std::shared_ptr<ckf::Model>> update_models;
    std::unordered_map<std::string, std::shared_ptr<ckf::Sensor>> sensors;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

    ckf::Model* main_model;

    void timer_callback();
    void publish_odometry(const V& state, const M& covariance);
    // Helper init functions
    void init_update_models();
    void init_sensors();
    template<typename SensorT, typename MsgT>
    void add_sensor(const std::string& name, const config_parser::Sensor& config,
        const std::vector<std::shared_ptr<ckf::Model>>& models, double mult, double rate);
};

void LocalizationNode::timer_callback() {
    const V state = main_model->get_state();
    const M covariance = main_model->get_covariance();
    publish_odometry(state, covariance);
}

void LocalizationNode::init_update_models() {
    update_models.clear();

    for (const auto& entry: config.update_models) {
        const std::string& name = entry.first;
        const config_parser::UpdateModel& mod = entry.second;

        if (update_models.find(name) != update_models.end()) {
            RCLCPP_ERROR(this->get_logger(), "Model `%s` already exists", name.c_str());
            throw std::runtime_error("Model already exists");
        }

        if (mod.type == "ACKERMANN") {
            update_models[name] = std::make_shared<ckf::standard_models::AckermannModel>(
                V::Zero(), M::Identity() * .1, M::Identity() * .1, .185, mod.state_mask);
        } else if (mod.type == "CARTESIAN") {
            update_models[name] = std::make_shared<ckf::standard_models::CartesianModel>(V::Zero(),
                M::Identity() * .1, M::Identity() * .1, mod.state_mask);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown model type: `%s`", mod.type.c_str());
            throw std::runtime_error("Unknown model type");
        }
    }
}

void LocalizationNode::init_sensors() {
    for (const auto& entry: config.sensors) {
        const std::string& name = entry.first;
        const config_parser::Sensor& sensor_config = entry.second;

        if (sensors.find(name) != sensors.end()) {
            RCLCPP_ERROR(this->get_logger(), "Sensor `%s` already exists", name.c_str());
            throw std::runtime_error("Sensor already exists");
        }

        // Create array for models to bind to
        std::vector<std::shared_ptr<ckf::Model>> models;
        models.reserve(sensor_config.estimator_models.size());
        for (const auto& model_name: sensor_config.estimator_models) {
            auto it = update_models.find(model_name);
            if (it == update_models.end()) {
                RCLCPP_ERROR(this->get_logger(), "Model `%s` does not exist", model_name.c_str());
                throw std::runtime_error("Model does not exist");
            }
            models.push_back(it->second);
        }

        if (sensor_config.type == "IMU") {
            add_sensor<standard_ros_sensors::IMUSensor, sensor_msgs::msg::Imu>(name, sensor_config, models);
        } else if (sensor_config.type == "RAW") {
            add_sensor<standard_ros_sensors::RawSensor, cev_msgs::msg::SensorCollect>(name, sensor_config, models);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown sensor type: `%s`", sensor_config.type.c_str());
            throw std::runtime_error("Unknown sensor type");
        }
    }
    
    // set main model
    auto it = update_models.find(config.main_model);
    if (it == update_models.end()) {
        RCLCPP_ERROR(this->get_logger(), "Main model `%s` does not exist", config.main_model.c_str());
        throw std::runtime_error("Main model does not exist");
    }
    main_model = it->second.get();
}

template<typename SensorT, typename MsgT>
void LocalizationNode::add_sensor(const std::string& name, const config_parser::Sensor& config,
    const std::vector<std::shared_ptr<ckf::Model>>& models, double mult = .1, double rate = 10) {
    auto sensor = std::make_shared<SensorT>(config.topic, V::Zero(), M::Identity() * mult, models,
        config.state_mask, config.use_message_covariance);

    auto subscriber = this->create_subscription<MsgT>(config.topic, rate,
        [sensor](const typename MsgT::SharedPtr msg) { sensor->msg_handler(msg); });

    sensors[name] = sensor;
    sensor_subscribers[name] = std::move(subscriber);
}

void LocalizationNode::publish_odometry(const V& state, const M& covariance) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = config.odom_frame;
    odom_msg.child_frame_id = config.base_link_frame;

    odom_msg.pose.pose.position.x = state[ckf::state::x];
    odom_msg.pose.pose.position.y = state[ckf::state::y];
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.covariance[0] = covariance(ckf::state::x, ckf::state::x);
    odom_msg.pose.covariance[7] = covariance(ckf::state::y, ckf::state::y);
    odom_msg.pose.covariance[35] = covariance(ckf::state::yaw, ckf::state::yaw);

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, state[ckf::state::yaw]);
    orientation.normalize();
    odom_msg.pose.pose.orientation.x = orientation.x();
    odom_msg.pose.pose.orientation.y = orientation.y();
    odom_msg.pose.pose.orientation.z = orientation.z();
    odom_msg.pose.pose.orientation.w = orientation.w();

    odom_msg.twist.twist.linear.x = state[ckf::state::d_x];
    odom_msg.twist.twist.linear.y = state[ckf::state::d_y];
    odom_msg.twist.twist.angular.z = state[ckf::state::d_yaw];

    odom_msg.twist.covariance[0] = covariance(ckf::state::d_x, ckf::state::d_x);
    odom_msg.twist.covariance[7] = covariance(ckf::state::d_y, ckf::state::d_y);
    odom_msg.twist.covariance[35] = covariance(ckf::state::d_yaw, ckf::state::d_yaw);

    odom_pub->publish(odom_msg);

    if (config.publish_tf) {
        // Publish transform
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = config.odom_frame;
        transform.child_frame_id = config.base_link_frame;

        transform.transform.translation.x = state[ckf::state::x];
        transform.transform.translation.y = state[ckf::state::y];
        transform.transform.translation.z = state[ckf::state::z];

        transform.transform.rotation.x = orientation.x();
        transform.transform.rotation.y = orientation.y();
        transform.transform.rotation.z = orientation.z();
        transform.transform.rotation.w = orientation.w();

        tf_broadcaster_->sendTransform(transform);
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationNode>());
    rclcpp::shutdown();
    return 0;
}

#pragma once

#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <utility>
#include <optional>

#include "estimator.h"

namespace cev_localization {
    namespace config_parser {
        struct Sensor {
            std::string type;
            std::string topic;
            std::optional<std::string> frame_id;
            std::vector<std::string> state_mask;
            double covariance_multiplier;
            bool use_message_covariance;
            std::vector<std::string> estimator_models;

            static Sensor loadSensor(const YAML::Node& sensorNode);
        };

        struct Publish {
            bool active = false;
            std::string topic = "";
            int rate = 100;
            std::string child_frame = "";
            std::string parent_frame = "";

            bool publish_tf = false;

            static Publish loadPublish(const YAML::Node& publishNode);
        };

        struct UpdateModel {
            std::string type;
            std::vector<std::string> state_mask;
            std::vector<std::string> estimator_models;

            Publish publish;

            static UpdateModel loadUpdateModel(const YAML::Node& modelNode);
        };

        struct Config {
            // Sensors
            std::unordered_map<std::string, Sensor> sensors;

            // Update Models
            std::unordered_map<std::string, UpdateModel> update_models;

            static Config loadConfig(const YAML::Node& configNode);
        };

        class ConfigParser {
        public:
            static Config load(const std::string& filePath);
        };
    }
}

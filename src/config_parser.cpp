#include "config_parser.h"

using namespace cev_localization::config_parser;

Sensor Sensor::loadSensor(const YAML::Node& sensorNode) {
    if (!sensorNode["type"] || !sensorNode["topic"]) {
        throw std::runtime_error("Sensor type and topic must be defined");
    }

    Sensor sensor;
    sensor.type = sensorNode["type"].as<std::string>();
    sensor.topic = sensorNode["topic"].as<std::string>();
    sensor.frame_id = sensorNode["frame_id"]
                          ? std::make_optional(sensorNode["frame_id"].as<std::string>())
                          : std::nullopt;
    sensor.state_mask = sensorNode["state"] ? sensorNode["state"].as<std::vector<std::string>>()
                                            : std::vector<std::string>{};
    sensor.covariance_multiplier = sensorNode["covariance_multiplier"].as<double>(1.0);
    sensor.use_message_covariance = sensorNode["use_message_covariance"].as<bool>(true);
    sensor.estimator_models = sensorNode["estimator_models"]
                                  ? sensorNode["estimator_models"].as<std::vector<std::string>>()
                                  : std::vector<std::string>{};

    return sensor;
}

Publish Publish::loadPublish(const YAML::Node& publishNode) {
    Publish publish;

    if (!publishNode["active"]) {
        return publish;
    }

    publish.active = publishNode["active"].as<bool>();

    if (!publishNode["topic"] || !publishNode["child_frame"] || !publishNode["parent_frame"]) {
        throw std::runtime_error("Publish topic and frames must be defined");
    }

    publish.topic = publishNode["topic"].as<std::string>();
    publish.rate = publishNode["rate"].as<int>(100);
    publish.child_frame = publishNode["child_frame"].as<std::string>("");
    publish.parent_frame = publishNode["parent_frame"].as<std::string>("");

    publish.publish_tf = publishNode["publish_tf"].as<bool>(false);

    return publish;
}

UpdateModel UpdateModel::loadUpdateModel(const YAML::Node& modelNode) {
    if (!modelNode["type"] || !modelNode["state"]) {
        throw std::runtime_error("Model type must be defined");
    }

    UpdateModel model;
    model.type = modelNode["type"].as<std::string>();
    model.state_mask = modelNode["state"].as<std::vector<std::string>>();
    model.estimator_models = modelNode["estimator_models"]
                                 ? modelNode["estimator_models"].as<std::vector<std::string>>()
                                 : std::vector<std::string>{};

    if (!modelNode["publish"]) {
        return model;
    }

    model.publish = Publish::loadPublish(modelNode["publish"]);

    return model;
}

Config Config::loadConfig(const YAML::Node& configNode) {
    Config config;

    for (const auto& sensorEntry: configNode["sensors"]) {
        const std::string sensorName = sensorEntry.first.as<std::string>();
        config.sensors[sensorName] = Sensor::loadSensor(sensorEntry.second);
    }

    for (const auto& modelEntry: configNode["update_models"]) {
        const std::string modelName = modelEntry.first.as<std::string>();
        config.update_models[modelName] = UpdateModel::loadUpdateModel(modelEntry.second);
    }

    return config;
}

Config ConfigParser::load(const std::string& filePath) {
    YAML::Node configNode = YAML::LoadFile(filePath);

    return Config::loadConfig(configNode);
}
# CEV Localization Architecture

This package is has:

1. **`cev_localization_ros2/cev_kalman_filter`** – EKF library with the estimation logic, process and sensor models.
2. **`cev/cev_localization_ros2`** – the ROS 2 integration layer to implement the EKF library into nodes.

---

## EKF Core (`cev_kalman_filter`)

Handles the filter state management.

### State and Base Types
| File | Purpose |
|------|---------|
| `include/estimator.h` | Defines the shared state layout (`ckf::state::*`), the `StatePackage`/`SimpleStatePackage`, and the `Estimator` base class with state `V`, covariance `M`, and timing. |
| `src/estimator.cpp` | Implements getters/setters for above. |

Every model or sensor in EKF inherits from `Estimator`.

### Models
| File | Purpose |
|------|---------|
| `include/model.h` | Declares `ckf::Model` class which extends `Estimator`. Defines general model predictions (`Model::update`/`Model::predict`) and update (`Model::estimate_update`) flows, leaving `update_step` and `update_jacobian` to be implemented. |
| `src/model.cpp` | Implements fundamental EKF math (prediction, measurement update, covariances). |
| `include/standard_models.h` & `src/standard_models.cpp` | Implements the actual kinematic models, Ackermann and Cartesian. Implements `update_step` and `update_jacobian`. |

For dependent models, `Model::bind_to` links a “parent” model to the dependent so that a state update propagates down the chain.

### Sensors
| File | Purpose |
|------|---------|
| `include/sensor.h` & `src/sensor.cpp` | Sensor-side analogue to `Model`: sensors inherit from `ckf::Sensor`, extending `Estimator`. A sensor has its own state/covariance and measurements. |
| `include/ros_sensor.h` | Helper to make `ckf::Sensor` act like a ROS subscriber callback through `msg_handler` to connect to ROS layer. |

---

## ROS 2 Integration (`cev_localization_ros2`)

This layer has the ROS nodes, parameters, config parsing, and sensor/model instantiation.

### Configuration
| File | Purpose |
|------|---------|
| `src/config_parser.cpp` & `include/config_parser.h` | Load YAML files (e.g. `config/ekf_real.yml`) into a `config_parser::Config`, which lists models, sensors, and the main model. |
| `config/` | YAML configurations describing the models and sensors to instantiate. |

### ROS Sensor Adapters
| File | Purpose |
|------|---------|
| `include/std_ros_sensors.h` & `src/std_ros_sensors.cpp` | ROS sensor adapters that translate ROS messages (`sensor_msgs::msg::Imu`, `cev_msgs::msg::SensorCollect`, etc.) into `StatePackage` updates and feed them into the EKF models. |

### Node Entry Point
| File | Purpose |
|------|---------|
| `src/ackermann_ekf.cpp` | The `LocalizationNode` class. It: <br>• Parses configuration parameters. <br>• Instantiates EKF models (`init_update_models`). <br>• Creates sensor adapters/subscriptions (`init_sensors`). <br>• Connects sensors to models via the configuration’s `estimator_models` bindings. <br>• Publishes the new odometry (`publish_odometry`) and optional TF transforms. |

// Source file template to make custom plugin for gazebo ignition

// Include Ignition Gazebo headers
// You could erase already includeed files in .hpp file.

// C++
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>
#include <filesystem>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>

#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/LinearAcceleration.hh>
#include <ignition/gazebo/components/AngularAcceleration.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/RollingMean.hh>
#include <ignition/math/Helpers.hh>

#include <ignition/plugin/Register.hh>

// Include ROS 2 headers
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/wrench.hpp>

#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "ign_plugin/OrbitMechanics.hpp"

#define PI 3.1415926535897931


/// @brief Plugin Template for AstroROS in source file
// Notice namepace and class have to be same as header file.

namespace ign_plugin // Name your plugin namespace
{

class IgnitionPlugin // Name your plugin class
  : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate,
    public ignition::gazebo::ISystemPostUpdate // these inheritances are fixed.
{
public:

  /// \brief Constructor
  IgnitionPlugin() : orbit_(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0){

    // We assume ROS has already been initialized externally, but if not:
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);  // or handle arguments properly
    }

    this->lin_vel_x_.SetWindowSize(10); this->lin_acc_x_.SetWindowSize(10);
    this->lin_vel_y_.SetWindowSize(10); this->lin_acc_y_.SetWindowSize(10);
    this->lin_vel_z_.SetWindowSize(10); this->lin_acc_z_.SetWindowSize(10);

    this->ang_vel_x_.SetWindowSize(10); this->ang_acc_x_.SetWindowSize(10);
    this->ang_vel_y_.SetWindowSize(10); this->ang_acc_y_.SetWindowSize(10);
    this->ang_vel_z_.SetWindowSize(10); this->ang_acc_z_.SetWindowSize(10);

  };

  /// \brief Destructor
  ~IgnitionPlugin() override
  {
    if (this->ros_spin_thread_.joinable())
    {
      this->ros_spin_thread_.join();
    }
    rclcpp::shutdown();
  }

  /// \brief Configure: Called once at startup
  /*
  _entity : objects in ignition.(world,gui,robot etc.) You will add this plugin into xacro file.
            So you could think this entity is robot "model" you want.
  _sdf    : sdf/xacro/urdf file. It means an object of sdf you wrote; the file that this plugin
            added.
  _ecm    : Entity manager for ignition entities. every entities are monitored by it.
  &       : If there is a factor in original function but do not use that factor, then "&" could be
            used without warning. (TBR)
  */
  void Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &) override
  {
    // For demonstration, get link name from SDF parameter (or you could hard-code)
    if (_sdf->HasElement("model_name")) {
      this->model_name_ = _sdf->Get<std::string>("model_name");
    } else {
      std::cout<<"[ERROR] Missing <model_name> parameter in SDF. Check your SDF/Xacro/URDF file."<<std::endl;
      return;
    }

    try {
      std::filesystem::path yaml_path = this->yaml_file_ / (this->model_name_ + ".yaml");
      this->yaml_config_ = YAML::LoadFile(yaml_path.string());
      std::cout << this->yaml_config_["semi_major"].as<double>() << std::endl;
      std::cout << this->yaml_config_["eccentricity"].as<double>() << std::endl;
      std::cout << this->yaml_config_["inclination"].as<double>() << std::endl;
      std::cout << this->yaml_config_["ascending"].as<double>() << std::endl;
      std::cout << this->yaml_config_["perigee"].as<double>() << std::endl;
      std::cout << this->yaml_config_["mean_anomaly"].as<double>() << std::endl;

    } catch (const std::exception &e) {
      std::cout << "[ign_plugin] Failed to load YAML file" << std::endl;
      return;
    }
    this->orbit_ = OrbitMechanics(this->yaml_config_["semi_major"].as<double>(),
                                  this->yaml_config_["eccentricity"].as<double>(),
                                  this->yaml_config_["inclination"].as<double>(),
                                  this->yaml_config_["ascending"].as<double>(),
                                  this->yaml_config_["perigee"].as<double>(),
                                  this->yaml_config_["mean_anomaly"].as<double>(),
                                  0,0,0,0,0,0);

    this->node_ = std::make_shared<rclcpp::Node>(this->model_name_+"_plugin_node");
    this->orbit_node_ = std::make_shared<rclcpp::Node>(this->model_name_+"_orbit_plugin_node");
    RCLCPP_INFO(this->node_->get_logger(),"[ign_plugin] Find model name : [%s]", this->model_name_.c_str());

    this->model_info_pub_ = this->node_->create_publisher<std_msgs::msg::Float32MultiArray>(
                            this->model_name_+"_info", 10);
    this->model_info_timer_ = this->node_->create_wall_timer(
      std::chrono::milliseconds(100),  // 10 Hz timer
      std::bind(&IgnitionPlugin::ModelPoseCallback, this));

    this->model_orbit_pub_ = this->orbit_node_->create_publisher<std_msgs::msg::Float32MultiArray>(
      this->model_name_+"_orbit", 10);
    this->model_orbit_timer_ = this->node_->create_wall_timer(
      std::chrono::milliseconds(100),  // 10 Hz timer
      std::bind(&IgnitionPlugin::ModelOrbitCallback, this));

    // Make ros thread so that plugin node can operate indepenetly
    this->ros_spin_thread_ = std::thread([this]() {
      rclcpp::executors::MultiThreadedExecutor executor;
      executor.add_node(this->node_);
      executor.add_node(this->orbit_node_);
      RCLCPP_INFO(this->node_->get_logger(), "[ign_plugin] ROS 2 MultiThreadedExecutor started.");
      executor.spin();  // Use MultiThreadedExecutor for safer threading
    });

    // Attempt to locate the model entity in ECM
    auto model = ignition::gazebo::Model(_entity);
    if (!model.Valid(_ecm)) {
      RCLCPP_ERROR(this->node_->get_logger(), "[ign_plugin] This plugin must be attached to a valid model.");
      return;
    }
    this->model_entity_ = model.Entity();
    auto model_entity_name = _ecm.Component<ignition::gazebo::components::Name>(this->model_entity_);

    if (model_entity_name->Data() != this->model_name_){
      RCLCPP_ERROR(this->node_->get_logger(), "[ign_plugin] Unmatched entity name: [%s] [%s].",
                    model_entity_name->Data().c_str(), this->model_name_.c_str());
      return;
    }
    RCLCPP_INFO(this->node_->get_logger(),"[ign_plugin] Model name matched : [%s] [%s]",
                model_entity_name->Data().c_str(), this->model_name_.c_str());
  }

  /// \brief PreUpdate: Called every simulation iteration before physics update
  /*
  _info : update data in ignition simulation; puasing, dt, simulation time etc.
  _ecm : entity component manager
  */
  void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (_info.paused) {
    return;  // Do nothing if simulation is paused
    }
    if (!this->is_init_)
    {
      return;
    }

    // this->model_orbit_data_.clear();
    // Vector3d model_accel{
    //   static_cast<double>(this->model_data_[10]),
    //   static_cast<double>(this->model_data_[11]),
    //   static_cast<double>(this->model_data_[12])
    // };
    // auto cart_state = this->orbit_.getCartesian();
    // auto state = this->orbit_.integrateStateRK4(cart_state, _info.dt.count(), model_accel);

    // this->model_orbit_data_.push_back(state(0));
    // this->model_orbit_data_.push_back(state(1));
    // this->model_orbit_data_.push_back(state(2));
    // this->model_orbit_data_.push_back(state(3));
    // this->model_orbit_data_.push_back(state(4));
    // this->model_orbit_data_.push_back(state(5));
  }
  //===============================================
  /// \brief PostUpdate: Called after physics update, so we can read final states
  /*
  _info : update data in ignition simulation; puasing, dt, simulation time etc.
  _ecm : entity component manager
  */
  void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                  const ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (_info.paused) {
      return;
    }
    // Pose Component 얻기
    auto model_pose = _ecm.Component<ignition::gazebo::components::Pose>(this->model_entity_);
    if (!model_pose) {
      RCLCPP_WARN(this->node_->get_logger(), "[ign_plugin] Model pose component not found.");
      return;
    }
    std::lock_guard<std::mutex> lock(this->data_mutex_);

    auto &model_pose_data = model_pose->Data(); // gz::math::Pose3d

    if (!this->is_init_)
    {
      this->last_pose_ = model_pose_data;
      this->last_update_time_ = std::chrono::steady_clock::time_point(_info.simTime);
      this->last_vel_ = {0, 0, 0, 0, 0, 0};
      this->is_init_ = true;
      return;
    }

    std::chrono::duration<double> dt = _info.dt;
    if (dt.count() <= 0)
      return;
    // this->last_update_time_ = std::chrono::steady_clock::time_point(_info.simTime);

    // RCLCPP_INFO(this->node_->get_logger(), "[ign_plugin] dt : [%lf]", dt.count());
    this->model_data_.clear();

    double dx = model_pose_data.Pos().X() - this->last_pose_.Pos().X();
    double dy = model_pose_data.Pos().Y() - this->last_pose_.Pos().Y();
    double dz = model_pose_data.Pos().Z() - this->last_pose_.Pos().Z();

    double lin_vel_x = dx / dt.count();
    double lin_vel_y = dy / dt.count();
    double lin_vel_z = dz / dt.count();
// =========================================================================================

    // Compute angular velocity
    double curr_roll = model_pose_data.Rot().Roll();
    double curr_pitch = model_pose_data.Rot().Pitch();
    double curr_yaw = model_pose_data.Rot().Yaw();

    double last_roll = this->last_pose_.Rot().Roll();
    double last_pitch = this->last_pose_.Rot().Pitch();
    double last_yaw = this->last_pose_.Rot().Yaw();

    double roll_rate = (curr_roll - last_roll) / dt.count();
    double pitch_rate = (curr_pitch - last_pitch) / dt.count();
    double yaw_rate = (curr_yaw - last_yaw) / dt.count();

    this->lin_vel_x_.Push(lin_vel_x);
    this->lin_vel_y_.Push(lin_vel_y);
    this->lin_vel_z_.Push(lin_vel_z);

    this->ang_vel_x_.Push(roll_rate);
    this->ang_vel_y_.Push(pitch_rate);
    this->ang_vel_z_.Push(yaw_rate);

    // Compute acceleration
    std::vector<double> new_vel = {this->lin_vel_x_.Mean(), this->lin_vel_y_.Mean(),
                                   this->lin_vel_z_.Mean(), this->ang_vel_x_.Mean(),
                                   this->ang_vel_y_.Mean(), this->ang_vel_z_.Mean()};

    double dvx = new_vel[0] - this->last_vel_[0];
    double dvy = new_vel[1] - this->last_vel_[1];
    double dvz = new_vel[2] - this->last_vel_[2];

    double lin_acc_x = dvx / dt.count();
    double lin_acc_y = dvy / dt.count();
    double lin_acc_z = dvz / dt.count();

    double dwx = new_vel[3] - this->last_vel_[3];
    double dwy = new_vel[4] - this->last_vel_[4];
    double dwz = new_vel[5] - this->last_vel_[5];

    double ang_acc_x = dwx / dt.count();
    double ang_acc_y = dwy / dt.count();
    double ang_acc_z = dwz / dt.count();

    this->lin_acc_x_.Push(lin_acc_x);
    this->lin_acc_y_.Push(lin_acc_y);
    this->lin_acc_z_.Push(lin_acc_z);

    this->ang_acc_x_.Push(ang_acc_x);
    this->ang_acc_y_.Push(ang_acc_y);
    this->ang_acc_z_.Push(ang_acc_z);

    // Store updated velocity and pose
    this->last_vel_ = new_vel;
    this->last_pose_ = model_pose_data;

    // model world data (ECI frame)
    this->model_data_.push_back(model_pose_data.Pos().X());
    this->model_data_.push_back(model_pose_data.Pos().Y());
    this->model_data_.push_back(model_pose_data.Pos().Z());

    this->model_data_.push_back(model_pose_data.Rot().W());
    this->model_data_.push_back(model_pose_data.Rot().X());
    this->model_data_.push_back(model_pose_data.Rot().Y());
    this->model_data_.push_back(model_pose_data.Rot().Z());

    this->model_data_.push_back(this->lin_vel_x_.Mean());
    this->model_data_.push_back(this->lin_vel_y_.Mean());
    this->model_data_.push_back(this->lin_vel_z_.Mean());

    this->model_data_.push_back(this->lin_acc_x_.Mean());
    this->model_data_.push_back(this->lin_acc_y_.Mean());
    this->model_data_.push_back(this->lin_acc_z_.Mean());

    this->model_data_.push_back(this->ang_vel_x_.Mean());
    this->model_data_.push_back(this->ang_vel_y_.Mean());
    this->model_data_.push_back(this->ang_vel_z_.Mean());

    this->model_data_.push_back(this->ang_acc_x_.Mean());
    this->model_data_.push_back(this->ang_acc_y_.Mean());
    this->model_data_.push_back(this->ang_acc_z_.Mean());

    this->model_orbit_data_.clear();
    Vector3d model_accel{
      static_cast<double>(this->model_data_[10]),
      static_cast<double>(this->model_data_[11]),
      static_cast<double>(this->model_data_[12])
    };
    auto cart_state = this->orbit_.getCartesian();
    auto state = this->orbit_.integrateStateRK4(cart_state, dt.count(), model_accel);
    // RCLCPP_INFO(this->node_->get_logger(), "[ign_plugin] dt : [%lf]", dt.count());
    this->model_orbit_data_.push_back(state(0));
    this->model_orbit_data_.push_back(state(1));
    this->model_orbit_data_.push_back(state(2));
    this->model_orbit_data_.push_back(state(3));
    this->model_orbit_data_.push_back(state(4));
    this->model_orbit_data_.push_back(state(5));
  }

private:

  std::mutex data_mutex_;  // mutex 멤버 추가
  //===============================================
  // ROS 2 Node, Executor, Thread
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr orbit_node_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr model_info_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr model_orbit_pub_;
  rclcpp::TimerBase::SharedPtr model_info_timer_;
  rclcpp::TimerBase::SharedPtr model_orbit_timer_;
  std::thread ros_spin_thread_;
  bool is_init_ = false;

  void ModelPoseCallback(){
    std::lock_guard<std::mutex> lock(this->data_mutex_);
    std_msgs::msg::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "model pose information";
    msg.layout.dim[0].size = this->model_data_.size();
    msg.layout.dim[0].stride = 1;
    msg.layout.data_offset = 0;
    msg.data = this->model_data_;

    this->model_info_pub_->publish(msg);
    // RCLCPP_INFO(this->node_->get_logger(), "Publishing message to /model_pose at 10 Hz");
  }
  void ModelOrbitCallback(){
    std::lock_guard<std::mutex> lock(this->data_mutex_);
    std_msgs::msg::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "model orbit information";
    msg.layout.dim[0].size = this->model_orbit_data_.size();
    msg.layout.dim[0].stride = 1;
    msg.layout.data_offset = 0;
    msg.data = this->model_orbit_data_;

    this->model_orbit_pub_->publish(msg);
    // RCLCPP_INFO(this->node_->get_logger(), "Publishing message to /model_pose at 10 Hz");
  }
  OrbitMechanics orbit_;
  // If you want the model as well
  std::string model_name_;
  ignition::gazebo::Entity model_entity_{ignition::gazebo::kNullEntity};
  std::vector<float> model_data_;
  std::vector<float> model_orbit_data_;

  ignition::math::Pose3d last_pose_;
  std::vector<double> last_vel_;
  std::chrono::steady_clock::time_point last_update_time_;
  ignition::math::RollingMean lin_vel_x_, lin_vel_y_, lin_vel_z_;
  ignition::math::RollingMean lin_acc_x_, lin_acc_y_, lin_acc_z_;
  ignition::math::RollingMean ang_vel_x_, ang_vel_y_, ang_vel_z_;
  ignition::math::RollingMean ang_acc_x_, ang_acc_y_, ang_acc_z_;

  YAML::Node yaml_config_;
  std::string pkg_share_path_ = ament_index_cpp::get_package_share_directory("ign_plugin");
  std::filesystem::path yaml_file_ = std::filesystem::path(pkg_share_path_) / "config";
}; // class IgnitionPlugin

}  // namespace ign_plugin

// Register the plugin with Ignition
IGNITION_ADD_PLUGIN(
  ign_plugin::IgnitionPlugin, // this is the registration name for this plugin
  ignition::gazebo::System,
  ign_plugin::IgnitionPlugin::ISystemConfigure,
  ign_plugin::IgnitionPlugin::ISystemPreUpdate,
  ign_plugin::IgnitionPlugin::ISystemPostUpdate
)

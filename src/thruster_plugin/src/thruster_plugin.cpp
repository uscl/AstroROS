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

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>

#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>

#include <ignition/math/Vector3.hh>

#include <ignition/plugin/Register.hh>

// Include ROS 2 headers
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#define PI 3.1415926535897931

/// @brief Plugin Template for AstroROS in source file
// Notice namepace and class have to be same as header file.

namespace thruster_plugin // Name your plugin namespace
{

class ThrusterPlugin // Name your plugin class
  : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate,
    public ignition::gazebo::ISystemPostUpdate // these inheritances are fixed.
{
public:

  /// \brief Constructor
  ThrusterPlugin(){

    // We assume ROS has already been initialized externally, but if not:
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);  // or handle arguments properly
    }
  };

  /// \brief Destructor
  ~ThrusterPlugin() override
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
    if (_sdf->HasElement("link_name")) {
      this->link_name_ = _sdf->Get<std::string>("link_name");
    } else {
      std::cout<<"[ERROR] Missing <link_name> parameter in SDF. Check your SDF/Xacro/URDF file."<<std::endl;
      return;
    }
    // Contain ECM for this plugin
    // this->ecm_ = &_ecm;

    this->node_ = std::make_shared<rclcpp::Node>(this->link_name_+"_plugin_node");
    RCLCPP_INFO(this->node_->get_logger(),
            "[%s_plugin] Find model name : [%s]",
            this->link_name_.c_str(), this->link_name_.c_str());

    // ex) topic name should be thru_1_cmd; link_name => thru_1
    this->thruster_cmd_sub_ = this->node_->create_subscription<std_msgs::msg::Float32>(
                            this->link_name_+"_cmd", 10,
                            std::bind(&ThrusterPlugin::CommandCallback, this, std::placeholders::_1)
                          );

    // Make ros thread so that plugin node can operate indepenetly
    this->ros_spin_thread_ = std::thread([this]() {
      rclcpp::executors::MultiThreadedExecutor executor;
      executor.add_node(this->node_);
      RCLCPP_INFO(this->node_->get_logger(),
                "[%s_plugin] ROS 2 MultiThreadedExecution",
                 this->link_name_.c_str());
      executor.spin();  // Use MultiThreadedExecutor for safer threading
    });

    // Attempt to locate the model entity in ECM
    auto model = ignition::gazebo::Model(_entity);
    if (!model.Valid(_ecm)) {
      RCLCPP_ERROR(this->node_->get_logger(),
                 "[%s_plugin] This plugin must be attached to a valid model.",
                 this->link_name_.c_str());
      return;
    }

    auto link_entities = _ecm.ChildrenByComponents(model.Entity(), ignition::gazebo::components::Link());
    auto link_exist  = false;
    for (auto &link_entity : link_entities) {
      auto link_comp_name = _ecm.Component<ignition::gazebo::components::Name>(link_entity);
      if (link_comp_name && link_comp_name->Data() == this->link_name_) {
        this->link_entity_ = link_entity;
        this->link_ = ignition::gazebo::Link(link_entity);
        RCLCPP_INFO(this->node_->get_logger(), "[%s_plugin] Found link [%s] (Entity=%lu).",
        this->link_name_.c_str(),link_comp_name->Data().c_str(),
        static_cast<unsigned long>(link_entity));
        link_exist = true;
      }
    }
    if(!link_exist){
      RCLCPP_ERROR(this->node_->get_logger(),
      "[%s_plugin] Link doesn't exist in this model. Check your SDF/Xacro/URDF file.",
      this->link_name_.c_str());
      return;
    }
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
    auto link_pose = _ecm.Component<ignition::gazebo::components::Pose>(this->link_entity_);
    if (!this->is_init_){
      if (link_pose) {
        this->link_pose_ = link_pose->Data();
        this->is_init_ = true;
        return;
      }
    }
    std::lock_guard<std::mutex> lock(this->data_mutex_);
    this->link_pose_ = link_pose->Data();

    if (this->shot_){
      ignition::math::Vector3d force(0, 0, -this->force_);
      ignition::math::Vector3d torque(0, 0, 0);
      this->link_.AddWorldWrench(_ecm,
      this->link_pose_.Rot().RotateVector(force),
      torque);
      this->force_ = 0;
      this->shot_ = false;
    }
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
    auto link_pose = _ecm.Component<ignition::gazebo::components::Pose>(this->link_entity_);
    if (!link_pose) {
      RCLCPP_WARN(this->node_->get_logger(),
      "[%s_plugin] Model pose component not found.",
      this->link_name_.c_str());
      return;
    }
    std::lock_guard<std::mutex> lock(this->data_mutex_);
    this->link_pose_ = link_pose->Data();
  }

private:

  std::mutex data_mutex_;  // mutex for avoiding race condition
  //===============================================
  // ROS 2 Node, Executor, Thread
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr thruster_cmd_sub_;
  std::thread ros_spin_thread_;
  bool is_init_ = false;

  //===============================================
  // ignition::gazebo::EntityComponentManager* ecm_;

  //===============================================
  std::string link_name_;
  ignition::gazebo::Entity link_entity_{ignition::gazebo::kNullEntity};
  ignition::gazebo::Link link_;
  ignition::math::Pose3d link_pose_;

  //===============================================
  float force_ = 0;
  bool shot_ = false;
  void CommandCallback(const std_msgs::msg::Float32::SharedPtr _msg)
  {
    std::lock_guard<std::mutex> lock(this->data_mutex_);
    // this->link_.AddWorldForce(*this->ecm_,
    // this->link_pose_.Rot().RotateVector(ignition::math::Vector3d(0, 0, -_msg->data)));
    this->force_ = _msg->data;
    this->shot_=true;
  }
}; // class ThrusterPlugin

}  // namespace thruster_plugin

// Register the plugin with Ignition
IGNITION_ADD_PLUGIN(
  thruster_plugin::ThrusterPlugin, // this is the registration name for this plugin
  ignition::gazebo::System,
  thruster_plugin::ThrusterPlugin::ISystemConfigure,
  thruster_plugin::ThrusterPlugin::ISystemPreUpdate,
  thruster_plugin::ThrusterPlugin::ISystemPostUpdate
)

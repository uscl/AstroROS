// C++ standard header
#include <functional>
#include <iostream>
#include <sstream>
#include <thread>

// ros header with message header
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>

// gazebo header 
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>

// ignition header
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Matrix3.hh>
#include <ignition/math/Pose3.hh>

// tf2 header
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>

// there are un-used headers that maybe will be utilized later.
// After development, Un-used headers should be removed for simplicity. 

// Example of custom message publish code
// m_publisher = nh_for_group.advertise<my_robotics_pkg::MyCustomMessageType>("great_custom_topic", queue_size, should_latch);

namespace gazebo
{
  class reaction_wheel_plugin : public ModelPlugin
  // Define plugin name for get plugined model data => Make sure that plugin description should be written in .xacro or .sdf etc files.
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) // Get model's object in gazebo world
    {
      this->model = _model; // Get model

      // Initialize ROS
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = nullptr;
        // Make handler if ros doesn't start
        ros::init(argc, argv, "reaction_wheel_plugin", ros::init_options::NoSigintHandler);
      }
      // Create a ROS node handler
      this->rosNode.reset(new ros::NodeHandle("reaction_wheel_plugin")); // make node handler and reset node
      
      ros::SubscribeOptions torqueSO = ros::SubscribeOptions::create<geometry_msgs::Vector3>("/torque",1,boost::bind(&reaction_wheel_plugin::ApplyTorque, this, _1),
      ros::VoidPtr(), &this->rosQueue);
      ros::SubscribeOptions forceSO = ros::SubscribeOptions::create<geometry_msgs::Vector3>("/force",1,boost::bind(&reaction_wheel_plugin::ApplyForce, this, _1),
      ros::VoidPtr(), &this->rosQueue);

      ros::SubscribeOptions thrusterSO_1 = ros::SubscribeOptions::create<geometry_msgs::Vector3>("/thruster_1",1,boost::bind(&reaction_wheel_plugin::ApplyThrust1, this, _1),
      ros::VoidPtr(), &this->rosQueue);
      ros::SubscribeOptions thrusterSO_2 = ros::SubscribeOptions::create<geometry_msgs::Vector3>("/thruster_2",1,boost::bind(&reaction_wheel_plugin::ApplyThrust2, this, _1),
      ros::VoidPtr(), &this->rosQueue);
      ros::SubscribeOptions thrusterSO_3 = ros::SubscribeOptions::create<geometry_msgs::Vector3>("/thruster_3",1,boost::bind(&reaction_wheel_plugin::ApplyThrust3, this, _1),
      ros::VoidPtr(), &this->rosQueue);
      ros::SubscribeOptions thrusterSO_4 = ros::SubscribeOptions::create<geometry_msgs::Vector3>("/thruster_4",1,boost::bind(&reaction_wheel_plugin::ApplyThrust4, this, _1),
      ros::VoidPtr(), &this->rosQueue);
      ros::SubscribeOptions thrusterSO_5 = ros::SubscribeOptions::create<geometry_msgs::Vector3>("/thruster_5",1,boost::bind(&reaction_wheel_plugin::ApplyThrust5, this, _1),
      ros::VoidPtr(), &this->rosQueue);
      ros::SubscribeOptions thrusterSO_6 = ros::SubscribeOptions::create<geometry_msgs::Vector3>("/thruster_6",1,boost::bind(&reaction_wheel_plugin::ApplyThrust6, this, _1),
      ros::VoidPtr(), &this->rosQueue);
      ros::SubscribeOptions thrusterSO_7 = ros::SubscribeOptions::create<geometry_msgs::Vector3>("/thruster_7",1,boost::bind(&reaction_wheel_plugin::ApplyThrust7, this, _1),
      ros::VoidPtr(), &this->rosQueue);
      ros::SubscribeOptions thrusterSO_8 = ros::SubscribeOptions::create<geometry_msgs::Vector3>("/thruster_8",1,boost::bind(&reaction_wheel_plugin::ApplyThrust8, this, _1),
      ros::VoidPtr(), &this->rosQueue);
      ros::SubscribeOptions thrusterSO_9 = ros::SubscribeOptions::create<geometry_msgs::Vector3>("/thruster_9",1,boost::bind(&reaction_wheel_plugin::ApplyThrust9, this, _1),
      ros::VoidPtr(), &this->rosQueue);
      ros::SubscribeOptions thrusterSO_10 = ros::SubscribeOptions::create<geometry_msgs::Vector3>("/thruster_10",1,boost::bind(&reaction_wheel_plugin::ApplyThrust10, this, _1),
      ros::VoidPtr(), &this->rosQueue);
      ros::SubscribeOptions thrusterSO_11 = ros::SubscribeOptions::create<geometry_msgs::Vector3>("/thruster_11",1,boost::bind(&reaction_wheel_plugin::ApplyThrust11, this, _1),
      ros::VoidPtr(), &this->rosQueue);
      ros::SubscribeOptions thrusterSO_12 = ros::SubscribeOptions::create<geometry_msgs::Vector3>("/thruster_12",1,boost::bind(&reaction_wheel_plugin::ApplyThrust12, this, _1),
      ros::VoidPtr(), &this->rosQueue);


      this->TorqueSub = this->rosNode->subscribe(torqueSO);
      this->ForceSub = this->rosNode->subscribe(forceSO);

      this->ThrustSub1 = this->rosNode->subscribe(thrusterSO_1);
      this->ThrustSub2 = this->rosNode->subscribe(thrusterSO_2);
      this->ThrustSub3 = this->rosNode->subscribe(thrusterSO_3);
      this->ThrustSub4 = this->rosNode->subscribe(thrusterSO_4);
      this->ThrustSub5 = this->rosNode->subscribe(thrusterSO_5);
      this->ThrustSub6 = this->rosNode->subscribe(thrusterSO_6);
      this->ThrustSub7 = this->rosNode->subscribe(thrusterSO_7);
      this->ThrustSub8 = this->rosNode->subscribe(thrusterSO_8);
      this->ThrustSub9 = this->rosNode->subscribe(thrusterSO_9);
      this->ThrustSub10 = this->rosNode->subscribe(thrusterSO_10);
      this->ThrustSub11 = this->rosNode->subscribe(thrusterSO_11);
      this->ThrustSub12 = this->rosNode->subscribe(thrusterSO_12);

      this->rosQueueThread = std::thread(std::bind(&reaction_wheel_plugin::QueueThread, this));
      
      // Publish "translation" topics
      this->translationPositionPub = this->rosNode->advertise<geometry_msgs::Vector3>("transition_position", 1);
      this->translationVelocityPub = this->rosNode->advertise<geometry_msgs::Vector3>("transition_velocity", 1);
      this->translationAccelerationPub = this->rosNode->advertise<geometry_msgs::Vector3>("transition_acceleration", 1);
      
      // Publish "angular" topics
      this->angularPositionPub = this->rosNode->advertise<geometry_msgs::Vector3>("rotation_position", 1);
      this->angularVelocityPub = this->rosNode->advertise<geometry_msgs::Vector3>("rotation_velocity", 1); //++++
      this->angularAccelerationPub = this->rosNode->advertise<geometry_msgs::Vector3>("rotation_acceleration", 1);
      
      // Publish "quaternion" topics
      this->quaternionPub = this->rosNode->advertise<geometry_msgs::Quaternion>("quaternion",1);

      // Update event to ROS and Gazeboc
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&reaction_wheel_plugin::OnUpdate, this)); 
    }
    
    void ApplyTorque(const geometry_msgs::Vector3::ConstPtr& _msg)
    {
      ignition::math::Vector3d t(_msg->x, _msg->y, _msg->z);
      this->model->GetLink("dummy_base_link")->AddRelativeTorque(t);
    }

    void ApplyForce(const geometry_msgs::Vector3::ConstPtr& _msg)
    {
      ignition::math::Vector3d t(_msg->x, _msg->y, _msg->z);
      this->model->GetLink("dummy_base_link")->AddRelativeForce(t);
    }

    void ApplyThrust1(const geometry_msgs::Vector3::ConstPtr& _msg)
    {
      ignition::math::Vector3d t(_msg->x, _msg->y, -_msg->z);
      this->model->GetLink("thruster_1")->AddRelativeForce(t);
    }
    void ApplyThrust2(const geometry_msgs::Vector3::ConstPtr& _msg)
    {
      ignition::math::Vector3d t(_msg->x, _msg->y, -_msg->z);
      this->model->GetLink("thruster_2")->AddRelativeForce(t);
    }
    void ApplyThrust3(const geometry_msgs::Vector3::ConstPtr& _msg)
    {
      ignition::math::Vector3d t(_msg->x, _msg->y, -_msg->z);
      this->model->GetLink("thruster_3")->AddRelativeForce(t);
    }
    void ApplyThrust4(const geometry_msgs::Vector3::ConstPtr& _msg)
    {
      ignition::math::Vector3d t(_msg->x, _msg->y, -_msg->z);
      this->model->GetLink("thruster_4")->AddRelativeForce(t);
    }
    void ApplyThrust5(const geometry_msgs::Vector3::ConstPtr& _msg)
    {
      ignition::math::Vector3d t(_msg->x, _msg->y, -_msg->z);
      this->model->GetLink("thruster_5")->AddRelativeForce(t);
    }
    void ApplyThrust6(const geometry_msgs::Vector3::ConstPtr& _msg)
    {
      ignition::math::Vector3d t(_msg->x, _msg->y, -_msg->z);
      this->model->GetLink("thruster_6")->AddRelativeForce(t);
    }
    void ApplyThrust7(const geometry_msgs::Vector3::ConstPtr& _msg)
    {
      ignition::math::Vector3d t(_msg->x, _msg->y, -_msg->z);
      this->model->GetLink("thruster_7")->AddRelativeForce(t);
    }
    void ApplyThrust8(const geometry_msgs::Vector3::ConstPtr& _msg)
    {
      ignition::math::Vector3d t(_msg->x, _msg->y, -_msg->z);
      this->model->GetLink("thruster_8")->AddRelativeForce(t);
    }
    void ApplyThrust9(const geometry_msgs::Vector3::ConstPtr& _msg)
    {
      ignition::math::Vector3d t(_msg->x, _msg->y, -_msg->z);
      this->model->GetLink("thruster_9")->AddRelativeForce(t);
    }
    void ApplyThrust10(const geometry_msgs::Vector3::ConstPtr& _msg)
    {
      ignition::math::Vector3d t(_msg->x, _msg->y, -_msg->z);
      this->model->GetLink("thruster_10")->AddRelativeForce(t);
    }
    void ApplyThrust11(const geometry_msgs::Vector3::ConstPtr& _msg)
    {
      ignition::math::Vector3d t(_msg->x, _msg->y, -_msg->z);
      this->model->GetLink("thruster_11")->AddRelativeForce(t);
    }
    void ApplyThrust12(const geometry_msgs::Vector3::ConstPtr& _msg)
    {
      ignition::math::Vector3d t(_msg->x, _msg->y, -_msg->z);
      this->model->GetLink("thruster_12")->AddRelativeForce(t);
    }

    void OnUpdate()
    {
      // if (flag == 0){
      // this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 0.01)); //++++++
        // flag = 1;
      // }
      
      // Get current angular informations
      ignition::math::Vector3d euler = this->model->WorldPose().Rot().Euler();            // unit : [rad]
      ignition::math::Vector3d angularVelocity = this->model->WorldAngularVel();          // unit : [rad/s]
      ignition::math::Vector3d angularAcceleration = this->model->WorldAngularAccel();    // unit : [rad/s^2]
      
      // Get current translation informations
      ignition::math::Vector3d translationPosition = this->model->WorldPose().Pos();      // unit : [m]
      ignition::math::Vector3d translationVelocity = this->model->WorldLinearVel();       // unit : [m/s]
      ignition::math::Vector3d translationAcceleration = this->model->WorldLinearAccel(); // unit : [m/s^2]
      
      // Get current quaternion information
      ignition::math::Quaterniond quaternion = this->model->WorldPose().Rot();             //unit : [1/z]
      quaternion.Normalize(); //++++++
      // // Convert from radians to degrees
      // euler *= 180.0 / M_PI; 
      // // Reset rotation if it reaches 360 degrees
      // for (int i = 0; i < 3; i++)
      // {
      //   while (euler[i] >= 360.0)
      //     euler[i] -= 360.0;
      //   while (euler[i] < 0.0)
      //     euler[i] += 360.0;
      // }

      // Publish translation informations as ROS topics
      geometry_msgs::Vector3 transPosMsg;
      // transPosMsg.header.stamp = ros::Time::now();
      transPosMsg.x = translationVelocity.X();
      transPosMsg.y = translationVelocity.Y();
      transPosMsg.z = translationVelocity.Z();
      this->translationVelocityPub.publish(transPosMsg);

      geometry_msgs::Vector3 transVelMsg;
      // transVelMsg.header.stamp = ros::Time::now();
      transVelMsg.x = translationPosition.X();
      transVelMsg.y = translationPosition.Y();
      transVelMsg.z = translationPosition.Z();
      this->translationPositionPub.publish(transVelMsg);

      geometry_msgs::Vector3 transAccelMsg;
      // transAccelMsg.header.stamp = ros::Time::now();
      transAccelMsg.x = translationAcceleration.X();
      transAccelMsg.y = translationAcceleration.Y();
      transAccelMsg.z = translationAcceleration.Z();
      this->translationAccelerationPub.publish(transAccelMsg);

      // Publish angular infomations as ROS topics
      geometry_msgs::Vector3 anglePosMsg;
      // anglePosMsg.header.stamp = ros::Time::now();
      anglePosMsg.x = euler.X();
      anglePosMsg.y = euler.Y();
      anglePosMsg.z = euler.Z();
      this->angularPositionPub.publish(anglePosMsg);

      // geometry_msgs::Vector3Stamped angularVelMsg;
      geometry_msgs::Vector3 angularVelMsg; //+++
      // angularVelMsg.header.stamp = ros::Time::now();
      // angularVelMsg.vector.x = angularVelocity.X();
      // angularVelMsg.vector.y = angularVelocity.Y();
      // angularVelMsg.vector.z = angularVelocity.Z();
      angularVelMsg.x = angularVelocity.X();//+++
      angularVelMsg.y = angularVelocity.Y();//+++
      angularVelMsg.z = angularVelocity.Z();//+++
      this->angularVelocityPub.publish(angularVelMsg);

      geometry_msgs::Vector3 angularAccelMsg;
      // angularAccelMsg.header.stamp = ros::Time::now();
      angularAccelMsg.x = angularAcceleration.X();
      angularAccelMsg.y = angularAcceleration.Y();
      angularAccelMsg.z = angularAcceleration.Z();
      this->angularAccelerationPub.publish(angularAccelMsg);

      // Publish quaternion informations as ROS topics
      geometry_msgs::Quaternion quaternionMsg;
      // quaternionMsg.header.stamp = ros::Time::now();
      quaternionMsg.x = quaternion.X();
      quaternionMsg.y = quaternion.Y();
      quaternionMsg.z = quaternion.Z();
      quaternionMsg.w = quaternion.W();
      this->quaternionPub.publish(quaternionMsg);
    }

  private:
    // gazebo object members
    physics::ModelPtr model;                    // target selection => "one script one model" for simplicity
    event::ConnectionPtr updateConnection;      // engine event updater

    // ROS-related members
    std::unique_ptr<ros::NodeHandle> rosNode;   // node generator

    ros::Publisher angularPositionPub;          // angular position
    ros::Publisher angularVelocityPub;          // angular velocity
    ros::Publisher angularAccelerationPub;      // angular acceleration
    ros::Publisher translationPositionPub;      // translation position
    ros::Publisher translationVelocityPub;      // translation velocity
    ros::Publisher translationAccelerationPub;  // translation acceleration
    ros::Publisher quaternionPub;               // quaternion

    ros::Subscriber TorqueSub;                  // Torque
    ros::Subscriber ForceSub;                   // Force

    ros::Subscriber ThrustSub1;
    ros::Subscriber ThrustSub2;
    ros::Subscriber ThrustSub3;              
    ros::Subscriber ThrustSub4;
    ros::Subscriber ThrustSub5;
    ros::Subscriber ThrustSub6;
    ros::Subscriber ThrustSub7;
    ros::Subscriber ThrustSub8;
    ros::Subscriber ThrustSub9;
    ros::Subscriber ThrustSub10;
    ros::Subscriber ThrustSub11;
    ros::Subscriber ThrustSub12;                  // Thruster Force

    // A thread the keeps running the rosQueue
    ros::CallbackQueue rosQueue;                // subscriber queue
    // A thread the keeps running the rosQueue
    std::thread rosQueueThread;                 // thread control
    // int flag = 0; //++++++
    
    void QueueThread()
    {
      static const double timeout = 0.001;
      while (this->rosNode->ok()){
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
  };

  // Upload plugin register into gazebo engine so that this script makes {lib(register name).so} file (share file)
  GZ_REGISTER_MODEL_PLUGIN(reaction_wheel_plugin)
}
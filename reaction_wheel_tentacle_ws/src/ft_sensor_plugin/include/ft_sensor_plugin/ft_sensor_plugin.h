#ifndef GAZEBO_FT_SENSOR_PLUGIN_H
#define GAZEBO_FT_SENSOR_PLUGIN_H

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Event.hh>

#include <gazebo/sensors/sensors.hh>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/Wrench.h>

namespace gazebo{
    class ftSensor : public ModelPlugin{
        public : ftSensor();
        public : virtual ~ftSensor();

        public : void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        protected : virtual void UpdateChild();

        protected : double gaussian_noise_;

        private : double GaussianKernel(double mu, double sigma);

        private : physics::JointPtr joint_;

        private : physics::LinkPtr parent_link_;

        private : physics::LinkPtr child_link_;

        private : physics::ModelPtr model_;

        private : physics::WorldPtr world_;

        private : ros::NodeHandle* rosnode_;
        private : ros::Publisher pub_;

        private : geometry_msgs::Wrench wrench_msg_;

        private : std::string joint_name_;
        private : std::string topic_name_;
        private : std::string robot_namespace_;
        private : std::string frame_name_;

        private : boost::mutex lock_;
        private : boost::thread callback_queue_thread_;

        private : common::Time last_time_;

        private : double update_rate_;

        private : int ft_connect_count_;

        private : void ftConnect();
        private : void ftDisConnect();

        private : ros::CallbackQueue queue_;
        private : void QueueThread();

        private : event::ConnectionPtr update_connection_;
    };
}

#endif
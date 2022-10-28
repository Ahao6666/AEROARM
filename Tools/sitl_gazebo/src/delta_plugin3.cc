#ifndef _DELTA_ROS_PlUGIN3_HH_
#define _DELTA_ROS_PlUGIN3_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  	// A plugin to control a Manipulator sensor.
	class DeltaROSPlugin3 : public ModelPlugin
	{
    		// Constructor
		public: DeltaROSPlugin3() {}

		// Handle an incoming message from ROS
		public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
		{

			this->SSetPosition(_msg->data);
		}

		// Set the joint's target position
		public: void SSetPosition(const double &_posi)
		{

  			this->model->GetJointController()->SetPositionTarget(
      			this->joint->GetScopedName(), _posi);
      			std_msgs::Float32 msg;
 			msg.data =this->joint->Position();
      			this->posi_pub.publish(msg);
		}

		// ROS helper function that processes messages
		private: void QueueThread()
		{
  			static const double timeout = 0.01;
  			while (this->rosNode->ok())
  			{
    				this->rosQueue.callAvailable(ros::WallDuration(timeout));
  			}
		}

		// Pointer to the model.
		private: physics::ModelPtr model;


		// Pointer to the joint.
		private: physics::JointPtr joint;

		// A PID controller for the joint.
		private: common::PID pid;

		// A node use for ROS transport
		private: std::unique_ptr<ros::NodeHandle> rosNode;

		// A ROS subscriber
		private: ros::Subscriber rosSub;
		private: ros::Publisher posi_pub;
		// A ROS callbackqueue that helps process messages
		private: ros::CallbackQueue rosQueue;

		// A thread the keeps running the rosQueue
		private: std::thread rosQueueThread;


		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
      			// Just output a message for now

  			// Store the model pointer for convenience.
  			this->model = _model;
  			this->joint = _model->GetJoints()[8];

			// Plugin location
			std::cerr << "\nThe delta ROS plugin 3 is attach to joint[" <<
        		this->joint->GetName() << "]\n";

  			// Setup a PID-controller, with a gain of 5,0.5,0.2.
  			this->pid = common::PID(2.0,0.01,0.02);

  			// Apply the P-controller to the joint.
  			this->model->GetJointController()->SetPositionPID(
      			this->joint->GetScopedName(), this->pid);
			if (!ros::isInitialized())
			{
  				int argc = 0;
  				char **argv = NULL;
  				ros::init(argc, argv, "gazebo_manipulator_client",
      				ros::init_options::NoSigintHandler);
			}

			// Create our ROS node. This acts in a similar manner to
			// the Gazebo node
			this->rosNode.reset(new ros::NodeHandle("gazebo_manipulator_client"));
			ros::NodeHandle n;
 			this->posi_pub = n.advertise<std_msgs::Float32>("joint/3/position_msg", 1000);

			// Create a named topic, and subscribe to it.
			ros::SubscribeOptions so =
  			ros::SubscribeOptions::create<std_msgs::Float32>(
      			"/joint/3/position_cmd",
      			0.6,
      			boost::bind(&DeltaROSPlugin3::OnRosMsg, this, _1),
      			ros::VoidPtr(), &this->rosQueue);
			this->rosSub = this->rosNode->subscribe(so);

			// Spin up the queue helper thread.
			this->rosQueueThread =
  			std::thread(std::bind(&DeltaROSPlugin3::QueueThread, this));
    		}
  	};

	// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
 	GZ_REGISTER_MODEL_PLUGIN(DeltaROSPlugin3)
}
#endif

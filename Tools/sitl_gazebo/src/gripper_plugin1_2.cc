#ifndef _GRIPPER_PLUGIN1_2_HH_
#define _GRIPPER_PLUGIN1_2_HH_

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
  	/// \brief A plugin to control a Manipulator sensor.
	class GripperROSPlugin1_2 : public ModelPlugin
	{
    		/// \brief Constructor
		public: GripperROSPlugin1_2() {}

		/// \brief Handle an incoming message from ROS
		/// \param[in] _msg A float value that is used to set the velocity
		/// of the Manipulator.
		public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
		{
//			double posi=_msg->data;
//  			bool r=this->joint->SetPosition(0,posi,true);
//			printf("%d \n",r);
//      			std_msgs::Float32 msg;
// 			msg.data =this->joint->Position();
//      			this->posi_pub.publish(msg);
			this->SSetPosition(_msg->data);
		}



		/// Set the joint's target velocity.
		public: void SSetPosition(const double &_vel)
		{

  			this->model->GetJointController()->SetPositionTarget(
      			this->joint->GetScopedName(), _vel);
//      			std::cerr << "\nSetting successful.\n";
			//printf("%f \n", _vel);
      			std_msgs::Float32 msg;
 			msg.data =this->joint->Position();
      			this->posi_pub.publish(msg);
		}

		/// \brief ROS helper function that processes messages
		private: void QueueThread()
		{
  			static const double timeout = 0.01;
  			while (this->rosNode->ok())
  			{
    				this->rosQueue.callAvailable(ros::WallDuration(timeout));
  			}
		}

		/// \brief Pointer to the model.
		private: physics::ModelPtr model;


		/// \brief Pointer to the joint.
		private: physics::JointPtr joint;

		/// \brief A PID controller for the joint.
		private: common::PID pid;

		/// \brief A node use for ROS transport
		private: std::unique_ptr<ros::NodeHandle> rosNode;

		/// \brief A ROS subscriber
		private: ros::Subscriber rosSub;
		private: ros::Publisher posi_pub;
		/// \brief A ROS callbackqueue that helps process messages
		private: ros::CallbackQueue rosQueue;

		/// \brief A thread the keeps running the rosQueue
		private: std::thread rosQueueThread;

    		/// \brief The load function is called by Gazebo when the plugin is
    		/// inserted into simulation
    		/// \param[in] _model A pointer to the model that this plugin is
    		/// attached to.
    		/// \param[in] _sdf A pointer to the plugin's SDF element.
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
      			// Just output a message for now

//  			if (_model->GetJointCount() == 0)
//  			{
//    				std::cerr << "Invalid joint count, Manipulator plugin not loaded\n";
//    				return;
//  			}

  			// Store the model pointer for convenience.
  			this->model = _model;
  			this->joint = _model->GetJoints()[13];
			std::cerr << "\nThe gripper1 ROS plugin 2 is attach to joint[" <<
        		this->joint->GetName() << "]\n";


  			// Setup a P-controller, with a gain of 0.1.
  			this->pid = common::PID(10, 0, 0.1);

  			// Apply the P-controller to the joint.
  			this->model->GetJointController()->SetPositionPID(
      			this->joint->GetScopedName(), this->pid);

//  			// Set the joint's target velocity. This target velocity is just
//  			// for demonstration purposes.
//  			this->model->GetJointController()->SetVelocityTarget(
//      			this->joint->GetScopedName(), 10.0);
//			std::cerr << "\nThe manipulator plugin is attach to model[" <<
//        		_model->GetJointCount() << "]\n";


			if (!ros::isInitialized())
			{
  				int argc = 0;
  				char **argv = NULL;
  				ros::init(argc, argv, "gazebo_manipulator_client_gripper1_2",
      				ros::init_options::NoSigintHandler);
			}



			// Create our ROS node. This acts in a similar manner to
			// the Gazebo node
			this->rosNode.reset(new ros::NodeHandle("gazebo_manipulator_client_gripper1_2"));
			ros::NodeHandle n;
 			this->posi_pub = n.advertise<std_msgs::Float32>("joint/gripper1_2/position_msg", 10);
			// Create a named topic, and subscribe to it.
			ros::SubscribeOptions so =
  			ros::SubscribeOptions::create<std_msgs::Float32>(
      			"/joint/gripper1_2/position_cmd",
      			0,
      			boost::bind(&GripperROSPlugin1_2::OnRosMsg, this, _1),
      			ros::VoidPtr(), &this->rosQueue);
			this->rosSub = this->rosNode->subscribe(so);

			// Spin up the queue helper thread.
			this->rosQueueThread =
  			std::thread(std::bind(&GripperROSPlugin1_2::QueueThread, this));
  			this->model->GetJointController()->SetPositionTarget(
      			this->joint->GetScopedName(), 0);

    		}
  	};

	// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
 	GZ_REGISTER_MODEL_PLUGIN(GripperROSPlugin1_2)
}
#endif

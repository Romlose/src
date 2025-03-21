#pragma once 

#include <urdf/model.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "sensor_msgs/JointState.h"
#include <geometry_msgs/WrenchStamped.h>

#include <boost/thread/condition.hpp>
#include <boost/scoped_ptr.hpp>
#include <memory.h>

#include "kuka_controllers/KukaState.h"


#ifdef TESTING
#define TESTABLE_PUBLIC public
#define TESTABLE_PRIVATE public
#else
#define TESTABLE_PUBLIC public
#define TESTABLE_PRIVATE private
#endif

namespace kuka_controllers{

    
class KukaStateController: public controller_interface::Controller<hardware_interface::EffortJointInterface>{

        TESTABLE_PUBLIC:

            KukaStateController();
            ~KukaStateController();

            bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
            void starting(const ros::Time& time);
            void update(const ros::Time& time, const ros::Duration& period);
            void stopping(const ros::Time& time);

        TESTABLE_PRIVATE:

            void ext_wrench_cb(const geometry_msgs::WrenchStamped::ConstPtr &wrench_msg);

            ros::NodeHandle nh_;
            KDL::Chain kdl_chain_;
            KDL::Tree kdl_tree_;
            std::vector<std::string> joint_names_;
            

            std::vector<KDL::Wrench> Wrenches;
            KDL::JntArrayAcc joint_states_;
            KDL::JntArray torques_;
            KDL::JntArray js_torques_;
            KDL::Jacobian jacobian_;
            std::unique_ptr<KDL::ChainIdSolver_RNE> id_solver_;
            std::vector<hardware_interface::JointHandle> joint_handles_;

            std::shared_ptr<realtime_tools::RealtimePublisher<kuka_controllers::KukaState> > realtime_pub_;
    };

}
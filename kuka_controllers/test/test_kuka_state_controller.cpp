#include <memory.h>
#include <gtest/gtest.h>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <kdl/kdl.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <controller_interface/controller.h>
#include <gazebo_msgs/GetPhysicsProperties.h>
#include <hardware_interface/joint_command_interface.h>

#define TESTING
#include <kuka_controllers/kuka_kuka_state_controller.h>

namespace kuka_controllers{

class TestKukaStateController : public ::testing::Test
{
    protected:

        void SetUp() override { 

            int argc = 0; 
            char** argv = nullptr;
            ros::init(argc, argv, "SetUpTesting");
            nh_ = ros::NodeHandle("~");

            loadRobotModel();

            ext_wrenches_.reset(new KDL::Wrenches(kdl_chain_->getNrOfJoints()));
            ros::Subscriber ext_wrenches_sub = nh_.subscribe<geometry_msgs::WrenchStamped>
                                                    ("ext_wrenches", 10, &TestKukaStateController::ext_Wrenches_CB, this);
        }

        void loadRobotModel(){ 

            ASSERT_TRUE(ros::param::search(nh_.getNamespace(),"robot_description", robot_description_));

            ASSERT_TRUE(nh_.getParam("root_name", root_name_));
            ASSERT_TRUE(nh_.getParam("tip_name", tip_name_));

            ASSERT_TRUE(nh_.hasParam(robot_description_));
            nh_.getParam(robot_description_.c_str(), xml_string_);

            EXPECT_GT(xml_string_.size(), 0u);

            model_ = std::make_unique<urdf::Model>();
            ASSERT_TRUE(model_->initString(xml_string_));

            kdl_tree_ = std::make_unique<KDL::Tree>();
            ASSERT_TRUE(kdl_parser::treeFromUrdfModel(*model_, *kdl_tree_));

            kdl_chain_ = std::make_unique<KDL::Chain>();
            ASSERT_TRUE(kdl_tree_->getChain(root_name_, tip_name_, *kdl_chain_));
        }

        void initKukaController(){

            controller_.reset(new kuka_controllers::KukaStateController());

            ros::NodeHandle ctrl_nh(nh_, "controller");
            ctrl_nh.setParam("root_name", "base_link");
            ctrl_nh.setParam("tip_name", "tool0");
            
            robot_ = std::make_unique<hardware_interface::EffortJointInterface>();
            ASSERT_TRUE(controller_->init(robot_.get(), ctrl_nh));
        }

        void ext_Wrenches_CB(const geometry_msgs::WrenchStamped::ConstPtr &msg){
            KDL::Wrench w(KDL::Vector(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z), 
                          KDL::Vector(msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z));
            
            ext_wrenches_->back() = w;
        }

        void TearDown() override {
            controller_->stopping(ros::Time::now());
            ros::shutdown();
        }
        
        ros::NodeHandle nh_;
        std::string robot_description_;
        std::string root_name_;
        std::string tip_name_;
        std::string xml_string_;
        
        KDL::JntArrayAcc joint_states_;

        std::unique_ptr<urdf::Model> model_;
        std::unique_ptr<KDL::Tree> kdl_tree_;
        std::unique_ptr<KDL::Chain> kdl_chain_;
        std::unique_ptr<KDL::Wrenches> ext_wrenches_;
        std::unique_ptr<hardware_interface::EffortJointInterface> robot_;
        std::unique_ptr<kuka_controllers::KukaStateController> controller_;
};

TEST_F(TestKukaStateController, TestNullPtr){

    EXPECT_NE(model_, nullptr);
    EXPECT_NE(kdl_tree_, nullptr);
    EXPECT_NE(kdl_chain_, nullptr);
    EXPECT_NE(robot_, nullptr);
}

TEST_F(TestKukaStateController, InitSuccess){

    EXPECT_EQ(controller_->KukaStateController::kdl_chain_.getNrOfJoints(), 6);
    EXPECT_EQ(controller_->KukaStateController::joint_handles_.size(), 6);
    EXPECT_NE(controller_->KukaStateController::id_solver_, nullptr);
}

TEST_F(TestKukaStateController, ExternalWrenchProcessing) {

    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.stamp = ros::Time::now();
    wrench_msg.wrench.force.x = 10.0;
    wrench_msg.wrench.torque.z = 5.0;
    
    controller_->ext_wrench_cb(
        boost::make_shared<geometry_msgs::WrenchStamped>(wrench_msg)
    );
    
    ASSERT_FALSE(controller_->Wrenches.empty());

    KDL::Wrench last_wrench = controller_->Wrenches.back();
    EXPECT_DOUBLE_EQ(last_wrench.force.x(), 10.0);
    EXPECT_DOUBLE_EQ(last_wrench.torque.z(), 5.0);
}

TEST_F(TestKukaStateController, TestInverseDynamicSolver){

    std::unique_ptr<KDL::ChainIdSolver_RNE> id_solver = 
                        std::make_unique<KDL::ChainIdSolver_RNE>(*kdl_chain_, KDL::Vector(0.0, 0.0, -9.81));

    for(int i = 0; i < 6; i++){
        joint_states_.q(i) = 0;
        joint_states_.qdot(i) = 0; 
        joint_states_.qdotdot(i) = 0;
    }

    KDL::JntArray torques(kdl_chain_->getNrOfJoints());
    SetToZero(torques);

    int result = id_solver->CartToJnt(
                            joint_states_.q, 
                            joint_states_.qdot, 
                            joint_states_.qdotdot, 
                            *ext_wrenches_,
                            torques
                            );

    ASSERT_EQ(result, 0);

    
    bool gravity_enable = false;

    ros::ServiceClient client = nh_.serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties");

    gazebo_msgs::GetPhysicsProperties srv;
    if (client.call(srv)) {
        gravity_enable = (srv.response.gravity.z > -0.2);
    }


    if(!gravity_enable){

        EXPECT_NEAR(torques(0), 0.0, 1e-3);
    }
    
    EXPECT_FALSE(torques.data.isZero());
}

TEST_F(TestKukaStateController, RealtimePublisherIntegration){

    auto& rt_pub = controller_->realtime_pub_;

    EXPECT_EQ(rt_pub->msg_.joint_name.size(), 6);
    EXPECT_EQ(rt_pub->msg_.torque_error.size(), 6);

    if(rt_pub->trylock()) {
        rt_pub->msg_.header.stamp = ros::Time::now();
        for(size_t i = 0; i < 6; ++i) {
            rt_pub->msg_.joint_name[i] = "test_joint";
            rt_pub->msg_.torque_error[i] = i * 0.1;
        }
        rt_pub->unlockAndPublish();
    }
}

TEST_F(TestKukaStateController, UpdateStateProcessing) {
    
    for(auto handle : controller_->joint_handles_) {
        handle.setCommand(10.0);
    }
      
    for(size_t i = 0; i < 6; ++i) {
        EXPECT_DOUBLE_EQ(controller_->joint_handles_[i].getCommand(), 10.0);
    }

    EXPECT_EQ(controller_->joint_handles_[3].getName(), "joint_a4");
}

TEST_F(TestKukaStateController, InvalidConfigurationHandling) {
    
    ros::NodeHandle bad_nh(nh_, "bad_controller");
    bad_nh.setParam("root_name", "invalid_root");
    bad_nh.setParam("tip_name", "invalid_tip");
    
    kuka_controllers::KukaStateController bad_controller;
    EXPECT_FALSE(bad_controller.init(nullptr, bad_nh));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_kuka_state_controller");
    return RUN_ALL_TESTS();
}

} //kuka_controllers 
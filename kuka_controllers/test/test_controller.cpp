#include <gtest/gtest.h>
#include <ros/ros.h>
#include <class_loader/class_loader.hpp>
#include <controller_interface/controller.h>

#include <kuka_controllers/kuka_kuka_state_controller.h>
#define TESTING


class TestController : public ::testing::Test{

    protected:

    void SetUp() override {

      nh_ = ros::NodeHandle();

      ros::param::set("/root_name", "base_link");
      ros::param::set("/tip_name", "tool0");

      createKukaStateController();

      initController();
    }

    void createKukaStateController(){

      try {

        std::string library_path = "/home/rosuser/catkin_ws/devel/lib/libkuka_controllers.so";
        loader_ = std::make_unique<class_loader::ClassLoader>(library_path);
    
        controller_base_ = loader_->createInstance<controller_interface::ControllerBase>(
          "kuka_controllers::KukaStateController"
        );
    
        controller_ = boost::dynamic_pointer_cast<kuka_controllers::KukaStateController>(controller_base_);
        
        if (!controller_) {
          throw std::runtime_error("Failed to cast controller to KukaStateController");
        }
    
      } catch (const class_loader::ClassLoaderException& e) {

        ROS_ERROR("Error of creation: %s", e.what());
        FAIL();

      }

    }

    void initController(){ 

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

    void TearDown() override{

      ros::shutdown();

    }

    protected:

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

    std::unique_ptr<class_loader::ClassLoader> loader_;
    boost::shared_ptr<controller_interface::ControllerBase> controller_base_;
    boost::shared_ptr<kuka_controllers::KukaStateController> controller_;

};

TEST_F(TestController, Subtest_1) {
    ASSERT_TRUE(1 == 1);
}

TEST_F(TestController, Subtest_2) {
    ASSERT_FALSE('b' != 'b');
  
}

TEST_F(TestController, TestNullPtr){

  EXPECT_NE(model_, nullptr);
  EXPECT_NE(kdl_tree_, nullptr);
  EXPECT_NE(kdl_chain_, nullptr);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_kuka_state_controller");
    return RUN_ALL_TESTS();
}


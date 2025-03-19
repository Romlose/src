#include <gtest/gtest.h>

#define TESTING 
#include <kuka_controllers/kuka_kuka_state_controller.h>

class TestController : public ::testing::Test{

    protected:

      void SetUp() override{

          controller = std::make_unique<kuka_controllers::KukaStateController>();
      }

      void TearDown() override{

      }
    
    private:
      
      std::unique_ptr<kuka_controllers::KukaStateController> controller;
      std::unique_ptr<hardware_interface::JointHandle> robot;
};

TEST(TestGroupName, Subtest_1) {
    ASSERT_TRUE(1 == 1);
}

TEST(TestGroupName, Subtest_2) {
    ASSERT_FALSE('b' == 'b');
  
}

TEST(TestGroupName, Subtest_3){

    ros::NodeHandle n;
    std::string robot_description, root_name, tip_name;

    ASSERT_TRUE(ros::param::search(n.getNamespace(),"robot_description", robot_description));

}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_kuka_state_controller");
    return RUN_ALL_TESTS();
}


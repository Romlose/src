#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <fstream>

#include "ros/ros.h"
#include "kdl/jntarray.hpp"
#include "kdl/chain.hpp"
#include <kdl/tree.hpp>
#include <kdl/jacobian.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

#include "sensor_msgs/JointState.h"
#include <geometry_msgs/WrenchStamped.h>

std::vector<std::string> joint_names;
KDL::JntArray q(6);
KDL::JntArray q_dot(6);
KDL::JntArray torque(6);
KDL::JntArray q_dot_dot(6);
std::vector<KDL::Wrench> Wrenches;

std::ofstream log_file;


void ftCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    KDL::Wrench w(KDL::Vector(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z), 
                  KDL::Vector(msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z));
    Wrenches.push_back(w); 
    ros::Duration(1.0).sleep();
}

void qPRNT(const KDL::JntArray& q){
    std::string output = "Angles: [" + std::to_string(q(0)) + ",  "
        + std::to_string(q(1)) + ",  "
        + std::to_string(q(2)) + ",  "
        + std::to_string(q(3)) + ",  "
        + std::to_string(q(4)) + ",  " 
        + std::to_string(q(5)) + "];";
    ROS_INFO_STREAM(output);
    log_file << output << std::endl;
}

void vPRNT(const KDL::JntArray& v){
    std::string output = "Revolute joint velocities: ["
    + std::to_string(v(0)) + ",  "
    + std::to_string(v(1)) + ",  "
    + std::to_string(v(2)) + ",  "
    + std::to_string(v(3)) + ",  "
    + std::to_string(v(4)) + ",  "
    + std::to_string(v(5)) + "];\n";
    ROS_INFO_STREAM(output);
    log_file << output << std::endl;
}

void JACPRNT(const KDL::Jacobian JAC){  
    int rows = JAC.rows(); // Получаем количество строк
    int cols = JAC.columns(); // Получаем количество столбцов

    std::stringstream ss;  // Используем stringstream для формирования строки
    ss << "[";
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            ss << JAC(i, j) << '\t'; // Добавляем элементы строки в стрим
            if (j < cols - 1) {
                ss << ", "; // Добавляем запятую, если это не последний элемент в строке
            }
        }
        if (i < rows - 1) {
            ss << ";\n"; // Добавляем разделитель между строками, если это не последняя строка
        }
    }
    ss << "];"; // Заканчиваем с закрывающей квадратной скобкой
    ROS_INFO_STREAM("JAC:" << ss.str()); // Выводим строку
    log_file << ss.str() << std::endl;
}

void jsCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for(size_t i = 0; i < msg->name.size(); i++){
        joint_names.push_back(msg->name[i]);
        q(i) = msg->position[i];
        q_dot(i) = msg->velocity[i]; 
        torque(i) = msg->effort[i];
    }
}

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "KUKA");
	ros::NodeHandle n;
	
    log_file.open("/home/rosuser/catkin_ws/src/kdl_dynamics/src/output_log.txt",
    std::ios::out | std::ios::trunc);
    if (!log_file.is_open()) {
        ROS_ERROR("Failed to open log file.");
        return -1;
    }

    ros::Subscriber sub = n.subscribe("/joint_states", 1, jsCallback);
    ros::Subscriber sub_ft_a1 = n.subscribe("FT_a1", 1, ftCallback);
    ros::Subscriber sub_ft_a2 = n.subscribe("FT_a2", 1, ftCallback);
    ros::Subscriber sub_ft_a3 = n.subscribe("FT_a3", 1, ftCallback);
    ros::Subscriber sub_ft_a4 = n.subscribe("FT_a4", 1, ftCallback);
    ros::Subscriber sub_ft_a5 = n.subscribe("FT_a5", 1, ftCallback);
    ros::Subscriber sub_ft_a6 = n.subscribe("FT_a6", 1, ftCallback);

    ros::Rate loop_rate(1);
    
    KDL::Tree my_tree;
    ros::NodeHandle node;
    std::string robot_desc_string;
    node.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
       ROS_ERROR("Failed to construct kdl tree");
       return false;
    }

    KDL::Chain chain;
    if(!my_tree.getChain("base_link", "tool0", chain)){
        ROS_ERROR("Failed to construct kdl chain");
        return false;
    }
    while (ros::ok()){
       ROS_INFO("KDL_KUKA_node working");

       KDL::ChainIdSolver_RNE solver(chain, KDL::Vector(0.0, 0.0, -9.81));
       KDL::JntArray calc_torques(6);

       solver.CartToJnt(q, q_dot, q_dot_dot, Wrenches, calc_torques);

       KDL::JntArray error;
       Subtract(calc_torques, torque, error);
    
       qPRNT(q);

       KDL::ChainJntToJacSolver jac_solver(chain);
       KDL::Jacobian jacobian(chain.getNrOfJoints());
       jac_solver.JntToJac(q, jacobian);

       JACPRNT(jacobian);
       vPRNT(q_dot);
       ros::spinOnce();  
       loop_rate.sleep();
   }

   log_file.close();
   return 0;
}

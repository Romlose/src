#include "kuka_controllers/kuka_kuka_state_controller.h"

namespace kuka_controllers{

    KukaStateController::KukaStateController()
    : joint_names_()
    , js_torques_()
    , id_solver_(nullptr)
    , torques_()
    {}

    KukaStateController::~KukaStateController(){}

    bool KukaStateController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n){
         nh_ = n;

        // get URDF and name of root and tip from the parameter server
        std::string robot_description, root_name, tip_name;

        if (!ros::param::search(n.getNamespace(),"robot_description", robot_description))
        {
            ROS_ERROR_STREAM("InverseDynamicController: No robot description (URDF) found on parameter server ("<<n.getNamespace()<<"/robot_description)");
            return false;
        }

        if (!nh_.getParam("root_name", root_name))
        {
            ROS_ERROR_STREAM("InverseDynamicController: No root name found on parameter server ("<<n.getNamespace()<<"/root_name)");
            return false;
        }

        tip_name = "tool0"; 

        // Construct an URDF model from the xml string
        std::string xml_string;

        if (n.hasParam(robot_description))
            n.getParam(robot_description.c_str(), xml_string);
        else
        {
            ROS_ERROR("Parameter %s not set, shutting down node...", robot_description.c_str());
            n.shutdown();
            return false;
        }

        if (xml_string.size() == 0)
        {
            ROS_ERROR("Unable to load robot model from parameter %s",robot_description.c_str());
            n.shutdown();
            return false;
        }

        ROS_DEBUG("%s content\n%s", robot_description.c_str(), xml_string.c_str());
        
        // Get urdf model out of robot_description
        urdf::Model model;
        if (!model.initString(xml_string))
        {
            ROS_ERROR("Failed to parse urdf file");
            n.shutdown();
            return false;
        }
        ROS_INFO("Successfully parsed urdf file");
        
        if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
        {
            ROS_ERROR("Failed to construct kdl tree");
            n.shutdown();
            return false;
        }

        // Populate the KDL chain
        if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
        {
            ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
            ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
            ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
            ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
            ROS_ERROR_STREAM("  The segments are:");

            KDL::SegmentMap segment_map = kdl_tree_.getSegments();
            KDL::SegmentMap::iterator it;

            for( it=segment_map.begin(); it != segment_map.end(); it++ )
              ROS_ERROR_STREAM( "    "<<(*it).first);

            return false;
        }

        //Get the joint's names

        joint_names_.push_back("joint_a1");
        joint_names_.push_back("joint_a2");
        joint_names_.push_back("joint_a3");
        joint_names_.push_back("joint_a4");
        joint_names_.push_back("joint_a5");
        joint_names_.push_back("joint_a6");

        for(auto it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it){

            if ( it->getJoint().getType() != KDL::Joint::None ){
                joint_handles_.push_back(robot->getHandle(it->getJoint().getName()));
                ROS_DEBUG("%s", it->getJoint().getName().c_str());
            }
        }

        ROS_DEBUG("Number of segments: %d", kdl_chain_.getNrOfSegments());
        ROS_DEBUG("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());

        unsigned int n_dof = kdl_chain_.getNrOfJoints();

        jacobian_.resize(n_dof);
        joint_states_.resize(n_dof);
        torques_.resize(n_dof);
        torques_.data.setZero();
        js_torques_.resize(n_dof);
        Wrenches.resize(kdl_chain_.getNrOfSegments());

        id_solver_.reset( new KDL::ChainIdSolver_RNE( kdl_chain_, KDL::Vector(0.0, 0.0, -9.81)) );
        
        realtime_pub_.reset(new realtime_tools::RealtimePublisher<kuka_controllers::KukaState>(n, "arm_state", 4));

        realtime_pub_->msg_.joint_name.resize(kdl_chain_.getNrOfJoints());
        realtime_pub_->msg_.torque_error.resize(kdl_chain_.getNrOfJoints());

        ros::Subscriber ext_wrenches_sub = nh_.subscribe<geometry_msgs::WrenchStamped>
                                            ("ext_wrenches", 10, &KukaStateController::ext_wrench_cb, this);

        return true;
    }

    void KukaStateController::starting(const ros::Time& time){}

    void KukaStateController::update(const ros::Time &time, const ros::Duration &period){
        
        for(int i = 0; i < 6; i++){
            joint_states_.q(i) = joint_handles_[i].getPosition();
            joint_states_.qdot(i) = joint_handles_[i].getVelocity();
            joint_states_.qdotdot(i) = 0.0;
        }
        /*
        if(realtime_pub_->trylock()){

            if(!id_solver_->
                CartToJnt(joint_states_.q, 
                          joint_states_.qdot, 
                          joint_states_.qdotdot, 
                          Wrenches, 
                          torques_) != 0){
                ROS_ERROR("Could not compute joint torques! Setting all torques to zero!");
                realtime_pub_->unlock();
            }
            realtime_pub_->msg_.header.stamp = time;
            for(unsigned int i = 0; i < 6; i ++ ){
                realtime_pub_->msg_.joint_name[i] = joint_names_[i];
                realtime_pub_->msg_.torque_error[i] = (*js_torques_)(i) - (*torques_)(i);
            }

            realtime_pub_->unlockAndPublish();
        }
        */
    }

    void KukaStateController::stopping(const ros::Time& time){}

    void KukaStateController::ext_wrench_cb(const geometry_msgs::WrenchStamped::ConstPtr &msg){
        KDL::Wrench w(KDL::Vector(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z), 
                  KDL::Vector(msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z));
        Wrenches.push_back(w);
    }

} //kuka_controllers

PLUGINLIB_EXPORT_CLASS(kuka_controllers::KukaStateController, controller_interface::ControllerBase)
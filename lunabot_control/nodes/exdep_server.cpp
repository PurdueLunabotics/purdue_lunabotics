#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <lunabot_msgs/BehaviorAction.h>
#include <lunabot_msgs/LunabotState.h>
#include <lunabot_msgs/Actuation.h>
#include <lunabot_msgs/Drivetrain.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <lunabot_control/states.h>
#include <lunabot_control/utils.h>

#define EXDEP_NAME "exdep_server"

#define EXC_MAX_WEIGHT 64

enum class ExDepGoal {
    EXCAVATE,
    DEPOSIT
};

class ExDepServer
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<lunabot_msgs::BehaviorAction> exdep_as_;
        lunabot_msgs::BehaviorActionFeedback feedback_;
        lunabot_msgs::BehaviorActionGoal goal_;
        lunabot_msgs::LunabotState state_;
        ros::Subscriber robot_state_sub_;

        std_msgs::UInt8 lin_act_msg_;
        std_msgs::UInt8 lead_screw_msg_;
        std_msgs::UInt8 dep_msg_;

        geometry_msgs::Twist cmd_vel_msg_;
        std_msgs::Float32 exc_msg_;

        float exc_weight;
        uint8_t dep_state;
        uint8_t lin_act_state;
        uint8_t lead_screw_state;
        
        ros::Publisher lead_screw_pub_;
        ros::Publisher lin_act_pub_;
        ros::Publisher excavate_pub_;
        ros::Publisher deposit_pub_;
        ros::Publisher cmd_vel_pub_;
    
    private:
        void robot_state_cb(const lunabot_msgs::LunabotState& state) {
            exc_weight = state.exc_state.bin_weight;
            dep_state = state.dep_state;
            lin_act_state = state.lin_act_state;
            lead_screw_state = state.lead_screw_state;
        }

    public:
        ExDepServer() : exdep_as_(nh_,EXDEP_NAME,boost::bind(&ExDepServer::exdepCB,this,_1),false) {
            exdep_as_.start();

            std::string robot_state_topic, exc_topic, lin_act_topic, lead_screw_topic, dep_topic, cmd_vel_topic;
            ros::param::get("robot_state_topic",robot_state_topic);
            ros::param::get("excavation_setp_topic",exc_topic);
            ros::param::get("deposition_setp_topic",dep_topic);
            ros::param::get("lead_screw_setp_topic",lead_screw_topic);
            ros::param::get("lin_act_setp_topic",lin_act_topic);
            ros::param::get("cmd_vel",cmd_vel_topic);

            lin_act_msg_.data = 0; 
            lead_screw_msg_.data = 0; 
            dep_msg_.data = 0; 
            drive_msg_.data = 0; 

            robot_state_sub_ = nh_.subscribe(robot_state_topic, 1, &robot_state_cb, this);
            excavate_pub_ = nh_.advertise<std_msgs::Float32>(exc_topic, 1);
            cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
            lin_act_pub_ = nh_.advertise<std_msgs::UInt8>(lin_act_topic, 1);
            lead_screw_pub_ = nh_.advertise<std_msgs::UInt8>(lead_screw_topic, 1);
            deposit_pub_ = nh_.advertise<std_msgs::UInt8>(dep_topic, 1);
        };

        ~ExDepServer(void) {
            
        }

        void stop_system() {
            cmd_vel_msg_.angular.x = 0;
            cmd_vel_msg_.angular.y = 0;
            cmd_vel_msg_.angular.z = 0;
            cmd_vel_msg_.linear.x = 0;
            cmd_vel_msg_.linear.y = 0;
            cmd_vel_msg_.linear.z = 0;
            exc_msg_.data = 0; 
            lin_act_msg_.data = INT(LinActState::STOPPED);
            lead_screw_msg_.data = INT(LeadScrewState::STOPPED);
            cmd_vel_pub_.publish(cmd_vel_msg_);
            excavate_pub_.publish(exc_msg_);
            lin_act_pub_.publish(lin_act_msg_);
            lead_screw_pub_.publish(lead_screw_msg_);
        }

        bool isCanceled() {
            if (exdep_as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", EXDEP_NAME);
                exdep_as_.setPreempted();
                stop_system();
                return true;
            }  
            return false;
        }


        void exdepCB(const lunabot_msgs::BehaviorActionGoalConstPtr& goal) {
            if(goal->goal.data == INT(ExDepGoal::EXCAVATE)) {
                excavateCB(goal);
            }
            else {
                depositCB(goal); 
            }
        }
        bool depositCB(const lunabot_msgs::BehaviorActionGoalConstPtr& goal) {
            bool success = true;
            lin_act_msg_.data = INT(LinActState::FULL_EXT); 
            lin_act_pub_.publish(lin_act_msg_); 
            while(lin_act_state != INT(LinActState::FULL_EXT)) {
               if(isCanceled()) return false; 
            }
            dep_msg_.data = INT(DepState::FULL_EXT); 
            deposit_pub_.publish(dep_msg_); 
            while(dep_state != INT(DepState::FULL_EXT)) {
                if(isCanceled()) return false; 
            }

            dep_msg_.data = INT(DepState::STORED); 
            deposit_pub_.publish(dep_msg_); 
            while(dep_state != INT(DepState::STORED)) {
                if(isCanceled()) return false; 
            }

            lin_act_msg_.data = INT(LinActState::STORED); 
            lin_act_pub_.publish(lin_act_msg_); 
            while(lin_act_state != INT(LinActState::STORED)) {
               if(isCanceled()) return false; 
            }

        }
    
        bool excavateCB(const lunabot_msgs::BehaviorActionGoalConstPtr& goal) {

            exc_msg_.data = 1.0;
            excavate_pub_.publish(exc_msg_); 
            lin_act_msg_.data = INT(LinActState::FULL_EXT); 
            lin_act_pub_.publish(lin_act_msg_); 
            while(lin_act_state != INT(LinActState::FULL_EXT)) {
               if(isCanceled()) return false; 
            }

            exc_msg_.data = 2.0;
            excavate_pub_.publish(exc_msg_); 
            while(lead_screw_state != INT(LeadScrewState::FULL_EXT)) {
               if(isCanceled()) return false; 
            }

            while(exc_weight <= EXC_MAX_WEIGHT) {
               if(isCanceled()) return false; 
            }

        }

};
#include <ros/ros.h>

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

namespace DummyNodes
{

BT::NodeStatus CheckAprilTag()
{
    std::cout << id << ": April Tag checked" << std::endl;
    //ROS_DEBUG("April Tag checked");
    return BT::NodeStatus::SUCCESS;
};

/*
class GoToPoint : public BT::SyncActionNode
{
    public:
        GoToPoint(const std::string& name) :
            BT::SyncActionNode(name, {})
        {
        }

        BT::NodeStatus tick() override
        {
            ROS_DEBUG("Going to point %s", this->name());
            //std::cout << "Moving to " << this->name() << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
};
*/
//using namespace BT;

}
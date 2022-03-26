#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>

#include <lunabot_behavior/dummy_nodes.h>


static const char* xml_text = R"(
 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <CheckAprilTag    name="check_april_tag"/>
            <CheckAprilTag    name="check_april_tag_2"/>
        </Sequence>
     </BehaviorTree>
 </root>
 )";

// Can probably use existing methods for checking april tags

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_behavior_tree");
    ros::NodeHandle nh;

    BT::BehaviorTreeFactory factory;

    using namespace DummyNodes;

    //factory.registerNodeType<GoToPoint>("GoToPoint");
    factory.registerSimpleCondition("CheckAprilTag", std::bind(CheckAprilTag));

    //factory.registerFromPlugin("./libdummy_nodes_dyn.so");

    auto tree = factory.createTreeFromText(xml_text);

    tree.tickRoot();

    ros::spin();

    return 0;
}
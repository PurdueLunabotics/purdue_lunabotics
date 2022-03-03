/*
Averaging filter on bluetooth beacon rssi stream
Author: Raghava Uppuluri
*/

#include <ros/ros.h>


class FilterNode {
    public:
        FilterNode(ros::NodeHandle* nh) : _nh(nh) {};
    private:
        ros::NodeHandle* _nh;

};


int main(int argc, char** argv) {
    ros::init("rssi_filter"); 



    return 0;
}
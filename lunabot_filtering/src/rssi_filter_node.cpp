/*
Boxcar averaging filter on bluetooth beacon rssi stream
Author: Raghava Uppuluri
*/

#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <lunabot_msgs/BeaconScanArray.h>
#include <rosgraph_msgs/Clock.h>

// rssi_pub_rate/WINDOW_SIZE = filter output rate
// current: 18hz/10 = 1.8hz filter output rate 
#define WINDOW_SIZE 250
#define BEACONS_CNT 3

class RssiFilterNode {
    public:
        RssiFilterNode();
        ~RssiFilterNode() = default;

    private:
        ros::NodeHandle nh_;
        ros::Publisher filtered_pub_;
        ros::Subscriber scan_sub_;
        uint8_t buff_i_;
        int8_t buffer_[BEACONS_CNT][WINDOW_SIZE];
        int sum_[BEACONS_CNT];
        void scan_cb(const lunabot_msgs::BeaconScanArray& msg);
};

RssiFilterNode::RssiFilterNode() : 
    nh_{}, 
    buffer_{ {0} },
    sum_{ {0} },
    buff_i_(0),
    filtered_pub_{nh_.advertise<lunabot_msgs::BeaconScanArray>("/filtered/beacon_scans",15)},
    scan_sub_{nh_.subscribe("beacon_scans",1,&RssiFilterNode::scan_cb,this)}
{}

void RssiFilterNode::scan_cb(const lunabot_msgs::BeaconScanArray& msg) {
    lunabot_msgs::BeaconScanArray filtered_scans; 
    for(int i = 0; i < BEACONS_CNT; i++) {
        sum_[i] -= buffer_[i][buff_i_]; 
        sum_[i] += buffer_[i][buff_i_] = msg.scans[i].rssi;  

        int8_t filtered = sum_[i] / WINDOW_SIZE; 

        lunabot_msgs::BeaconScan scan;
        scan.id = i;
        scan.rssi = filtered;
        scan.stamp = ros::Time::now();
        filtered_scans.scans.push_back(scan);
    }
    buff_i_ = (buff_i_ + 1) % WINDOW_SIZE;
    filtered_pub_.publish(filtered_scans);
}

int main(int argc, char** argv) {
    ros::init(argc,argv,"rssi_filter_node"); 
    RssiFilterNode filter;
    ros::spin();
    return 0;
}
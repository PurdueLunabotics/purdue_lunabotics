#include <ros/ros.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <cerrno>
#include <array>
#include <iomanip>
#include <vector>
#include <boost/optional.hpp>

#include <signal.h>

#include <stdexcept>

#include <blepp/logging.h>
#include <blepp/pretty_printers.h>
#include <blepp/blestatemachine.h>
#include <blepp/lescan.h>

#include <lunabot_msgs/BeaconScan.h>
#include <lunabot_msgs/BeaconScanArray.h>

using namespace std;
using namespace BLEPP;

#define MIN_SCAN_CNT 3

void catch_function(int)
{
	cerr << "\nInterrupted!\n";
  ros::shutdown();
}

enum beac_id {
	BEAC_ID1=1,
	BEAC_ID2=2,
	BEAC_ID3=3
};

class BeaconScannerNode {
	private:
		ros::NodeHandle* _nh;
    ros::Publisher scans_publisher_;
    std::string _addr1, _addr2, _addr3;
		void process_scan(int beac_id, int8_t rssi, lunabot_msgs::BeaconScan* scan);
	public:
		BeaconScannerNode(ros::NodeHandle* nh);
		void scan();
};

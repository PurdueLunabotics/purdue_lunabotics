#include <lunabot_bt/scan.h>

BeaconScannerNode::BeaconScannerNode(ros::NodeHandle* nh) : _nh(nh) {
	signal(SIGINT, catch_function);

	nh->getParam("/lunabot_bt/beac_addr1", _addr1);
	nh->getParam("/lunabot_bt/beac_addr2", _addr2);
	nh->getParam("/lunabot_bt/beac_addr3", _addr3);

	scans_publisher_ = nh->advertise<lunabot_msgs::BeaconScanArray>("/beacon_scans", 10);
}

void BeaconScannerNode::process_scan(int beac_id, int8_t rssi, lunabot_msgs::BeaconScan* scan) {
	if(rssi == 127) {
		ROS_DEBUG("Beacon #%d Unavailable",beac_id);
	}
	else if(rssi > 20) {
		ROS_DEBUG("  RSSI = %s unknown",to_hex((uint8_t)rssi).c_str());
	}
	else {
		ROS_ERROR("processing");
		scan->id = beac_id;
		scan->rssi = rssi;
		scan->stamp = ros::Time::now();
	}
}

void BeaconScannerNode::scan() {
	HCIScanner::ScanType type = HCIScanner::ScanType::Active;
	HCIScanner::FilterDuplicates filter = HCIScanner::FilterDuplicates::Off;

	log_level = LogLevels::Warning;
	HCIScanner scanner(true, filter, type);

  ros::Rate loop(100);
	lunabot_msgs::BeaconScanArray beacon_scans; 

  beacon_scans.scans.resize(MIN_SCAN_CNT);
  uint8_t is_set = 0b000;
  uint8_t mask = 0b111;

	while (ros::ok()) {
    //Check to see if there's anything to read from the HCI
    //and wait if there's not.
    struct timeval timeout;     
    timeout.tv_sec = 0;     
    timeout.tv_usec = 50000;

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(scanner.get_fd(), &fds);
    int err = select(scanner.get_fd()+1, &fds, NULL, NULL,  &timeout);
    
    //Interrupted, so quit and clean up properly.
    if(err < 0 && errno == EINTR)	{
      return;
    }

    if(err < 0) {	
      cout << "error" << "\n";
      return;
    } 

    if(FD_ISSET(scanner.get_fd(), &fds))
    {
      // Only read id there's something to read
      vector<AdvertisingResponse> ads = scanner.get_advertisements();

      for(const auto& ad: ads)
      {
          
          lunabot_msgs::BeaconScan scan;
          scan.id = 0; // invalid scan
          if(ad.address == _addr1) {
            cout << "scan1" << "\n";
            process_scan(BEAC_ID1, ad.rssi, &scan);
          }
          else if(ad.address == _addr2) {
            cout << "scan2" << "\n";
            process_scan(BEAC_ID2, ad.rssi, &scan);
          }
          else if(ad.address == _addr3) {
            cout << "scan3" << "\n";
            process_scan(BEAC_ID3, ad.rssi, &scan);
          }
          if(scan.id != 0) {
            cout << "setting" << "\n";
            beacon_scans.scans[scan.id - 1] = scan;
            is_set |= 1 << (scan.id - 1);
          }
      }

      if(is_set & mask == mask) {
        cout << "publishing" << "\n";
        
		    beacon_scans.stamp = ros::Time::now();
        scans_publisher_.publish(beacon_scans);
        //is_set = 0b000;
      }
    }
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_node");
  ros::NodeHandle nh;
	BeaconScannerNode scanner(&nh);
  scanner.scan();
  return 0;
}

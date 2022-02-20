#include <lunabot_bt/scan.h>

BeaconScannerNode::BeaconScannerNode(ros::NodeHandle* nh) : _nh(nh) {
	HCIScanner scanner(true, filter, type);

	signal(SIGINT, catch_function);

	int UUID1, UUID2, UUID3;
	nh->getParam("~beac_UUID1", UUID1);
	nh->getParam("~beac_UUID2", UUID2);
	nh->getParam("~beac_UUID3", UUID3);

	//Turn provided UUID chars into UUIDs
	UUID1, UUID2, UUID3;
	bt_uuid16_create(&this->_UUID1, (uint16_t) UUID1);
	bt_uuid16_create(&this->_UUID2, (uint16_t) UUID2);
	bt_uuid16_create(&this->_UUID3, (uint16_t) UUID3);

	scans_publisher_ = nh->advertise<lunabot_msgs::BeaconScanArray>("/beacon_scans", 10);
}

void BeaconScannerNode::process_scan(int beac_id, int8_t rssi, int16_t uuid, lunabot_msgs::BeaconScan* scan) {
	if(rssi == 127) {
		ROS_DEBUG("Beacon #%d Unavailable",beac_id);
	}
	else if(rssi > 20) {
		ROS_DEBUG("  RSSI = %s unknown",to_hex((uint8_t)rssi));
	}
	else {
		scan->id = beac_id;
		scan->rssi = rssi;
		scan->stamp = ros::Time::now();
		scan->uuid = uuid;
	}
}

void BeaconScannerNode::scan() {
	//Check to see if there's anything to read from the HCI
	//and wait if there's not.
	struct timeval timeout;     
	timeout.tv_sec = 0;     
	timeout.tv_usec = 300000;

	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(scanner.get_fd(), &fds);
	int err = select(scanner.get_fd()+1, &fds, NULL, NULL,  &timeout);
	
	//Interrupted, so quit and clean up properly.
	if(err < 0 && errno == EINTR)	
		return;
	
	if(FD_ISSET(scanner.get_fd(), &fds))
	{
		// Only read id there's something to read
		vector<AdvertisingResponse> ads = scanner.get_advertisements();
		lunabot_msgs::BeaconScanArray beacon_scans; 

		for(const auto& ad: ads)
		{
			for(BLEPP::UUID uuid: ad.UUIDs) {
				lunabot_msgs::BeaconScan scan;
				scan.id = -1; // invalid scan
				if(uuid == UUID::from(_UUID1)) {
					process_scan(BEAC_ID1, ad.rssi, uuid.value.u16, &scan);
				}
				else if(uuid == UUID::from(_UUID2)) {
					process_scan(BEAC_ID2, ad.rssi, uuid.value.u16, &scan);
				}
				else if(uuid == UUID::from(_UUID3)) {
					process_scan(BEAC_ID3, ad.rssi, uuid.value.u16, &scan);
				}
				if(scan.id != -1) beacon_scans.scans.push_back(scan);
			}
		}
		if(beacon_scans.scans.size() >= MIN_SCAN_CNT) {
			scans_publisher_.publish(beacon_scans);
		}
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_node");
    ros::NodeHandle nh;

	BeaconScannerNode scanner(&nh);

    ros::AsyncSpinner spinner(4);
    spinner.start();
  
  	ros::Rate loop(10);

	while (ros::ok()) {
		scanner.scan();
	    loop.sleep();
	}

  return 0;
}

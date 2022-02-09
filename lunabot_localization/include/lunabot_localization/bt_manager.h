#pragma once

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
#include <blepp/uuid.h>

using namespace std;
using namespace BLEPP;

struct btevent //Stores information when a given UUID is detected
{
    unsigned char r_id = 0;
    bt_uuid_t UUID;
    chrono::time_point<std::chrono::system_clock> t; //Time of event
    int8_t rssi; //RSSI of signal
};

class BTManager {
  public:
    BTManager(uint128_t UUID1_, uint128_t UUID2_, uint128_t UUID3_);
    std::vector<btevent> getResults(); //Run this when you want to get scan results
    
  private:
    BLEPP::bt_uuid_t UUID1;
    BLEPP::bt_uuid_t UUID2;
    BLEPP::bt_uuid_t UUID3;
    BLEPP::HCIScanner * scanner;

};

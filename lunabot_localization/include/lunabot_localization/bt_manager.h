#pragma once

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

#include <blepp/logging.h>
#include <blepp/pretty_printers.h>
#include <blepp/blestatemachine.h>
#include <blepp/lescan.h>
#include <blepp/uuid.h>

struct btevent //Stores information when a given UUID is detected
{
    unsigned char r_id = 0;
    BLEPP::bt_uuid_t UUID;
    std::chrono::time_point<std::chrono::system_clock> t; //Time of event
    int8_t rssi; //RSSI of signal
};

class BTManager {
  public:
    BTManager();
    std::vector<btevent> getResults(); //Run this when you want to get scan results
    
  private:
    BLEPP::bt_uuid_t UUID1;
    BLEPP::bt_uuid_t UUID2;
    BLEPP::bt_uuid_t UUID3;
    BLEPP::HCIScanner * scanner;

};

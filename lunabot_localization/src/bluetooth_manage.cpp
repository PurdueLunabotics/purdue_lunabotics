#include <string.h>
#include <vector>
#include <chrono>
#include <ctime>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

#include <blepp/logging.h>
#include <blepp/pretty_printers.h>
#include <blepp/blestatemachine.h>
#include <blepp/lescan.h>
#include <blepp/uuid.h>

using namespace BLEPP;

//UUIDs of BLE beacons
const BLEPP::bt_uuid_t UUID1;
const BLEPP::bt_uuid_t UUID2;
const BLEPP::bt_uuid_t UUID3;

struct btevent //Stores information when a given UUID is detected
{
    unsigned char r_id = 0;
    bt_uuid_t UUID;
    std::chrono::time_point<std::chrono::system_clock> t; //Time of event
    int8_t rssi; //RSSI of signal
};

HCIScanner* scanSetup() 
{
    //Scan setup
    HCIScanner::ScanType type = HCIScanner::ScanType::Active;
    HCIScanner::FilterDuplicates filter = HCIScanner::FilterDuplicates::Off; //Needs testing
    HCIScanner scanner(true, filter, type);
    //Probably this should send errors and results somewhere (this will be part of ROS wrapper)
    return &scanner;
}

std::vector<btevent> getResults(HCIScanner scanner) //Run this when you want to get scan results
{
    std::vector<AdvertisingResponse> responses = scanner.get_advertisements();
    std::vector<btevent> events = std::vector<btevent>();
    bool w1, w2, w3 = false;
    AdvertisingResponse test_event;
    while (!(w1 & w2 & w3)) 
    {
        test_event = responses.back(); responses.pop_back();
        if (!w1 && test_event.UUIDs.at(2) == UUID::from(UUID1)) 
        {
            btevent event;
            event.t = std::chrono::system_clock::now();
            event.UUID = UUID1;
            event.r_id = 1;
            event.rssi = test_event.rssi;
            events.push_back(event);
        }
        else if (!w2 && test_event.UUIDs.at(2) == UUID::from(UUID2)) 
        {
            btevent event;
            event.t = std::chrono::system_clock::now();
            event.UUID = UUID2;
            event.r_id = 2;
            event.rssi = test_event.rssi;
            events.push_back(event);
        }
        else if (!w3 && test_event.UUIDs.at(2) == UUID::from(UUID3)) 
        {
            btevent event;
            event.t = std::chrono::system_clock::now();
            event.UUID = UUID3;
            event.r_id = 3;
            event.rssi = test_event.rssi;
            events.push_back(event);
        }
    }
    return events;
}
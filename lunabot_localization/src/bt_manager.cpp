#include <lunabot_localization/bt_manager.h>

using namespace BLEPP;

BTManager::BTManager() 
{
  //Scan setup
  HCIScanner::ScanType type = HCIScanner::ScanType::Active;
  HCIScanner::FilterDuplicates filter = HCIScanner::FilterDuplicates::Off; //Needs testing
  HCIScanner scanner(true, filter, type);
  //Probably this should send errors and results somewhere (this will be part of ROS wrapper)
  this->scanner = &scanner;
}

std::vector<btevent> BTManager::getResults() //Run this when you want to get scan results
{
    std::vector<AdvertisingResponse> responses = this->scanner->get_advertisements();
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

#include <lunabot_localization/bt_manager.h>

using namespace BLEPP;

BTManager::BTManager(uint128_t UUID1_, uint128_t UUID2_, uint128_t UUID3_) 
{
  //Turn provided UUID chars into UUIDs
  bt_uuid128_create(&this->UUID1, UUID1_);
  bt_uuid128_create(&this->UUID2, UUID2_);
  bt_uuid128_create(&this->UUID3, UUID3_);
  //Scan setup
  HCIScanner::ScanType type = HCIScanner::ScanType::Active;
  HCIScanner::FilterDuplicates filter = HCIScanner::FilterDuplicates::Off; //Needs testing
  HCIScanner scanner(true, filter, type);
  //Boilerplate code
  log_level = LogLevels::Warning;
  HCIScanner scanner(true, filter, type);
  //Catch the interrupt signal. If the scanner is not
  //cleaned up properly, then it doesn't reset the HCI state.
  signal(SIGINT, catch_function)

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

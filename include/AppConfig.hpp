#ifndef TRAFFIC_MONITOR_RUN_H
#define TRAFFIC_MONITOR_RUN_H

#include "Tracker.hpp"

class AppConfig {
 public:
  void setup(Tracker &tracker, std::string video_path = "data/CarsDrivingUnderBridge.mp4");
};
#endif //TRAFFIC_MONITOR_RUN_H

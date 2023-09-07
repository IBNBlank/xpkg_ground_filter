/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2023-09-07
 ****************************************************************/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <functional>

#include "ground_filter/data_interface.h"
#include "ground_filter/ground_filter.h"

using hex::perception::DataInterface;
using hex::perception::GroundFilter;
using hex::perception::LogLevel;

namespace ground_filter_plugin {

void TimeCallback() {
  enum class FiniteState { kInitState = 0, kWorkState };
  static FiniteState finite_state_machine_state = FiniteState::kInitState;
  static GroundFilter& ground_filter = GroundFilter::GetGroundFilter();
  static DataInterface& data_interface = DataInterface::GetDataInterface();

  switch (finite_state_machine_state) {
    case FiniteState::kInitState: {
      if (ground_filter.Init()) {
        data_interface.Log(LogLevel::kInfo, "%s : Init Succeded",
                           "ground_filter");
        finite_state_machine_state = FiniteState::kWorkState;
      } else {
        data_interface.Log(LogLevel::kWarn, "%s : Init Failed",
                           "ground_filter");
        finite_state_machine_state = FiniteState::kInitState;
      }
      break;
    }
    case FiniteState::kWorkState: {
      if (ground_filter.Work()) {
        // data_interface.Log(LogLevel::kInfo, "%s : Work Succeded",
        //                    "ground_filter");
        finite_state_machine_state = FiniteState::kWorkState;
      } else {
        data_interface.Log(LogLevel::kWarn, "%s : Work Failed",
                           "ground_filter");
        finite_state_machine_state = FiniteState::kInitState;
      }
      break;
    }
    default: {
      data_interface.Log(LogLevel::kError, "%s : Unknown State",
                         "ground_filter");
      finite_state_machine_state = FiniteState::kInitState;
      break;
    }
  }
}

class GroundFilterPlugin : public nodelet::Nodelet {
 public:
  GroundFilterPlugin() = default;
  virtual ~GroundFilterPlugin() = default;

 private:
  virtual void onInit();
};

void GroundFilterPlugin::onInit() {
  DataInterface& data_interface = DataInterface::GetDataInterface();
  data_interface.Init(&getPrivateNodeHandle(), "ground_filter", 50.0,
                      TimeCallback);
  data_interface.Work();
  data_interface.Deinit();
}

PLUGINLIB_EXPORT_CLASS(ground_filter_plugin::GroundFilterPlugin,
                       nodelet::Nodelet);

}  // namespace ground_filter_plugin

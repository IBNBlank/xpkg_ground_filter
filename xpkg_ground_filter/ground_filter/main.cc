/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2023-08-29
 ****************************************************************/

#include <ros/ros.h>

#include "ground_filter/data_interface.h"
#include "ground_filter/ground_filter.h"

using hex::perception::DataInterface;
using hex::perception::GroundFilter;
using hex::perception::LogLevel;

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

int main(int argc, char* argv[]) {
  DataInterface& data_interface = DataInterface::GetDataInterface();
  data_interface.Init(argc, argv, "ground_filter", 50.0, TimeCallback);

  data_interface.Work();

  data_interface.Deinit();

  return 0;
}

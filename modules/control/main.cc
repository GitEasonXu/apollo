/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/common/apollo_app.h"
#include "modules/control/common/control_gflags.h"
#include "modules/control/control.h"

APOLLO_MAIN(apollo::control::Control);  //control主函数 BUILD文件定义生成control二进制文件

///
#define APOLLO_MAIN(APP)                                       \
  int main(int argc, char **argv) {                            \
    google::InitGoogleLogging(argv[0]);                        \ //Google log初始化
    google::ParseCommandLineFlags(&argc, &argv, true);         \ //gflags参数解析
    signal(SIGINT, apollo::common::apollo_app_sigint_handler); \ //中断检测，如果检测到SIGINT信号ros::shutdown();
    APP apollo_app_;                                           \ //创建apollo::control::Control实例 APP
    ros::init(argc, argv, apollo_app_.Name());                 \ //初始化节点 节点名称为："control" 在control_gflags.cc中定义
    apollo_app_.Spin();                                        \ //自循环 下面详细介绍
    return 0;                                                  \
  }

#endif  // MODULES_COMMON_APOLLO_APP_H_
///


///
int ApolloApp::Spin() {
  auto status = Init();                            //初始化 由Status Control::Init()实现
  if (!status.ok()) {
    AERROR << Name() << " Init failed: " << status;
    return -1;
  }

  std::unique_ptr<ros::AsyncSpinner> spinner;
  if (callback_thread_num_ > 1) {
    spinner = std::unique_ptr<ros::AsyncSpinner>(
        new ros::AsyncSpinner(callback_thread_num_));  //开启callback_thread_num_个数个线程
  }
 
  status = Start();                                    //启动 由Status Control::Start()实现
  if (!status.ok()) {
    AERROR << Name() << " Start failed: " << status;
    return -2;
  }
  ExportFlags();                                       //输出Flag参数信息
  if (spinner) {
    spinner->start();                                  //开启节点消息线程
  } else {
    ros::spin();
  }
  ros::waitForShutdown();                              //循环检测是否关闭
  Stop();                                              //关闭的话停止Control模块运行
  AINFO << Name() << " exited.";
  return 0;
}
///

///
Status Control::Init() {
  init_time_ = Clock::NowInSeconds();

  AINFO << "Control init, starting ...";
  CHECK(common::util::GetProtoFromFile(FLAGS_control_conf_file, &control_conf_)) //检测并加载control_conf_file: modules/control/conf/lincoln.pb.txt
      << "Unable to load control conf file: " + FLAGS_control_conf_file;         //lincoln.pb.txt 定义了
										 //control_period: 0.01   控制时间间隔
										 //trajectory_period: 0.1 轨迹点时间间隔
										 //chassis_period: 0.01
										 //localization_period: 0.01 获取定位信息时间间隔

  AINFO << "Conf file: " << FLAGS_control_conf_file << " is loaded.";            //以及还有一些车辆在高低度情况下 PID控制参数

  AdapterManager::Init(FLAGS_control_adapter_config_filename);   //"modules/control/conf/adapter.conf" adapter.conf文件配置了ROS topic的收发

  common::monitor::MonitorLogBuffer buffer(&monitor_logger_);    //定义Monitor日志缓存

  // set controller
  if (!controller_agent_.Init(&control_conf_).ok()) {
    std::string error_msg = "Control init controller failed! Stopping...";
    buffer.ERROR(error_msg);
    return Status(ErrorCode::CONTROL_INIT_ERROR, error_msg);
  }

  // lock it in case for after sub, init_vehicle not ready, but msg trigger
  // come
  CHECK(AdapterManager::GetLocalization())
      << "Localization is not initialized.";

  CHECK(AdapterManager::GetChassis()) << "Chassis is not initialized.";

  CHECK(AdapterManager::GetPlanning()) << "Planning is not initialized.";

  CHECK(AdapterManager::GetPad()) << "Pad is not initialized.";

  CHECK(AdapterManager::GetMonitor()) << "Monitor is not initialized.";

  CHECK(AdapterManager::GetControlCommand())
      << "ControlCommand publisher is not initialized.";

  AdapterManager::AddPadCallback(&Control::OnPad, this);
  AdapterManager::AddMonitorCallback(&Control::OnMonitor, this);

  return Status::OK();
}
///


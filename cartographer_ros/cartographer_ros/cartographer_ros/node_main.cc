/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"
#include "cartographer_ros_msgs/ChangeMode.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "cartographer_ros/node_constants.h"
#include "absl/synchronization/mutex.h"
#include "absl/strings/str_cat.h"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");
bool pure_localization_ = true;

//absl::Mutex mutex_; 
namespace cartographer_ros {
namespace {

int iMapping();
int iLocalization();

int iMapping() {
  std::string dir;
  dir = absl::StrCat(absl::StrCat("/home/", getlogin()), "/Downloads");
  ROS_INFO("in iMapping %s", getlogin());
  std::string filename = "";
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =
      LoadOptions(absl::StrCat(dir, "/configuration_files"), "revo_lds.lua");
  auto map_builder =
      cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  Node node(node_options, std::move(map_builder), &tf_buffer,
            FLAGS_collect_metrics);
  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }
  if (FLAGS_start_trajectory_with_default_topics) {
    node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  while(!pure_localization_)
  {
      ::ros::spinOnce();
  }
  node.FinishAllTrajectories();
  node.RunFinalOptimization();

  //保存成pbstream
  filename = absl::StrCat(dir, "/Default.pbstream");
  node.SerializeState(filename,false);
  //保存yaml、pgm
  node.save(filename, absl::StrCat(dir, "/Default"), 0.05);

  node.~Node();
  cartographer_ros::iLocalization();

  return 0;
}

int iLocalization() {
  ROS_INFO("in iLocalization");
  std::string dir;
  dir = absl::StrCat(absl::StrCat("/home/", getlogin()), "/Downloads");
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;

  //加载定位option
  std::tie(node_options, trajectory_options) = LoadOptions(
    absl::StrCat(dir, "/configuration_files"), "backpack_2d_localization.lua");
    LOG(INFO)<<trajectory_options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data();
  auto map_builder =
      cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  Node node(node_options, std::move(map_builder), &tf_buffer,
            FLAGS_collect_metrics);
  //加载pbstream
  node.LoadState(absl::StrCat(dir, "/Default.pbstream"), FLAGS_load_frozen_state);
  
  if (FLAGS_start_trajectory_with_default_topics) {
    node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  while(pure_localization_)
  {
      ::ros::spinOnce();
  }

  node.~Node();
  cartographer_ros::iMapping();
  return 0;
}

bool changeMode(cartographer_ros_msgs::ChangeMode::Request& request,
            cartographer_ros_msgs::ChangeMode::Response& response){
      pure_localization_ = request.pure_localization;
      response.status.code = cartographer_ros_msgs::StatusCode::OK;
      if(pure_localization_){
        response.status.message = "pure_localization_mode.";
      }
      else if(!pure_localization_){
        response.status.message = "mapping_mode.";
      }
      else{
        response.status.message = "change is wrong.";
        return false;
      }
      return true;
}


}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
    ::ros::init(argc, argv, "cartographer_node");
    ::ros::start();
//////////////切换服务//////////////////////////////
::ros::NodeHandle node_handle_;
::ros::ServiceServer mode_change_servers_;
//sleep(2);
mode_change_servers_ = node_handle_.advertiseService(
      "ChangeMode", &cartographer_ros::changeMode);
//////////////////////////////////////////////////

    FLAGS_logtostderr = false;
    FLAGS_alsologtostderr = false;
    FLAGS_log_prefix = true;
    FLAGS_log_dir = "./log";

  cartographer_ros::ScopedRosLogSink ros_log_sink;

  //cartographer_ros::iMapping();
  cartographer_ros::iLocalization();

  ::ros::shutdown();
}

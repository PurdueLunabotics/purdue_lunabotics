/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <yaml-cpp/yaml.h>
#define PC_SIZE 2
class PointsConcatFilter {
public:
  PointsConcatFilter();

private:
  typedef pcl::PointXYZ PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef sensor_msgs::PointCloud2 PointCloudMsgT;
  typedef message_filters::sync_policies::ApproximateTime<PointCloudMsgT, PointCloudMsgT>
      SyncPolicyT;

  ros::NodeHandle nh_, pnh_;
  message_filters::Subscriber<PointCloudMsgT> *cloud_subscribers_[PC_SIZE];
  message_filters::Synchronizer<SyncPolicyT> *cloud_synchronizer_;
  ros::Subscriber config_subscriber_;
  ros::Publisher cloud_publisher_;
  tf::TransformListener tf_listener_;

  std::vector<std::string> input_topics_;
  std::string output_frame_id_;
  std::string output_topic_;

  void pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1,
                           const PointCloudMsgT::ConstPtr &msg2);
};

PointsConcatFilter::PointsConcatFilter() : nh_(), pnh_("~"), tf_listener_() {
  ros::param::get("~input_topics", input_topics_);
  ros::param::get("~output_frame_id", output_frame_id_);
  ros::param::get("~output_topic", output_topic_);

  if (input_topics_.size() != PC_SIZE) {
    ROS_ERROR("The size of input_topics must be between 2");
    ros::shutdown();
  }
  for (size_t i = 0; i < PC_SIZE; ++i) {
    if (i < input_topics_.size()) {
      cloud_subscribers_[i] =
          new message_filters::Subscriber<PointCloudMsgT>(nh_, input_topics_[i], 1);
    } else {
      cloud_subscribers_[i] =
          new message_filters::Subscriber<PointCloudMsgT>(nh_, input_topics_[0], 1);
    }
  }
  cloud_synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(
      SyncPolicyT(10), *cloud_subscribers_[0], *cloud_subscribers_[1]);

  cloud_synchronizer_->registerCallback(
      boost::bind(&PointsConcatFilter::pointcloud_callback, this, _1, _2));
  cloud_publisher_ = nh_.advertise<PointCloudMsgT>(output_topic_, 1);
}

void PointsConcatFilter::pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1,
                                             const PointCloudMsgT::ConstPtr &msg2) {
  PointCloudMsgT::ConstPtr msgs[2] = {msg1, msg2};
  PointCloudT::Ptr cloud_sources[2];
  PointCloudT::Ptr cloud_concatenated(new PointCloudT);

  // transform points
  try {
    for (size_t i = 0; i < input_topics_.size(); ++i) {
      // Note: If you use kinetic, you can directly receive messages as
      // PointCloutT.
      cloud_sources[i] = PointCloudT().makeShared();
      pcl::fromROSMsg(*msgs[i], *cloud_sources[i]);
      tf_listener_.waitForTransform(output_frame_id_, msgs[i]->header.frame_id, ros::Time(0),
                                    ros::Duration(1.0));
      pcl_ros::transformPointCloud(output_frame_id_, ros::Time(0), *cloud_sources[i],
                                   msgs[i]->header.frame_id, *cloud_sources[i], tf_listener_);
    }
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // merge points
  for (size_t i = 0; i < input_topics_.size(); ++i) {
    *cloud_concatenated += *cloud_sources[i];
  }

  // publsh points
  cloud_concatenated->header = pcl_conversions::toPCL(msgs[0]->header);
  cloud_concatenated->header.frame_id = output_frame_id_;
  cloud_publisher_.publish(cloud_concatenated);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "points_concat");
  PointsConcatFilter node;
  ros::spin();
  return 0;
}
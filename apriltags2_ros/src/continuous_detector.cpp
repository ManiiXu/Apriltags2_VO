/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include "apriltags2_ros/continuous_detector.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(apriltags2_ros::ContinuousDetector, nodelet::Nodelet);

namespace apriltags2_ros
{

ContinuousDetector::ContinuousDetector ()
{
}

void ContinuousDetector::onInit ()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& pnh = getPrivateNodeHandle();

  tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(pnh));
  draw_tag_detections_image_ = getAprilTagOption<bool>(pnh, 
      "publish_tag_detections_image", false);

  it_ = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(nh));

  camera_image_subscriber_ =
      it_->subscribeCamera("image_rect", 1,
                          &ContinuousDetector::imageCallback, this);
  tag_detections_publisher_ =
      nh.advertise<AprilTagDetectionArray>("tag_detections", 1);

  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
  }

  odomtry_publisher_ = nh.advertise<nav_msgs::Odometry>("tag_Odometry", 1);
  path_pubilsher = nh.advertise<nav_msgs::Path>("path", 1);

}

void ContinuousDetector::imageCallback (
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTags 2 on the iamge
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image_rect,
                                    sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Publish detected tags in the image by AprilTags 2
  AprilTagDetectionArray tag_detection_array(tag_detector_->detectTags(cv_image_,camera_info));
  tag_detections_publisher_.publish(tag_detection_array);

  if (true) {
    for (unsigned int i = 0; i < tag_detection_array.detections.size(); i++) {
      nav_msgs::Odometry odometry;
      odometry.header = tag_detection_array.detections[i].pose.header;
      //odometry.header.frame_id = "my_bundle";
      odometry.pose.pose = tag_detection_array.detections[i].pose.pose.pose;
      odomtry_publisher_.publish(odometry);

      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header = odometry.header;
      //pose_stamped.header.frame_id = "world";
      pose_stamped.pose.position.x = tag_detection_array.detections[i].pose.pose.pose.position.x;
      pose_stamped.pose.position.y = tag_detection_array.detections[i].pose.pose.pose.position.y;
      pose_stamped.pose.position.z = tag_detection_array.detections[i].pose.pose.pose.position.z;
      camera_path.header = pose_stamped.header;
      //camera_path.header.frame_id = "world";
      camera_path.poses.push_back(pose_stamped);
      path_pubilsher.publish(camera_path);

    }
  }

  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_->drawDetections(cv_image_);
    tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
  }
}

} // namespace apriltags2_ros

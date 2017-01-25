/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#include "convert.h"

#include <pcl_conversions/pcl_conversions.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne_rawdata::RawData())
  {
    data_->setup(private_nh);

    // use private node handle to get parameters
    private_nh.param("dual_return_mode", dual_return_mode_, false);

    // advertise output point cloud (before subscribing to input data)
    if(!dual_return_mode_) {
      output_ = node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 1);
    } else {
      output_strong = node.advertise<sensor_msgs::PointCloud2>("velodyne_points/strong", 1);
      output_last = node.advertise<sensor_msgs::PointCloud2>("velodyne_points/last", 1);
    }

      
    srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_pointcloud::
      CloudNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig>::
      CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);

    // subscribe to VelodyneScan packets
    velodyne_scan_ =
      node.subscribe("velodyne_packets", 1,
                     &Convert::processScan, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));
  }
  
  void Convert::callback(velodyne_pointcloud::CloudNodeConfig &config,
                uint32_t level)
  {
  ROS_INFO("Reconfigure Request");
  data_->setParameters(config.min_range, config.max_range, config.view_direction,
                       config.view_width);
  }

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if(!dual_return_mode_) {
      if (output_.getNumSubscribers() == 0)         // no one listening?
        return;                                     // avoid much work

      // allocate a point cloud with same time and frame ID as raw data
      velodyne_rawdata::VPointCloud::Ptr
          outMsg(new velodyne_rawdata::VPointCloud());
      // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
      outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
      outMsg->header.frame_id = scanMsg->header.frame_id;
      outMsg->height = 1;

      // process each packet provided by the driver
      for (size_t i = 0; i < scanMsg->packets.size(); ++i)
      {

        data_->unpack(scanMsg->packets[i], *outMsg);
      }

      // publish the accumulated cloud message
      ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
                                     << " Velodyne points, time: " << outMsg->header.stamp);
      output_.publish(outMsg);
    } else {
      // Dual return mode - make two pointclouds
      if (output_strong.getNumSubscribers() == 0 && output_last.getNumSubscribers() == 0)         // no one listening?
        return;                                     // avoid much work

      // allocate a point cloud with same time and frame ID as raw data
      velodyne_rawdata::VPointCloud::Ptr outMsgStrong(new velodyne_rawdata::VPointCloud());
      // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
      outMsgStrong->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
      outMsgStrong->header.frame_id = scanMsg->header.frame_id;
      outMsgStrong->height = 1;
      velodyne_rawdata::VPointCloud::Ptr outMsgLast(new velodyne_rawdata::VPointCloud());
      outMsgLast->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
      outMsgLast->header.frame_id = scanMsg->header.frame_id;
      outMsgLast->height = 1;

      // process each packet provided by the driver
      for (size_t i = 0; i < scanMsg->packets.size(); ++i)
      {

        data_->unpack(scanMsg->packets[i], *outMsgStrong, *outMsgLast);
      }

      // publish the accumulated cloud message
      ROS_DEBUG_STREAM("Publishing Dual Return Clouds. Strong: " << outMsgStrong->height * outMsgStrong->width
                                                                 << ", Last: " << outMsgLast->height * outMsgLast->width
                                                                 << " time: " << outMsgLast->header.stamp);
      output_strong.publish(outMsgStrong);
      output_last.publish(outMsgLast);
    }

  }

} // namespace velodyne_pointcloud

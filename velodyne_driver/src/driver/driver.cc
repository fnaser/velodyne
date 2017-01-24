/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the Velodyne 3D LIDARs
 */

#include <string>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <velodyne_msgs/VelodyneScan.h>

#include "driver.h"
#include <angles/angles.h>

namespace velodyne_driver
{
  VelodyneDriver::VelodyneDriver(ros::NodeHandle node,
                                 ros::NodeHandle private_nh)
  {
    // use private node handle to get parameters
    float packet_rate;
    std::string dump_file;
    int udp_port;
    std::string devip;
    private_nh.param("frame_id", config_.frame_id, std::string("velodyne"));
    private_nh.param("packet_rate", packet_rate, 3472.0f);  // Only used with pcap file to mimic the original rate
    private_nh.param("model", config_.model, std::string("64E"));
    private_nh.param("rpm", config_.rpm, 600.0);
    private_nh.param("pcap", dump_file, std::string(""));
    private_nh.param("port", udp_port, (int)UDP_PORT_NUMBER);
    private_nh.param("device_ip", devip, std::string(""));
    private_nh.param("cutoff_angle", config_.cutoff_angle, M_PI);
    std::string tf_prefix = tf::getPrefixParam(private_nh);
    ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
    config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

    ROS_INFO_STREAM(config_.model << " rotating at " << config_.rpm << " RPM");
    double frequency = (config_.rpm / 60.0);     // expected Hz rate

    // Initialize dynamic reconfigure
    srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_driver::
    VelodyneNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne_driver::VelodyneNodeConfig>::
    CallbackType f;
    f = boost::bind (&VelodyneDriver::callback, this, _1, _2);
    srv_->setCallback (f); // Set callback function und call initially

    // initialize diagnostics
    diagnostics_.setHardwareID(config_.model);
    const double diag_freq = frequency;
    diag_max_freq_ = diag_freq;
    diag_min_freq_ = diag_freq;
    ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

    using namespace diagnostic_updater;
    diag_topic_.reset(new TopicDiagnostic("velodyne_packets", diagnostics_,
                                          FrequencyStatusParam(&diag_min_freq_,
                                                               &diag_max_freq_,
                                                               0.1, 10),
                                          TimeStampStatusParam()));

    // open Velodyne input device or file
    if (dump_file != "")
    {
      input_.reset(new velodyne_driver::InputPCAP(private_nh,
                                                  packet_rate,
                                                  dump_file));
    }
    else
    {
      input_.reset(new velodyne_driver::InputSocket(private_nh, udp_port));
    }

    // Setup device_ip
    if(!devip.empty())
      ROS_INFO_STREAM("Set device ip to " << devip << ", only accepting packets from this address." );
    input_->setDeviceIP(devip);

    // raw data output topic
    output_ = node.advertise<velodyne_msgs::VelodyneScan>("velodyne_packets", 10);
  }

/** poll the device
 *
 *  @returns true unless end of file reached
 */
  bool VelodyneDriver::poll(void)
  {
    // Allocate a new shared pointer for zero-copy sharing with other nodelets.
    velodyne_msgs::VelodyneScanPtr scan(new velodyne_msgs::VelodyneScan);
    //scan->packets.resize(config_.npackets);

    // Since the velodyne delivers data at a very high rate, keep
    // reading and publishing scans as fast as possible.
    double rotation = -1.0f, prev_rotation;
    bool scan_complete = false;
    while (!scan_complete && ros::ok())
    {
      velodyne_msgs::VelodynePacket pack;
      while (ros::ok())
      {
        // keep reading until full packet received
        int rc = input_->getPacket(&pack, config_.time_offset);

        //Add a check to avoid adding bad packets. The header bytes must be ffee or ffdd according to velodyne manual
        if(pack.data[0] != 0xff || (pack.data[1] != 0xee && pack.data[1] != 0xdd)) continue;

        scan->packets.push_back(pack);
        if (rc == 0) break;       // got a full packet?
        if (rc < 0) return false; // end of file reached?
      }
      uint16_t angle = (pack.data[3] << 8) | pack.data[2];      ///< 3rd and 4th bytes are angles 0-35999, divide by 100 to get degrees
      prev_rotation = rotation;
      rotation = angles::from_degrees(ROTATION_RESOLUTION * angle);

      // A rotation is complete when the angle in a packet crosses the cutoff threshold
      scan_complete = (prev_rotation > 0 && prev_rotation <= config_.cutoff_angle && rotation > config_.cutoff_angle);

      // The endpoints are special, detect an endpoint crossing
      scan_complete |= (config_.cutoff_angle == 0.0 || config_.cutoff_angle == M_2_PI) && prev_rotation > M_PI && rotation < M_PI;
      ROS_DEBUG_STREAM("Flag: " << unsigned(pack.data[0]) << " " << unsigned(pack.data[1]) <<" : Rotation: " << rotation << " : Cutoff Check: " << scan_complete);
    }

    // publish message using time of last packet read
    ROS_DEBUG_STREAM("Publishing a full Velodyne scan with " << scan->packets.size() <<" points.");
    scan->header.stamp = scan->packets.back().stamp;
    scan->header.frame_id = config_.frame_id;
    output_.publish(scan);

    // notify diagnostics that a message has been published, updating
    // its status
    diag_topic_->tick(scan->header.stamp);
    diagnostics_.update();

    return true;
  }

  void VelodyneDriver::callback(velodyne_driver::VelodyneNodeConfig &config,
                                uint32_t level)
  {
    ROS_INFO("Reconfigure Request");
    config_.time_offset = config.time_offset;
  }

} // namespace velodyne_driver

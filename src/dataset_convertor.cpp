/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Jan 13, 2016
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file dataset_convertor.cpp
 * @brief Source file for the VioParametersReader class.
 * @author Stefan Leutenegger
 * @author Andrea Nicastro
 */

#include <vector>
#include <map>
#include <memory>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <algorithm>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <ros/ros.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/chunked_file.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#pragma GCC diagnostic pop

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>

using namespace boost::filesystem;

const std::string RESET = "\033[0m";
const std::string BLACK = "0m";
const std::string RED = "1m";
const std::string GREEN = "2m";
const std::string BOLD = "\033[1;3";
const std::string REGULAR = "\033[0;3";
const std::string UNDERLINE = "\033[4;3";
const std::string BACKGROUND = "\033[4";

const int DOUBLE_PRECISION = 17;

// yaml parameters
const std::string SENSOR_LIST = "sensors";
const std::string CSVFILE = "data_file";
const std::string DATADIR = "data_dir";
const std::string SENSOR_TYPE = "type";
const std::string INFO = "info";
const std::string TOPIC = "topic";
const std::string NAME = "name";
const std::string CAMERA = "camera";
const std::string IMU = "imu";
const std::string VICON = "vicon";

///\todo: obsolete! delete
const std::string configDirectoryName = "/config/";
const std::string cam0DirectoryName = "/cam0";
const std::string cam1DirectoryName = "/cam1";
const std::string imuDirectoryName = "/imu0";
const std::string imuFileName = "imu0.csv";

std::ofstream imu_file_;

void signalHandler(int s)
{
  imu_file_.close();
}

std::string colouredString(std::string str, std::string colour, std::string option)
{
  return option + colour + str + RESET;
}

bool createDirs(std::string folderPath, std::map<std::string, std::map<std::string, std::string>> sensor_info)
{

  path p(folderPath);
  bool res = true;
  if (exists(p)) {
    std::cout << colouredString("\tCleaning previous dataset...", RED, REGULAR);
    remove_all(p);
    std::cout << colouredString("\t[DONE!]", GREEN, REGULAR) << std::endl;
  }

  std::cout << colouredString("\tCreating dataset folder...", RED, REGULAR);
  res = res && create_directories(p);
  std::cout << colouredString("\t[DONE!]", GREEN, REGULAR) << std::endl;

  for (auto &iterator : sensor_info) {
    std::stringstream sensor_folder;
    sensor_folder << folderPath;
    if (iterator.second.find(DATADIR) != iterator.second.end()) {
      sensor_folder << "/" << iterator.second[DATADIR];

    } else
      sensor_folder << "/" << iterator.second[NAME];
    path sensor_path(sensor_folder.str());

    res = res && create_directories(sensor_path);
  }

  return res;
}

void writeCameraHeader(std::shared_ptr<std::ofstream> file)
{
  *file << "#timestamp [ns]," << "filename" << std::endl;
}

void writeImuHeader(std::shared_ptr<std::ofstream> file)
{
  *file << "#timestamp [ns]," << "w_S_x [rad s^-1]," << "w_S_y [rad s^-1],"
      << "w_S_z [rad s^-1]," << "a_S_x [m s^-2]," << "a_S_y [m s^-2],"
      << "a_S_z [m s^-2]" << std::endl;
}

void writeViconHeader(std::shared_ptr<std::ofstream> file)
{
  *file << "#timestamp [ns]," << "p_S_x [m]," << "p_S_y [m]," << "p_S_z [m],"
      << "R_S_w []" << "R_S_x []," << "R_S_y []," << "R_S_z []" << std::endl;
}

void writeCSVHeaders(std::map<std::string, std::shared_ptr<std::ofstream>> &files,
                     const std::map<std::string, std::map<std::string, std::string>> &sensor_info)
{
  for (auto iterator : sensor_info) {
    if (iterator.second[SENSOR_TYPE].compare(CAMERA) == 0)
      writeCameraHeader(files[iterator.first]);
    else if (iterator.second[SENSOR_TYPE].compare(IMU) == 0)
      writeImuHeader(files[iterator.first]);
    else if (iterator.second[SENSOR_TYPE].compare(VICON) == 0)
      writeViconHeader(files[iterator.first]);
  }
}

std::map<std::string, std::shared_ptr<std::ofstream> > openFileStreams(
    const std::string folder_path, std::map<std::string, std::map<std::string, std::string>> &sensor_info)
{
  std::map<std::string, std::shared_ptr<std::ofstream>> topic2file_map;
  for (auto &iterator : sensor_info) {
    std::string topic = iterator.first;
    std::string csv_file_path = folder_path + std::string("/") + iterator.second[CSVFILE];
    std::ofstream *file = new std::ofstream(csv_file_path.c_str());
    std::shared_ptr < std::ofstream > file_ptr(file);
    topic2file_map.insert(
        std::pair<std::string, std::shared_ptr<std::ofstream>>(topic, file_ptr));
  }
  writeCSVHeaders(topic2file_map, sensor_info);
  return topic2file_map;
}

std::map<std::string, std::map<std::string, std::string> > sensorInfo(const ros::NodeHandle &nh)
{
  std::cout << colouredString("\tRetrieving sensor list...", RED, REGULAR);

  std::vector < std::string > sensor_list;
  if (!nh.getParam(SENSOR_LIST, sensor_list)) {
    std::stringstream msg;
    msg << "FAIL! Missing \"" << SENSOR_LIST
        << "\" parameter. Check your yaml or launch file";
    std::cout << colouredString(msg.str(), RED, BACKGROUND) << std::endl;
    exit (EXIT_FAILURE);
  }
  std::cout << colouredString("\t[DONE!]", GREEN, REGULAR) << std::endl;

  std::cout << colouredString("\tRetrieving CSV filename...", RED, REGULAR);

  std::string csv_filename;
  if (!nh.getParam(CSVFILE, csv_filename)) {
    std::stringstream msg;
    msg << "FAIL! Missing \"" << CSVFILE
        << "\" parameter. Check your yaml or launch file";
    std::cout << colouredString(msg.str(), RED, BACKGROUND) << std::endl;
    exit (EXIT_FAILURE);
  }
  std::cout << colouredString("\t[DONE!]", GREEN, REGULAR) << std::endl;

  std::cout << colouredString("\tRetrieving sensor list...", RED, REGULAR);
  std::map<std::string, std::map<std::string, std::string>> topic2info;

  std::map < std::string, std::string > sensor_new_info;

  for (std::string sensor : sensor_list) {
    sensor_new_info.clear();

    std::stringstream ss;
    ss << INFO << "/" << sensor;
    std::map < std::string, std::string > sensor_params;

    if (!nh.getParam(ss.str(), sensor_params)) {
      std::stringstream msg;
      msg << "FAIL! Missing \"" << ss.str()
          << "\" parameter. Check your yaml or launch file";
      std::cout << colouredString(msg.str(), RED, BACKGROUND) << std::endl;
      exit (EXIT_FAILURE);
    }

    std::string topic = sensor_params[TOPIC];

    std::string csv_file_path = sensor + std::string("/") + csv_filename;

    sensor_new_info.insert(std::pair<std::string, std::string>(CSVFILE, csv_file_path));

    if (sensor_params.find(DATADIR) != sensor_params.end()) {
      std::string data_dir = sensor + std::string("/") + sensor_params[DATADIR];
      sensor_new_info.insert(std::pair<std::string, std::string>(DATADIR, data_dir));
    }

    std::string sensor_type = sensor_params[SENSOR_TYPE];

    sensor_new_info.insert(std::pair<std::string, std::string>(SENSOR_TYPE, sensor_type));

    sensor_new_info.insert(std::pair<std::string, std::string>(NAME, sensor));

    topic2info.insert(
        std::pair<std::string, std::map<std::string, std::string>>(topic, sensor_new_info));

  }

  std::cout << colouredString("\t[DONE!]", GREEN, REGULAR) << std::endl;

  return topic2info;

}

void writeCSVCamera(std::shared_ptr<std::ofstream> file, ros::Time stamp)
{
  std::stringstream ss;
  ss << stamp.toNSec() << "," << stamp.toNSec() << ".png";

  *file << ss.str() << std::endl;
}

void writeCSVImu(std::shared_ptr<std::ofstream> file, sensor_msgs::Imu::ConstPtr imu)
{
  std::ostringstream ss;
  ss << std::setprecision(DOUBLE_PRECISION) << imu->header.stamp.toNSec() << ","
      << imu->angular_velocity.x << "," << imu->angular_velocity.y << ","
      << imu->angular_velocity.z << "," << imu->linear_acceleration.x << ","
      << imu->linear_acceleration.y << "," << imu->linear_acceleration.z;
  *file << ss.str() << std::endl;
}

void writeCSVVicon(std::shared_ptr<std::ofstream> file,
                   geometry_msgs::TransformStamped::ConstPtr vicon)
{
  std::ostringstream ss;
  ss << std::setprecision(DOUBLE_PRECISION) << vicon->header.stamp.toNSec()
      << "," << vicon->transform.translation.x << ","
      << vicon->transform.translation.y << "," << vicon->transform.translation.z
      << "," << vicon->transform.rotation.w << ","
      << vicon->transform.rotation.x << "," << vicon->transform.rotation.y
      << "," << vicon->transform.rotation.z;

  *file << ss.str() << std::endl;
}

bool isTopicInMap(std::map<std::string, std::map<std::string, std::string> > &topic2info,
                  std::string topic_name)
{

  bool res = false;
  if (topic2info.find(topic_name) != topic2info.end()) {
    res = true;
  } else {
    size_t first_slash = topic_name.find_first_of("/");
    if (first_slash == 0) {
      if (topic2info.find(topic_name.substr(1)) != topic2info.end()) {
        res = true;
      }
    } else {
      if (topic2info.find("/" + topic_name) != topic2info.end()) {
        res = true;
      }

    }
  }
  return res;
}

bool findTopicInMap(std::map<std::string, std::map<std::string, std::string> > &topic2info,
                    std::string topic_name,
                    std::map<std::string, std::map<std::string, std::string> >::iterator &element)
{
  element = topic2info.end();

  bool res = false;
  if (topic2info.find(topic_name) != topic2info.end()) {
    element = topic2info.find(topic_name);
    res = true;
  } else {
    size_t first_slash = topic_name.find_first_of("/");
    if (first_slash == 0) {
      if (topic2info.find(topic_name.substr(1)) != topic2info.end()) {
        element = topic2info.find(topic_name.substr(1));
        res = true;
      }
    } else {
      if (topic2info.find("/" + topic_name) != topic2info.end()) {
        element = topic2info.find("/" + topic_name);
        res = true;
      }

    }
  }
  return res;
}

int main(int argc, char **argv)
{

  std::cout << colouredString("Initializing ROS node:", RED, BOLD) << std::endl;

  ros::init(argc, argv, "dataset_converter");

  ros::NodeHandle nh;

  std::cout << colouredString("DONE!", GREEN, BOLD) << std::endl;

  std::cout << colouredString("Initializing sensor information:", RED, BOLD) << std::endl;

  std::map<std::string, std::map<std::string, std::string>> topic2info_map = sensorInfo(nh);

  std::cout << colouredString("DONE!", GREEN, BOLD) << std::endl;

  std::cout << colouredString("Creating folders:", RED, BOLD) << std::endl;

  std::string path(argv[1]);
  size_t pos = path.find_last_of("/");
  size_t pos_dot = path.find_last_of(".");

  std::string bagname;
  std::string bagpath;
  if (pos == std::string::npos) {
    std::cout
        << colouredString(
            "Relative path are not supported. Use an absolute path instead."
            "For example: roslaunch okvis_ros convert_datasert.launch bag:= /absolute/path/here",
            RED, BOLD) << std::endl;
    exit (EXIT_FAILURE);
  } else {
    bagname = path.substr(pos + 1, pos_dot - pos - 1);
    bagpath = path.substr(0, pos + 1);
  }

  if (!createDirs(bagpath + bagname, topic2info_map)) {
    std::cout << colouredString("FAILED!", RED, BACKGROUND);
    exit (EXIT_FAILURE);
  } else
    std::cout << colouredString("DONE!", GREEN, BOLD) << std::endl;

  std::cout << colouredString("Reading bag:", RED, BOLD) << std::endl;

  rosbag::Bag bag;
  std::cout << colouredString("\tOpening bag...", RED, REGULAR);
  bag.open(argv[1], rosbag::bagmode::Read);
  std::cout << colouredString("\t[DONE!]", GREEN, REGULAR) << std::endl;

  std::cout << colouredString("\tQuering topics bag...", RED, REGULAR);

  std::vector < std::string > topic_list;

  rosbag::View view(bag);

  std::vector<const rosbag::ConnectionInfo *> bag_info = view.getConnections();
  std::set < std::string > bag_topics;

  for (const rosbag::ConnectionInfo *info : bag_info) {
    std::string topic_name;
    topic_name = info->topic;

    if (isTopicInMap(topic2info_map, topic_name)) {
      topic_list.push_back(topic_name);
    }
  }

  view.addQuery(bag, rosbag::TopicQuery(topic_list));

  std::cout << colouredString("\t[DONE!]", GREEN, REGULAR) << std::endl;

  std::cout << colouredString("\tOpening file streams...", RED, REGULAR);

  std::map<std::string, std::shared_ptr<std::ofstream>> topic2file = openFileStreams(
      bagpath + bagname, topic2info_map);
  std::cout << colouredString("\t[DONE!]", GREEN, REGULAR) << std::endl;

  std::cout << colouredString("\tParsing the bag...\n\t", RED, REGULAR);

  double view_size = view.size();
  double counter = 0;
  for (auto bagIt : view) {
    std::string topic = bagIt.getTopic();

    std::map<std::string, std::map<std::string, std::string> >::iterator sensor_it;

    findTopicInMap(topic2info_map, topic, sensor_it);
    if (sensor_it == topic2info_map.end()) {
      counter++;
      std::printf("\r Progress: %.2f %%", 100.0 * counter / view_size);
      continue;
    }

    std::string sens_type = sensor_it->second.find(SENSOR_TYPE)->second;
    if (sens_type.compare(CAMERA) == 0) {
      sensor_msgs::Image::ConstPtr image =
          bagIt.instantiate<sensor_msgs::Image>();

      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
      cv::Mat image_cv = cv_ptr->image;

      std::stringstream ss;
      ss << bagpath << bagname << "/" << sensor_it->second.find(DATADIR)->second
          << "/" << image->header.stamp.toNSec() << ".png";
      cv::imwrite(ss.str(), image_cv);

      writeCSVCamera(topic2file.find(sensor_it->first)->second,
                     image->header.stamp);

    }
    if (sens_type.compare(IMU) == 0) {
      sensor_msgs::Imu::ConstPtr imuReading = bagIt
          .instantiate<sensor_msgs::Imu>();
      writeCSVImu(topic2file.find(sensor_it->first)->second, imuReading);
    }
    if (sens_type.compare(VICON) == 0) {

      geometry_msgs::TransformStamped::ConstPtr viconReading = bagIt
          .instantiate<geometry_msgs::TransformStamped>();
      writeCSVVicon(topic2file.find(sensor_it->first)->second, viconReading);
    }

    counter++;
    std::printf("\r Progress: %.2f %%", 100.0 * counter / view_size);

  }
  std::cout << colouredString("\n\t[DONE!]", GREEN, REGULAR) << std::endl;

  std::cout << colouredString("DONE!", GREEN, BOLD) << std::endl;

  std::cout << colouredString("Close file:", RED, BOLD) << std::endl;
  for (auto it : topic2file) {
    it.second->close();
  }
  std::cout << colouredString("DONE!", GREEN, BOLD) << std::endl;

  ros::shutdown();

}

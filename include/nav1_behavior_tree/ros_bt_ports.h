#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/PoseStamped.h>

namespace BT {

template <> inline geometry_msgs::PoseStamped convertFromString(StringView str)
{
  auto parts = splitString(str, ';');
  if (parts.size() == 3) {
    geometry_msgs::PoseStamped output;
    output.header.frame_id = "map";
    output.pose.orientation.w = 1;
    output.pose.orientation.x = 0;
    output.pose.orientation.y = 0;
    output.pose.orientation.z = 0;
    output.pose.position.x = convertFromString<double>(parts[0]);
    output.pose.position.y = convertFromString<double>(parts[1]);
    output.pose.position.z = convertFromString<double>(parts[2]);
    return output;
  }
  
  if (parts.size() == 4) {
    geometry_msgs::PoseStamped output;
    output.header.frame_id = convertFromString<std::string>(parts[0]);
    output.pose.orientation.w = 1;
    output.pose.orientation.x = 0;
    output.pose.orientation.y = 0;
    output.pose.orientation.z = 0;
    output.pose.position.x = convertFromString<double>(parts[1]);
    output.pose.position.y = convertFromString<double>(parts[2]);
    output.pose.position.z = convertFromString<double>(parts[3]);
    return output;
  }
  
  if (parts.size() == 8) {
    geometry_msgs::PoseStamped output;
    output.header.frame_id = convertFromString<std::string>(parts[0]);
    output.pose.orientation.w = convertFromString<double>(parts[1]);
    output.pose.orientation.x = convertFromString<double>(parts[2]);
    output.pose.orientation.y = convertFromString<double>(parts[3]);
    output.pose.orientation.z = convertFromString<double>(parts[4]);
    output.pose.position.x = convertFromString<double>(parts[5]);
    output.pose.position.y = convertFromString<double>(parts[6]);
    output.pose.position.z = convertFromString<double>(parts[7]);
    return output;
  }
  
  if (parts.size() == 11) {
    geometry_msgs::PoseStamped output;
    output.header.frame_id = convertFromString<std::string>(parts[0]);
    output.header.seq = convertFromString<uint32_t>(parts[1]);
    output.header.stamp = ros::Time(convertFromString<uint32_t>(parts[2]),
      convertFromString<uint32_t>(parts[3]));
    
    output.pose.orientation.w = convertFromString<double>(parts[4]);
    output.pose.orientation.x = convertFromString<double>(parts[5]);
    output.pose.orientation.y = convertFromString<double>(parts[6]);
    output.pose.orientation.z = convertFromString<double>(parts[7]);
    
    output.pose.position.x = convertFromString<double>(parts[8]);
    output.pose.position.y = convertFromString<double>(parts[9]);
    output.pose.position.z = convertFromString<double>(parts[10]);
    return output;
  }
  throw RuntimeError("invalid input)");
}

} // end namespace BT
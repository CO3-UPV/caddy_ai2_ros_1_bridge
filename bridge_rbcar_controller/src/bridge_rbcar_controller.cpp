// https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
#include "ros/ros.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/Imu.h>
#include <curtis_msgs/DriveData.h>
#include <rbcar_steering_controller/SteeringControllerStatus.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <async-sockets/udpsocket.hpp>
#include <async-sockets/udpserver.hpp>

#include <iostream>
#include <mutex>

UDPSocket<512> udpSocket_1(true);
UDPSocket<512> udpSocket_2(true);
UDPSocket<512> udpSocket_3(true);

bool _ros_1_server_binded_ = true;
std::mutex mtx;
UDPServer<512> udpServer;

void _sub_1_callback(const curtis_msgs::DriveDataConstPtr& message)
{
  json j;
  j["speed"] = (float) message->speed;
  udpSocket_1.Send(j.dump());
}

void _sub_2_callback(const sensor_msgs::ImuConstPtr& message){
  json j;
  j["orientation"]["x"] = (float) message->orientation.x;
  j["orientation"]["y"] = (float) message->orientation.y;
  j["orientation"]["z"] = (float) message->orientation.z;
  j["orientation"]["w"] = (float) message->orientation.w;
  j["angular_velocity"]["x"] = (float) message->angular_velocity.x;
  j["angular_velocity"]["y"] = (float) message->angular_velocity.y;
  j["angular_velocity"]["z"] = (float) message->angular_velocity.z;
  j["linear_acceleration"]["x"] = (float) message->linear_acceleration.x;
  j["linear_acceleration"]["y"] = (float) message->linear_acceleration.y;
  j["linear_acceleration"]["z"] = (float) message->linear_acceleration.z;
  udpSocket_2.Send(j.dump());
}

void _sub_3_callback(const rbcar_steering_controller::SteeringControllerStatusConstPtr& message)
{
  json j;
  j["steering"] = (float) message->steering_position;
  udpSocket_3.Send(j.dump());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bridge_rbcar_controller");
  ros::NodeHandle _node_;

  std::string _pub_topic_;
  int _ros_1_server_port_;

  std::string _ros_2_server_ip_;
  std::string _sub_topic_1_;
  int _ros_2_server_port_1_;
  std::string _sub_topic_2_;
  int _ros_2_server_port_2_;
  std::string _sub_topic_3_;
  int _ros_2_server_port_3_;

  int _Hz_;

  _node_.param<std::string>("pub_topic", _pub_topic_, "command");
  _node_.param<int>("ros_1_server_port", _ros_1_server_port_, 8888);

  _node_.param<std::string>("ros_2_server_ip", _ros_2_server_ip_, "localhost");
  _node_.param<std::string>("sub_topic_1", _sub_topic_1_, "master_drive");
  _node_.param<int>("ros_2_server_port_1", _ros_2_server_port_1_, 8889);
  _node_.param<std::string>("sub_topic_2", _sub_topic_2_, "imu");
  _node_.param<int>("ros_2_server_port_2", _ros_2_server_port_2_, 8890);
  _node_.param<std::string>("sub_topic_3", _sub_topic_3_, "status");
  _node_.param<int>("ros_2_server_port_3", _ros_2_server_port_3_, 8891);

  _node_.param<int>("Hz", _Hz_, 100);


  ros::Subscriber _sub_1_ = _node_.subscribe(_sub_topic_1_, 1, _sub_1_callback);
  ros::Subscriber _sub_2_ = _node_.subscribe(_sub_topic_2_, 1, _sub_2_callback);
  ros::Subscriber _sub_3_ = _node_.subscribe(_sub_topic_3_, 1, _sub_3_callback);
  ros::Publisher _pub_ = _node_.advertise<ackermann_msgs::AckermannDriveStamped>(_pub_topic_, 1);

  udpSocket_1.Connect(_ros_2_server_ip_, _ros_2_server_port_1_);
  udpSocket_2.Connect(_ros_2_server_ip_, _ros_2_server_port_2_);
  udpSocket_3.Connect(_ros_2_server_ip_, _ros_2_server_port_3_);

  udpServer.Bind(_ros_1_server_port_, [](int errorCode, std::string errorMessage) {
      _ros_1_server_binded_ = false; // Error binding socket
  });
  if (!_ros_1_server_binded_) 
  { 
    ROS_ERROR("No se ha podido iniciar correctamente el servidor UDP - ha fallado el BINDING - posiblemente puerto ocupado por otro proceso."); 
    return -1;
  }


  bool _new_incoming_message_ = false;
  uint32_t _pub_message_counter_ = 0;
  json _pub_json_;
  ackermann_msgs::AckermannDriveStamped _pub_msg_;


  udpServer.onRawMessageReceived = [&](const char* message, int length, std::string ipv4, uint16_t port) {
      mtx.lock();
      _pub_json_ = json::parse(message);
      _new_incoming_message_ = true;
      mtx.unlock();
  };


  ros::Rate loop_rate(_Hz_);
  while (ros::ok())
  {
    if(_new_incoming_message_)
    {
      _pub_msg_.header.seq = _pub_message_counter_;
      _pub_msg_.header.stamp = ros::Time::now();
      // _pub_msg_.header.frame_id = ""; // Unknown

      mtx.lock();
      _pub_msg_.drive.steering_angle = _pub_json_["drive"]["steering_angle"];
      //_pub_msg_.drive.steering_angle_velocity = _pub_json_["drive"]["steering_angle_velocity"]; // Not used
      _pub_msg_.drive.speed = _pub_json_["drive"]["speed"];
      //_pub_msg_.drive.acceleration = _pub_json_["drive"]["acceleration"];  // Not used
      //_pub_msg_.drive.jerk = _pub_json_["drive"]["jerk"];  // Not used

      _new_incoming_message_ = false;
      mtx.unlock();

      _pub_message_counter_++;

      _pub_.publish(_pub_msg_);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  udpSocket_1.Close();
  udpSocket_2.Close();
  udpSocket_3.Close();
  udpServer.Close();

  return 0;
}
// https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
#include "ros/ros.h"
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <async-sockets/udpsocket.hpp>
#include <async-sockets/udpserver.hpp>

#include <iostream>
#include <mutex>

int counter = 0;
bool _ros_1_server_binded_ = true;
std::mutex mtx; // Declarar un mutex
UDPSocket<512> udpSocket(true);
UDPServer<512> udpServer;

/*
void _sub_callback(const ackermann_msgs::AckermannDriveStampedConstPtr& message)
{
  json j;
  // https://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html
  // https://docs.ros.org/en/jade/api/std_msgs/html/msg/Header.html
  // https://docs.ros2.org/galactic/api/builtin_interfaces/msg/Time.html
  // https://docs.ros.org/en/diamondback/api/rostime/html/classros_1_1Time.html
  j["header"]["seq"] = (uint32_t) counter; // message.header.seq; // Not used in ROS 2, en ROS 1 creo que es un id incremental
  j["header"]["stamp"]["sec"] = (int32_t) message->header.stamp.sec; // En ROS 1 es uint32_t, ya veremos como compatibilizar esto
  j["header"]["stamp"]["nanosec"] = (uint32_t) message->header.stamp.nsec;
  j["header"]["frame_id"] = (std::string) message->header.frame_id;
  // https://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDrive.html
  j["drive"]["steering_angle"] = (float) message->drive.steering_angle;
  j["drive"]["steering_angle_velocity"] = (float) message->drive.steering_angle_velocity;
  j["drive"]["speed"] = (float) message->drive.speed;
  j["drive"]["acceleration"] = (float) message->drive.acceleration;
  j["drive"]["jerk"] = (float) message->drive.jerk;
  counter++;  
}
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bridge_rbcar_controller");
  ros::NodeHandle _node_;

  //std::string _sub_topic_;
  std::string _pub_topic_;
  std::string _ros_2_server_ip_;
  int _ros_2_server_port_ = 8888;
  int _ros_1_server_port_ = 8889;
  int _Hz_ = 10;

  _node_.param<std::string>("_pub_topic_", _pub_topic_, "command");
  _node_.param<std::string>("_ros_2_server_ip_", _ros_2_server_ip_, "localhost");
  _node_.param<int>("_ros_2_server_port_", _ros_2_server_port_, 8888);
  _node_.param<int>("_ros_1_server_port_", _ros_1_server_port_, 8889);
  _node_.param<int>("_Hz_", _Hz_, 10);


  //ros::Subscriber _sub_ = _node_.subscribe(_sub_topic_, 1, _sub_callback);
  ros::Publisher _pub_ = _node_.advertise<ackermann_msgs::AckermannDriveStamped>(_pub_topic_, 1);

  //udpSocket.Connect(_IP, _PORT);

  // Bind the server to a port.
  udpServer.Bind(_ros_1_server_port_, [](int errorCode, std::string errorMessage) {
      // BINDING FAILED:
      // std::cout << errorCode << " : " << errorMessage << std::endl;
      _ros_1_server_binded_ = false;
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
      std::cout << ipv4 << ":" << port << " => " << message << "(" << length << ")" << std::endl;
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

      std::cout << "HERE" << std::endl;

      _pub_message_counter_++;

      _pub_.publish(_pub_msg_);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  //udpSocket.Close();
  udpServer.Close();

  return 0;
}
#include <ros/ros.h>
#include <message_ui/sent_msg.h>
#include <arithmetic_node/arithmetic_reply.h>
#include <string>

using namespace std;

#define PUBLISH_TOPIC "arithmetic_reply"
#define SUBSCRIBE_TOPIC "sent_msg"

// define publisher
ros::Publisher reply_msg_pub;
std::string name;

void sentMsgCallback(const message_ui::sent_msg::ConstPtr& msg) {
  double time_received = ros::Time::now().toSec();
  std::string msg_content = msg->message;
  
  size_t add_pos = msg_content.find('+');
  size_t sub_pos = msg_content.find('-');
  size_t mul_pos = msg_content.find('*');
  size_t div_pos = msg_content.find('/');

  float answer;
  std::string oper_type;
  
  if (add_pos != std::string::npos) {
    try {
      std::string first = msg_content.substr(0,add_pos);
      std::string second = msg_content.substr(add_pos+1);
	  double first_double = std::stod(first);
      double second_double = std::stod(second);
      answer = (float) (first_double + second_double);
      oper_type = "Add";
    } catch (...) {
      return;
    }
  } else if (sub_pos != std::string::npos) {
    try {
      std::string first = msg_content.substr(0,sub_pos);
      std::string second = msg_content.substr(sub_pos+1);
	  double first_double = std::stod(first);
      double second_double = std::stod(second);
      answer = (float) (first_double - second_double);
      oper_type = "Subtract";
    } catch (...) {
      return;
    }
  } else if (mul_pos != std::string::npos) {
    try {
      std::string first = msg_content.substr(0,mul_pos);
      std::string second = msg_content.substr(mul_pos+1);
	  double first_double = std::stod(first);
      double second_double = std::stod(second);
      answer = (float) (first_double * second_double);
      oper_type = "Multipy";
    } catch (...) {
      return;
    }
  } else if (div_pos != std::string::npos) {
    try {
      std::string first = msg_content.substr(0,div_pos);
      std::string second = msg_content.substr(div_pos+1);
	  double first_double = std::stod(first);
      double second_double = std::stod(second);
      answer = (float) (first_double / second_double);
      oper_type = "Divide";
    } catch (...) {
      return;
    }
  } else {
    return;
  }

  double time_answered = ros::Time::now().toSec();
  double process_time = time_answered - time_received;

  arithmetic_node::arithmetic_reply new_msg;
  new_msg.header.seq = msg->header.seq;
  new_msg.header.frame_id = msg->header.frame_id;
  new_msg.header.stamp = ros::Time::now();
  new_msg.oper_type = oper_type;
  new_msg.answer = answer;
  new_msg.time_received = time_received;
  new_msg.time_answered = time_answered;
  new_msg.process_time = process_time;
  reply_msg_pub.publish(new_msg);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "chatbot_node");
  ros::NodeHandle n;

  // initialize publisher
  reply_msg_pub = n.advertise<arithmetic_node::arithmetic_reply>(PUBLISH_TOPIC, 1000);

  // initialize subscriber
  ros::Subscriber sent_msg_sub = n.subscribe(SUBSCRIBE_TOPIC, 1000, sentMsgCallback);

  ros::Rate loop_rate(20);

  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

#include <ros/ros.h>
#include <chatbot_node/reply_msg.h>
#include <message_ui/sent_msg.h>
#include <string>

using namespace std;

#define PUBLISH_TOPIC "reply_msg"
#define SUBSCRIBE_TOPIC "sent_msg"
#define PARAM_NAME "name"

#define QUESTION_1 "Hello"
#define QUESTION_2 "What is your name?"
#define QUESTION_3 "How are you?"

// define publisher
ros::Publisher reply_msg_pub;
std::string name;

void sentMsgCallback(const message_ui::sent_msg::ConstPtr& msg) {
  std::string msg_content = msg->message;
  
  std::string reply_string;
  if (msg_content.compare(QUESTION_1) == 0) {
    reply_string = std::string("Hello, ")+name;
  } else if (msg_content.compare(QUESTION_2) == 0) {
    reply_string = std::string("My name is MRSD Siri");
  } else if (msg_content.compare(QUESTION_3) == 0) {
	reply_string = std::string("I am fine, thank you.");
  }

  chatbot_node::reply_msg new_msg;
  new_msg.message = reply_string;
  new_msg.header.seq = msg->header.seq;
  new_msg.header.frame_id = msg->header.frame_id;
  new_msg.header.stamp = ros::Time::now();
  reply_msg_pub.publish(new_msg);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "chatbot_node");
  ros::NodeHandle n;

  n.getParam(PARAM_NAME, name);

  // initialize publisher
  reply_msg_pub = n.advertise<chatbot_node::reply_msg>(PUBLISH_TOPIC, 1000);

  // initialize subscriber
  ros::Subscriber sent_msg_sub = n.subscribe(SUBSCRIBE_TOPIC, 1000, sentMsgCallback);

  ros::Rate loop_rate(20);

  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

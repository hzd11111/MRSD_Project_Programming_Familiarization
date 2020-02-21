#include <ros/ros.h>
#include <chatbot_node/reply_msg.h>
#include <message_ui/sent_msg.h>
#include <counter_node/counter.h>
#include <arithmetic_node/arithmetic_reply.h>

#define SERVICE_TOPIC "message_counter"

int num_reply_msg = 0;
int num_sent_msg = 0;

ros::Time last_sent_msg_time;
ros::Time last_reply_msg_time;

ros::Subscriber reply_msg_sub;
ros::Subscriber arithmetic_reply_msg_sub;
ros::Subscriber sent_msg_sub;

void sent_msg_callback(const message_ui::sent_msg msg)
{
	num_sent_msg++;
	last_sent_msg_time = msg.header.stamp;
}

void reply_msg_callback(const chatbot_node::reply_msg msg)
{
	num_reply_msg++;
	last_reply_msg_time = msg.header.stamp;
}

void arithmetic_reply_msg_callback(const arithmetic_node::arithmetic_reply msg)
{
	num_reply_msg++;
	last_reply_msg_time = msg.header.stamp;
}

bool serviceReply(counter_node::counter::Request &req,
				  counter_node::counter::Response &res) {
  int16_t req_id = req.req_id;

  if (req_id == 0) { // total number of messages
	res.reply = num_sent_msg + num_reply_msg;
  } else if (req_id == 1) { // total number of replied messages
    res.reply = num_reply_msg;
  } else if (req_id == 2) { // total number of sent messages
    res.reply = num_sent_msg;
  } else if (req_id == 3) { // time elapsed since last reply message
    double secs = ros::Time::now().toSec();
    res.reply = (float) (secs - last_reply_msg_time.toSec());
  } else if (req_id == 4) { // time elapsed since last user message
    double secs = ros::Time::now().toSec();
    res.reply = (float) (secs - last_sent_msg_time.toSec());
  }
  return true;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "counter_node");
  ros::NodeHandle n;

  reply_msg_sub = n.subscribe("reply_msg", 1000, reply_msg_callback);
  sent_msg_sub = n.subscribe("sent_msg", 1000, sent_msg_callback);
  arithmetic_reply_msg_sub = n.subscribe("arithmetic_reply", 1000, arithmetic_reply_msg_callback);

  // initialize service
  ros::ServiceServer counter_service = n.advertiseService(SERVICE_TOPIC, serviceReply);
  ros::Rate loop_rate(20);

  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

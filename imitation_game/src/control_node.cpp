/** @file central_node.cpp
*  @brief File storing definitions of the state machine functions.
*/

#include <control_node.hpp>

using namespace std;

ContralNode::ContralNode()
{
  // Create the service client
  text_to_speech_client = nh.serviceClient<SERVICE TYPE>("SERVICE NAME");
  // Subscribe to the text
  text_sub  = nh.advertiseService("ig/text_to_speech", text_to_speech);
}

bool text_to_speech(imitation_game::text_to_speech::Request  &req, imitation_game::text_to_speech::Response &res)
{
  string sentence = text[req.id];

  SERVICE_TYPE msg;
  msg.req = sentence;

  // Create the message and call service
  text_to_speech_client.call()
  return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ig_control");
  ControlNode cn;
  ros::spin();
  sleep(1);
  return 0;
}

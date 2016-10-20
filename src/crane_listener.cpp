// %Tag(FULLTEXT)%


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "evart_bridge/TrackSegment.h"
#include "crane_message_handler.hh"
#include "signal_handler.hh"

// %EndTag(CALLBACK)%

struct evart_set_of_points{
  std::string body_name_;
  std::string body_ros_name_;
  std::string segment_name_;

  evart_set_of_points(const std::string &bn,
		      const std::string &brn,
		      const std::string &sn) : 
    body_name_(bn), 
    body_ros_name_(brn), 
    segment_name_(sn) 
  {}

} ;


void init_ros(ros::NodeHandle &n,
	      CraneMessageHandler &craneMsgHandler,
	      evart_set_of_points set_to_follow)
{

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  // %Tag(SUBSCRIBER)%

  // Call evart to follow HRP2.
  ros::ServiceClient client = n.serviceClient<evart_bridge::TrackSegment>("/evart/track_segments");

  evart_bridge::TrackSegment trackASegment; 
  
  trackASegment.request.body_name=set_to_follow.body_name_;
  trackASegment.request.segment_name=set_to_follow.segment_name_;

  unsigned int res_to_call;

  if ((res_to_call=client.call(trackASegment)))
    {
      std::cout << "Succeeded in subscribing to track_segment:" 
		<< trackASegment.request.body_name << "/" 
		<< trackASegment.request.segment_name 
		<< std::endl;
    }
  else
    {
      std::cerr << "Failed to subscribe to " 
		<< trackASegment.request.body_name << "/" 
		<< trackASegment.request.segment_name << std::endl;
    }

  craneMsgHandler.setMocapStatus(res_to_call);
  
}


int main(int argc, char **argv)
{
  int iret=0;
  CraneMessageHandler craneMsgHandler;

  evart_set_of_points 
  /*set_to_follow(std::string("facom-box"),
		  std::string("facom_box"),
		  std::string("facom_box"));*/
  set_to_follow(std::string("hrp2_14_head"),
		std::string("hrp2_14_head"),
		std::string("hrp2_14_head"));
  /*set_to_follow(std::string("helmet"),
		std::string("helmet"),
		std::string("helmet"));*/

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;


  init_ros(n,craneMsgHandler,set_to_follow);

  ros::Subscriber sub = n.subscribe("joy", 1, 
				    &CraneMessageHandler::chatterCallback,
				    &craneMsgHandler); 

  std::string evart_topic("/evart/"+ set_to_follow.body_ros_name_ + "/" + set_to_follow.segment_name_);
  sub = n.subscribe(evart_topic.c_str(),
		    1000,&CraneMessageHandler::updatePositionCallback,&craneMsgHandler);
  // %EndTag(SUBSCRIBER)%

  ros::Rate loop_rate(50);

  int count = 0;

  
  try
    {
   
      while (ros::ok())
	{
	  
	  /**
	   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
	   * callbacks will be called from within this thread (the main one).  ros::spin()
	   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	   */
	  // %Tag(SPIN)%
	  // %EndTag(SPIN)%
  
	  ros::spinOnce();
	  // %EndTag(SPINONCE)%

	  // Control the crane
	  craneMsgHandler.applyControlStrategy();

	  // %Tag(RATE_SLEEP)%
	  loop_rate.sleep();
	  // %EndTag(RATE_SLEEP)%
	  ++count;
	}

    }

  catch(SignalException &e)
    {
      craneMsgHandler.stopEverything();
      std::cerr << "SignalException: " << e.what() << std::endl;
      iret = EXIT_FAILURE;
    }
  return iret;
}
// %EndTag(FULLTEXT)%

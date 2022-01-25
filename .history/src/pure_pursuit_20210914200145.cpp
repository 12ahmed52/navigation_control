#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include <string>
#include <iostream>
#include <cmath>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include "kinmatic_model/model_state.h"
#include "std_msgs/UInt16MultiArray.h"

using namespace std;


kinmatic_model::model_state nav_msg_out;





class sub_pub_node
{
public:

  sub_pub_node()
  {
	path_to_nav_sub = handel.subscribe("/path", 10,&sub_pub_node::path_callback,this );
    model_to_nav_sub = handel.subscribe("/model_k_state", 10,&sub_pub_node::model_k_state_callback,this );

	nav_out_pub = handel.advertise<kinmatic_model::model_state>("/nav_output", 10);

      ros::Rate loop_rate(1);
      

  
  }

  void publish()
  {
    nav_out_pub.publish(nav_msg_out);
    //model_out_pub.publish(model_msg_output);

  }

    void path_callback (const std_msgs::UInt16MultiArray::ConstPtr& msg)
    {

    }
    void model_k_state_callback(const kinmatic_model::model_state::ConstPtr& msg)
    {
        
    }

private:
  ros::NodeHandle handel; 
  ros::Publisher nav_out_pub;

  ros::Subscriber model_to_nav_sub;
  ros::Subscriber path_to_nav_sub;

};





int main(int argc, char **argv)
{
    
  ros::init(argc, argv, "model");

  sub_pub_node sub_pub_object;

  //ros::spin();
  
  ros::Rate loop_rate(100);


  //sub_pub_object.publish();
ros::spin();

/*
  while (ros::ok())
  {
        sub_pub_object.publish();

    ros::spinOnce();
    loop_rate.sleep();
  }
*/
  return 0;
}
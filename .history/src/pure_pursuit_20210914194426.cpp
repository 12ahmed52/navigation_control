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








class sub_pub_node
{
public:

  sub_pub_node()
  {
	path_to_nav_sub = handel.subscribe("/path", 10,&sub_pub_node::path_callback,this );
    model_to_nav_sub = handel.subscribe("/model_k_state", 10,&sub_pub_node::model_k_state_callback,this );

	model_out_pub = handel.advertise<geometry_msgs::Pose2D>("/model_k_state", 10);

      ros::Rate loop_rate(1);
      

  
  }

  void publish()
  {
    //nav_out_pub.publish(nav_msg_out);
    model_out_pub.publish(model_msg_output);

  }


  void model_input_callback(const kinmatic_model::model_state::ConstPtr& msg)
  {
    //stringstream ss(msg->data);
     //ss>>name>>age>>height;
    //ROS_INFO("naem: [%s]",name.c_str());
    

    ROS_INFO("x  : [%f]",model_msg_output.x);
        ROS_INFO("y  : [%f]",model_msg_output.y);
                ROS_INFO("theta  : [%f]",model_msg_output.theta);


    bike1.input[0]= msg->cd_velocity;
    bike1.input[2]=msg->steering_wheel;
    ROS_INFO("velocity: [%f]",msg->cd_velocity);
    ROS_INFO("steering angel: [%f]",msg->steering_wheel);
    cout<<"============================"<<endl;

    bike1.forward_kinematics();
   // bike1.print_output();

    model_msg_output.x= bike1.output[0];
    model_msg_output.y= bike1.output[1];
    model_msg_output.theta= bike1.output[2];
    
    //model_out_pub.publish(model_msg_output);
    //publish();
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
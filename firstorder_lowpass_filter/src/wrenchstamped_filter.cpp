#include <firstorder_lowpass_filter/FirstorderLowpassFilter.h>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <functional>

int main(int argc, char** argv){
  ros::init(argc,argv,"wrenchstamped_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  double cutoff_freq;
  pnh.param("cutoff_freq", cutoff_freq, 1.0); // 1 hz

  std::vector<firstorder_lowpass_filter::FirstOrderLowPassFilter<double> > filters;
  for(size_t i=0;i<6;i++){
    filters.push_back(firstorder_lowpass_filter::FirstOrderLowPassFilter<double>(cutoff_freq,0.1,0));
  }
  ros::Time stamp;

  ros::Publisher wrenchStampedPub = pnh.advertise<geometry_msgs::WrenchStamped>("output",1000);
  ros::Subscriber wrenchStampedSub =
    pnh.subscribe<geometry_msgs::WrenchStamped>
    ("input",
     1,
     [&](const geometry_msgs::WrenchStamped::ConstPtr& msg) {
      double dt = (msg->header.stamp - stamp).toSec();
      if(dt<=0) return;

      geometry_msgs::WrenchStamped output;
      output.header = msg->header;
      output.wrench.force.x = filters[0].passFilter(msg->wrench.force.x,dt);
      output.wrench.force.y = filters[1].passFilter(msg->wrench.force.y,dt);
      output.wrench.force.z = filters[2].passFilter(msg->wrench.force.z,dt);
      output.wrench.torque.x = filters[3].passFilter(msg->wrench.torque.x,dt);
      output.wrench.torque.y = filters[4].passFilter(msg->wrench.torque.y,dt);
      output.wrench.torque.z = filters[5].passFilter(msg->wrench.torque.z,dt);

      wrenchStampedPub.publish(output);
      stamp = msg->header.stamp;
     });

  ros::spin();

  return 0;
}

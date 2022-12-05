#include <cpp_filters/IIRFilter.h>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <functional>
#include <Eigen/Eigen>
#include <dynamic_reconfigure/server.h>
#include <wrench_filter/WrenchFilterConfig.h>

int main(int argc, char** argv){
  ros::init(argc,argv,"wrench_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  cpp_filters::IIRFilter<Eigen::VectorXd> lpf;
  cpp_filters::IIRFilter<Eigen::VectorXd> lpf_for_hpf;
  ros::Rate r(100);
  wrench_filter::WrenchFilterConfig config;
  bool is_initial = true;

  dynamic_reconfigure::Server<wrench_filter::WrenchFilterConfig> cfgServer_;
  cfgServer_.setCallback([&](wrench_filter::WrenchFilterConfig& config_in, int32_t level){
      if(is_initial ||
         config.lpf_cutoff_hz != config_in.lpf_cutoff_hz ||
         config.rate != config_in.rate)
        lpf.setParameterAsBiquad(config_in.lpf_cutoff_hz, 1.0/sqrt(2), config_in.rate, lpf.get());
      if(is_initial ||
         config.hpf_cutoff_hz != config_in.hpf_cutoff_hz ||
         config.rate != config_in.rate)
        lpf_for_hpf.setParameterAsBiquad(config_in.hpf_cutoff_hz, 1.0/sqrt(2), config_in.rate, lpf_for_hpf.get());
      if(is_initial ||
         config.rate != config_in.rate)
        r = ros::Rate(config_in.rate);
      config = config_in;
      is_initial = false;
    });

  Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
  std::string frame_id;

  ros::Publisher pub = pnh.advertise<geometry_msgs::WrenchStamped>("output",1);
  ros::Subscriber sub =
    pnh.subscribe<geometry_msgs::WrenchStamped>
    ("input",
     1,
     [&](const geometry_msgs::WrenchStamped::ConstPtr& msg) {
      frame_id = msg->header.frame_id;
      wrench[0] = msg->wrench.force.x;
      wrench[1] = msg->wrench.force.y;
      wrench[2] = msg->wrench.force.z;
      wrench[3] = msg->wrench.torque.x;
      wrench[4] = msg->wrench.torque.y;
      wrench[5] = msg->wrench.torque.z;
     }
     );

  geometry_msgs::WrenchStamped msg;
  while(ros::ok()){
    ros::spinOnce();

    const Eigen::VectorXd w_hp = wrench - lpf_for_hpf.passFilter( wrench );
    const Eigen::VectorXd w_lp = lpf.passFilter( wrench );
    const Eigen::VectorXd w_shaped = wrench * config.gain + w_hp * config.hpf_gain + w_lp * config.lpf_gain;

    Eigen::VectorXd wrench_limited(6);
    for(int i=0;i<3;i++) wrench_limited[i] = std::max(std::min(w_shaped[i], config.max_force), -config.max_force);
    for(int i=3;i<6;i++) wrench_limited[i] = std::max(std::min(w_shaped[i], config.max_torque), -config.max_torque);

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    msg.wrench.force.x = wrench_limited[0];
    msg.wrench.force.y = wrench_limited[1];
    msg.wrench.force.z = wrench_limited[2];
    msg.wrench.torque.x = wrench_limited[3];
    msg.wrench.torque.y = wrench_limited[4];
    msg.wrench.torque.z = wrench_limited[5];
    pub.publish(msg);

    r.sleep();
  }


  return 0;
}

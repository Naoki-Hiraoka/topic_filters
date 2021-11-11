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

  wrench_filter::WrenchFilterConfig config;
  pnh.param("hpf_cutoff_hz", config.hpf_cutoff_hz, 20.0);
  pnh.param("lpf_cutoff_hz", config.lpf_cutoff_hz, 0.3);
  pnh.param("gain", config.gain, 0.1);
  pnh.param("hpf_gain", config.hpf_gain, 0.1);
  pnh.param("lpf_gain", config.lpf_gain, 0.0);
  pnh.param("rate", config.rate, 100.0);

  cpp_filters::IIRFilter<Eigen::VectorXd> lpf;
  lpf.setParameterAsBiquad(config.lpf_cutoff_hz, 1.0/sqrt(2), config.rate, Eigen::VectorXd::Zero(6));
  cpp_filters::IIRFilter<Eigen::VectorXd> lpf_for_hpf;
  lpf_for_hpf.setParameterAsBiquad(config.hpf_cutoff_hz, 1.0/sqrt(2), config.rate, Eigen::VectorXd::Zero(6));
  ros::Rate r(config.rate);

  dynamic_reconfigure::Server<wrench_filter::WrenchFilterConfig> cfgServer_;
  cfgServer_.setConfigDefault(config);
  cfgServer_.updateConfig(config);
  cfgServer_.setCallback([&](wrench_filter::WrenchFilterConfig& config_in, int32_t level){
      if(config.lpf_cutoff_hz != config_in.lpf_cutoff_hz ||
         config.rate != config_in.rate)
        lpf.setParameterAsBiquad(config_in.lpf_cutoff_hz, 1.0/sqrt(2), config_in.rate, lpf.get());
      if(config.hpf_cutoff_hz != config_in.hpf_cutoff_hz ||
         config.rate != config_in.rate)
        lpf_for_hpf.setParameterAsBiquad(config_in.hpf_cutoff_hz, 1.0/sqrt(2), config_in.rate, lpf_for_hpf.get());
      if(config.rate != config_in.rate)
        r = ros::Rate(config_in.rate);
      config = config_in;
    });//setCallbackの中でcallbackが呼ばれるので、その前にupdateConfigを呼ぶ必要がある

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

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    msg.wrench.force.x = w_shaped[0];
    msg.wrench.force.y = w_shaped[1];
    msg.wrench.force.z = w_shaped[2];
    msg.wrench.torque.x = w_shaped[3];
    msg.wrench.torque.y = w_shaped[4];
    msg.wrench.torque.z = w_shaped[5];
    pub.publish(msg);

    r.sleep();
  }


  return 0;
}

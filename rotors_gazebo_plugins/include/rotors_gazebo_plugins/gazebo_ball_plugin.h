#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_BAl_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_BAl_PLUGIN_H

#include <functional>
#include <gazebo/common/common.hh> 
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <mav_msgs/default_topics.h>  // This comes from the mav_comm repo


#include "rotors_gazebo_plugins/common.h"
#include "Vector3dStamped.pb.h"
#include "Float32.pb.h"
#include "ConnectGazeboToRosTopic.pb.h"

namespace gazebo {
class GazeboBall : public ModelPlugin {
 public:
  GazeboBall();
  ~GazeboBall();

  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate();

 private:
  void CreatePubsAndSubs();
  void Publish();
  
  std::string namespace_;
  std::string ball_pos_pub_topic_;

  physics::ModelPtr model_;
  event::ConnectionPtr update_connection_;

  gazebo::transport::NodePtr node_handle_;

  gazebo::transport::PublisherPtr ball_position_pub_;

  gz_geometry_msgs::Vector3dStamped position_msg_;
  gz_std_msgs::Float32 pos_x_msg_;

  bool first_time_;
  bool pubs_and_subs_created_;
};

// Register this plugin with the simulator
}  // namespace gazebo

#endif
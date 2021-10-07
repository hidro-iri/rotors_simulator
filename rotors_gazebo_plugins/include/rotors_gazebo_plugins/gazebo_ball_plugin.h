#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_BALL_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_BALL_PLUGIN_H

#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <mav_msgs/default_topics.h>  // This comes from the mav_comm repo

#include "rotors_gazebo_plugins/common.h"

#include "rotors_gazebo_plugins/common.h"
#include "Vector3dStamped.pb.h"
#include "Float32.pb.h"
#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"
#include "ExternalTrigger.pb.h"
#include "TransformStampedWithFrameIds.pb.h"

namespace gazebo {

typedef const boost::shared_ptr<const gz_mav_msgs::ExternalTrigger> GzExternalTriggerMsgPtr;

class GazeboBall : public ModelPlugin {
 public:
  GazeboBall();
  ~GazeboBall();

  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate();

 private:
  void CreatePubsAndSubs();
  void Publish();

  void CallbackThrowBall(GzExternalTriggerMsgPtr& external_trigger_input_msg);

  std::string namespace_;

  std::string pos_pub_topic_;
  std::string vel_pub_topic_;

  std::string throw_sub_topic_;

  std::string parent_frame_id_;
  std::string child_frame_id_;

  physics::ModelPtr model_;
  event::ConnectionPtr update_connection_;

  gazebo::transport::NodePtr node_handle_;

  gazebo::transport::PublisherPtr pos_pub_;
  gazebo::transport::PublisherPtr vel_pub_;
  gazebo::transport::PublisherPtr broadcast_transform_pub_;

  gazebo::transport::SubscriberPtr throw_sub_;

  gz_geometry_msgs::Vector3dStamped pos_msg_;
  gz_geometry_msgs::Vector3dStamped vel_msg_;

  ignition::math::Vector3d pos_0_;
  ignition::math::Vector3d vel_0_;

  bool pubs_and_subs_created_;
  bool thrown_;
};

// Register this plugin with the simulator
}  // namespace gazebo

#endif
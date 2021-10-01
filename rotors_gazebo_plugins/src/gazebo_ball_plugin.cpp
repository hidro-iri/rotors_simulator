#include "rotors_gazebo_plugins/gazebo_ball_plugin.h"

namespace gazebo {

GazeboBall::GazeboBall() : ModelPlugin() {}

GazeboBall::~GazeboBall() {}

void GazeboBall::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;

  getSdfParam<std::string>(_sdf, "ballNamespace", namespace_, "ball");
  getSdfParam<std::string>(_sdf, "commandSubTopic", ball_pos_pub_topic_, "position");

  node_handle_ = gazebo::transport::NodePtr(new transport::Node());
  // Initialise with default namespace (typically /gazebo/default/)
  node_handle_->Init();

  first_time_ = true;
  pubs_and_subs_created_ = false;

  update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboBall::OnUpdate, this));
}

void GazeboBall::OnUpdate() {
  if (first_time_) {
    model_->SetLinearVel(ignition::math::Vector3d(1, 0, 1));
    first_time_ = false;
  }

  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  Publish();
}

void GazeboBall::CreatePubsAndSubs() {
  gazebo::transport::PublisherPtr gz_connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>("~/" + kConnectGazeboToRosSubtopic, 1);
  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;

  // ============================================ //
  //  ACTUAL POSITION OF THE BALL (GAZEBO->ROS)  //
  // ============================================ //
  ball_position_pub_ =
      node_handle_->Advertise<gz_geometry_msgs::Vector3dStamped>("~/" + namespace_ + "/" + ball_pos_pub_topic_, 1);
  // ball_position_pub_ = node_handle_->Advertise<gz_std_msgs::Float32>("~/" + namespace_ + "/" + ball_pos_pub_topic_,
  // 1);
  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" + ball_pos_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" + ball_pos_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(gz_std_msgs::ConnectGazeboToRosTopic::VECTOR_3D_STAMPED);
  gz_connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg, true);
}

void GazeboBall::Publish() {
  common::Time now = model_->GetWorld()->SimTime();

  position_msg_.mutable_header()->mutable_stamp()->set_sec(now.sec);
  position_msg_.mutable_header()->mutable_stamp()->set_nsec(now.nsec);
  position_msg_.mutable_header()->set_frame_id("");
  position_msg_.mutable_position()->set_x(model_->GetLink("ball")->WorldPose().Pos().X());
  position_msg_.mutable_position()->set_y(model_->GetLink("ball")->WorldPose().Pos().Y());
  position_msg_.mutable_position()->set_z(model_->GetLink("ball")->WorldPose().Pos().Z());

  ball_position_pub_->Publish(position_msg_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboBall);
}  // namespace gazebo
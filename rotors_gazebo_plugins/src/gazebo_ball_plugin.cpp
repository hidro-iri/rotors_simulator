#include "rotors_gazebo_plugins/gazebo_ball_plugin.h"

namespace gazebo {

GazeboBall::GazeboBall() : ModelPlugin() {}

GazeboBall::~GazeboBall() {}

void GazeboBall::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;

  getSdfParam<std::string>(_sdf, "ballNamespace", namespace_, "ball");
  getSdfParam<std::string>(_sdf, "positionPubTopic", pos_pub_topic_, "pos");
  getSdfParam<std::string>(_sdf, "velocityPubTopic", vel_pub_topic_, "vel");
  getSdfParam<std::string>(_sdf, "throwBallSubTopic", throw_sub_topic_, "throw");
  getSdfParam<ignition::math::Vector3d>(_sdf, "initialPosition", pos_0_, ignition::math::Vector3d(0, 0, 0));
  getSdfParam<ignition::math::Vector3d>(_sdf, "initialVelocity", vel_0_, ignition::math::Vector3d(0, 0, 0));
  getSdfParam<std::string>(_sdf, "parentFrameId", parent_frame_id_, "world");
  getSdfParam<std::string>(_sdf, "childFrameId", child_frame_id_, "ball");

  node_handle_ = gazebo::transport::NodePtr(new transport::Node());
  // Initialise with default namespace (typically /gazebo/default/)
  node_handle_->Init();

  pubs_and_subs_created_ = false;
  thrown_ = false;

  update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboBall::OnUpdate, this));
}

void GazeboBall::OnUpdate() {
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

  gazebo::transport::PublisherPtr gz_connect_ros_to_gazebo_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectRosToGazeboTopic>("~/" + kConnectRosToGazeboSubtopic, 1);
  gz_std_msgs::ConnectRosToGazeboTopic connect_ros_to_gazebo_topic_msg;

  // ============================================ //
  //  ACTUAL POSITION OF THE BALL (GAZEBO->ROS)  //
  // ============================================ //
  pos_pub_ = node_handle_->Advertise<gz_geometry_msgs::Vector3dStamped>("~/" + namespace_ + "/" + pos_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" + pos_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" + pos_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(gz_std_msgs::ConnectGazeboToRosTopic::VECTOR_3D_STAMPED);
  gz_connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg, true);

  // ============================================ //
  //  ACTUAL VELOCITY OF THE BALL (GAZEBO->ROS)  //
  // ============================================ //
  vel_pub_ = node_handle_->Advertise<gz_geometry_msgs::Vector3dStamped>("~/" + namespace_ + "/" + vel_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" + vel_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" + vel_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(gz_std_msgs::ConnectGazeboToRosTopic::VECTOR_3D_STAMPED);
  gz_connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg, true);

  // ============================================ //
  // = THROW BALL (ROS->GAZEBO) = //
  // ============================================ //

  throw_sub_ =
      node_handle_->Subscribe("~/" + namespace_ + "/" + throw_sub_topic_, &GazeboBall::CallbackThrowBall, this);

  connect_ros_to_gazebo_topic_msg.set_ros_topic(namespace_ + "/" + throw_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" + throw_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_msgtype(gz_std_msgs::ConnectRosToGazeboTopic::EXTERNAL_TRIGGER);
  gz_connect_ros_to_gazebo_topic_pub->Publish(connect_ros_to_gazebo_topic_msg, true);

  // ============================================ //
  // ======== TRANSFORM STAMPED MSG SETUP ======= //
  // ============================================ //
  broadcast_transform_pub_ =
      node_handle_->Advertise<gz_geometry_msgs::TransformStampedWithFrameIds>("~/" + kBroadcastTransformSubtopic, 1);
}

void GazeboBall::Publish() {
  common::Time now = model_->GetWorld()->SimTime();

  pos_msg_.mutable_header()->mutable_stamp()->set_sec(now.sec);
  pos_msg_.mutable_header()->mutable_stamp()->set_nsec(now.nsec);
  pos_msg_.mutable_header()->set_frame_id("world");
  pos_msg_.mutable_position()->set_x(model_->GetLink("ball")->WorldPose().Pos().X());
  pos_msg_.mutable_position()->set_y(model_->GetLink("ball")->WorldPose().Pos().Y());
  pos_msg_.mutable_position()->set_z(model_->GetLink("ball")->WorldPose().Pos().Z());

  vel_msg_.mutable_header()->mutable_stamp()->set_sec(now.sec);
  vel_msg_.mutable_header()->mutable_stamp()->set_nsec(now.nsec);
  vel_msg_.mutable_header()->set_frame_id("");
  vel_msg_.mutable_position()->set_x(model_->GetLink("ball")->WorldLinearVel().X());
  vel_msg_.mutable_position()->set_y(model_->GetLink("ball")->WorldLinearVel().Y());
  vel_msg_.mutable_position()->set_z(model_->GetLink("ball")->WorldLinearVel().Z());

  gz_geometry_msgs::TransformStampedWithFrameIds transform_stamped_with_frame_ids_msg;
  transform_stamped_with_frame_ids_msg.mutable_header()->CopyFrom(pos_msg_.header());
  transform_stamped_with_frame_ids_msg.mutable_transform()->mutable_translation()->set_x(
      model_->GetLink("ball")->WorldPose().Pos().X());
  transform_stamped_with_frame_ids_msg.mutable_transform()->mutable_translation()->set_y(
      model_->GetLink("ball")->WorldPose().Pos().Y());
  transform_stamped_with_frame_ids_msg.mutable_transform()->mutable_translation()->set_z(
      model_->GetLink("ball")->WorldPose().Pos().Z());

  gazebo::msgs::Quaternion q_W_L;
  q_W_L.set_w(1);
  q_W_L.set_x(0);
  q_W_L.set_y(0);
  q_W_L.set_z(0);
  transform_stamped_with_frame_ids_msg.mutable_transform()->mutable_rotation()->CopyFrom(q_W_L);
  transform_stamped_with_frame_ids_msg.set_parent_frame_id(parent_frame_id_);
  transform_stamped_with_frame_ids_msg.set_child_frame_id(child_frame_id_);

  pos_pub_->Publish(pos_msg_);
  vel_pub_->Publish(vel_msg_);
  broadcast_transform_pub_->Publish(transform_stamped_with_frame_ids_msg);
}

void GazeboBall::CallbackThrowBall(GzExternalTriggerMsgPtr& external_trigger_input_msg) {
  if (!thrown_ && external_trigger_input_msg->data()) {
    std::cout << pos_0_ << std::endl;
    model_->SetLinkWorldPose(ignition::math::Pose3d(pos_0_, ignition::math::Quaterniond(0, 0, 0)), "ball");
    model_->SetLinearVel(vel_0_);
  }
  thrown_ = external_trigger_input_msg->data();
}

GZ_REGISTER_MODEL_PLUGIN(GazeboBall);
}  // namespace gazebo
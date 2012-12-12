#ifndef _CAT_USER_ENTITY_H_
#define _CAT_USER_ENTITY_H_

#include <cat_user_entity/tf_scenegraph_object.h>
#include <cat_user_entity/camera_node.h>
#include <cat_user_entity/manipulator_node.h>

#include <ros/ros.h>
#include <eigen3/Eigen/Geometry>

namespace something {

typedef tf::Vector3 Vector3;
typedef tf::Quaternion Quaternion;
typedef tf::Transform Transform;

class UserEntity: public tf::SceneGraphNode {


public:

  // Methods only!

  // Constructor
  // Must be defined in the header due to library issues in CHAI3D.
  UserEntity(const std::string& tf_parent_frame_id, const std::string& frame_prefix,
             tf::TransformListener* tfl, tf::TransformBroadcaster* tfb, ros::Publisher* pub_markers)
    : SceneGraphNode(frame_prefix + "frame", tfl, tfb, pub_markers),
      right_(0),
      left_(0),
      view_(0),
      prefix_(frame_prefix),
      grabbing_(false)
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    double period = 0.01;
    pnh.param("period", period, 0.01);
    std::string device = "";
    pnh.param("device", device, std::string("hydra"));

    update_timer_ = nh.createTimer(ros::Duration(period), boost::bind(&UserEntity::update, this));
    changeParentFrameId(tf_parent_frame_id);

    init(device);
  }

  virtual ~UserEntity();

  // Must be defined in the header due to library issues in CHAI3D.
  void init(const std::string &device)
  {
    printf("Initializing user entity!\n");

    view_ = new something::CameraNode(prefix_ + "camera", tfl_, tfb_);
    view_->setPosition(tf::Vector3(-1.5, 0, 0.5)); // default
    addChild(view_);

    if(device == "haptic")
    {
      view_->setPosition(tf::Vector3(-1, 0, 0));
      right_ = new something::ManipulatorNode(prefix_ + "right_workspace", tfl_, tfb_, something::ManipulatorNode::HAPTIC);
      right_->setPosition(tf::Vector3(0, 0, 0));
      addChild(right_);
    }
    else if(device == "hydra")
    {
      view_->setPosition(tf::Vector3(-1.5, 0, 0.5));

      right_ = new something::ManipulatorNode(prefix_ + "right_workspace", tfl_, tfb_, something::ManipulatorNode::HYDRA_RIGHT);
      right_->setPosition(tf::Vector3(0, 0, 0));
      addChild(right_);

      left_ =  new something::ManipulatorNode(prefix_ + "left_workspace", tfl_, tfb_, something::ManipulatorNode::HYDRA_LEFT);
      left_->setPosition(tf::Vector3(0, 0, 0));
      addChild(left_);
    }



    grab_start_world_to_handle_.setIdentity();

    //printChildren(true);
    ROS_INFO("Done! Here we go...");
  }

  void attachCoupling();

  void update();

  void changeParentFrameId(const std::string &parent_id);

  // Moves the ENTIRE user entity
  //void moveUserViewFrame();

  bool getGrabState();

  void updateClutch();

protected:
    // Methods

protected:
    // Members

    // The transform from workspace to handle when we started moving the world
    Transform clutch_start_transform_;

    tf::StampedTransform grab_start_world_to_handle_;

    // Is there any good reason to support N widgets?
    something::ManipulatorNode *right_, *left_;
    something::CameraNode *view_;

    std::string prefix_;

    ros::Timer update_timer_;

    bool grabbing_;

    float user_workspace_separation_;
};

}  // namespace something

#endif

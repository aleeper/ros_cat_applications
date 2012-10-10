#ifndef _CAT_ABSTRACT_INTERACTION_TOOL_H_
#define _CAT_ABSTRACT_INTERACTION_TOOL_H_


#include <cat_user_entity/tf_scenegraph_object.h>
#include <cat_user_entity/abstract_handle.h>

#include <Eigen/Geometry>

namespace something {

typedef tf::Vector3 Vector3;
typedef tf::Quaternion Quaternion;
typedef tf::Transform Transform;

class AbstractInteractionTool: public tf::SceneGraphNode{

public:
  // Methods only!

  // Constructor
    // Constructor
      AbstractInteractionTool(const std::string &frame_id,
                 tf::TransformListener *tfl, tf::TransformBroadcaster *tfb)
          : SceneGraphNode(frame_id, tfl, tfb),
            handle_(0),
            k_linear_(0),
            k_angular_(0)
    {
        init();
    }

    virtual ~AbstractInteractionTool()
    {
          if(handle_) delete handle_;
    }

    void init()
    {
        handle_ = new something::AbstractHandle(transform_.child_frame_id_ + "_handle", tfl_, tfb_);
        addChild(handle_);
    }


    const something::AbstractHandle* getHandle()
    {
        return handle_;
    }


    // Read the state of the binary switches on the tool.
    bool getToolButtonState(const unsigned int &index) const
    {
        if(index >= getToolButtonCount()) return false;
        return button_state_[index];
    }

    // Get the number of buttons available on the tool.
    unsigned int getToolButtonCount() const
    {
        return (unsigned int)button_state_.size();
    }

    // Set the force applied to the tool
    virtual void setToolForce(const Vector3 &force)           { last_tool_force_ = force; }

    // Set the torque applied to the tool
    virtual void setToolTorque(const Vector3 &torque)         { last_tool_torque_ = torque; }

    // Set the force and torque applied to the tool
    virtual void setToolForceAndTorque(const Vector3 &force, const Vector3 &torque) { setToolForce(force); setToolTorque(torque); }

//    // Set the gripper force on the tool
//    virtual void setToolGripperForce(const float &force);

    virtual void attachHandleToTfFrame(const std::string& attached_frame_id, const tf::Pose& attached_frame_pose)
    {
      attached_frame_id_ = attached_frame_id;
      attached_frame_T_grasp_ = attached_frame_pose;
    }


protected: 
// Methods

    void setToolButtonCount(const unsigned int &count)
    {
        button_state_.resize(count, false);
    }

    void setToolButtonState(const size_t &index, const bool &state)
    {
        if(index >= button_state_.size())
        {
            ROS_ERROR("Can't set button %zd state, max size is %zd", index, button_state_.size());
            return;
        }
        button_state_[index] = state;
    }

    virtual void updateVirtualCoupling();

    virtual void drawSelf(const ros::Time now, visualization_msgs::MarkerArray& array);

//    virtual void updateDevice()
//    {
//      // Here is where, for example, we:
//      // readDevicePosition();
//      // getAttachedFramePosition();
//      // computeVirtualCouplingForce();
//      // sendDeviceForce();
//    }


// Members

  something::AbstractHandle *handle_;

  Vector3 last_tool_force_;
  Vector3 last_tool_torque_;

  std::string attached_frame_id_;
  tf::Pose attached_frame_T_grasp_;

  float k_linear_;
  float k_angular_;
  std::vector<bool> button_state_;

};

}  // namespace something

#endif

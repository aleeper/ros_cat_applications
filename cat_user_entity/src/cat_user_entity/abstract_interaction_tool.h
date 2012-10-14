#ifndef _CAT_ABSTRACT_INTERACTION_TOOL_H_
#define _CAT_ABSTRACT_INTERACTION_TOOL_H_


#include <cat_user_entity/tf_scenegraph_object.h>
#include <cat_user_entity/abstract_handle.h>
#include <interaction_cursor_msgs/InteractionCursorUpdate.h>
#include <interaction_cursor_msgs/InteractionCursorFeedback.h>


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
            attached_(false),
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
        ros::NodeHandle nh;
        std::string side = "";
        if(transform_.child_frame_id_.find("right") != std::string::npos) side = "_right";
        else if(transform_.child_frame_id_.find("left") != std::string::npos) side = "_left";

        std::string base_topic = std::string("interaction_cursor") + side;
        subscribe_cursor_ = nh.subscribe<interaction_cursor_msgs::InteractionCursorFeedback>(base_topic + "/feedback", 10,
                                           boost::bind( &AbstractInteractionTool::receiveInteractionCursorFeedback, this, _1 ) );
        publish_cursor_ = nh.advertise<interaction_cursor_msgs::InteractionCursorUpdate>(base_topic + "/update", 10);
    }


    const something::AbstractHandle* getHandle()
    {
        return handle_;
    }


    // Read the state of the binary switches on the tool.
    bool getToolButtonState(const unsigned int &index) const
    {
        if(index >= getToolButtonCount()) return false;
        return button_state_[index] && !attached_; // TODO this will break camera movement...
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

    virtual void timerUpdate();


protected: 
// Methods

    void receiveInteractionCursorFeedback(const interaction_cursor_msgs::InteractionCursorFeedbackConstPtr& icf_cptr)
    {

      switch(icf_cptr->event_type)
      {

      case interaction_cursor_msgs::InteractionCursorFeedback::NONE:
        attached_frame_id_ = icf_cptr->pose.header.frame_id;
        tf::poseMsgToTF(icf_cptr->pose.pose, attached_frame_T_grasp_);
        attachment_type_ = icf_cptr->attachment_type;
        attached_ = false;
        break;
      case interaction_cursor_msgs::InteractionCursorFeedback::GRABBED:
        attached_frame_id_ = icf_cptr->pose.header.frame_id;
        tf::poseMsgToTF(icf_cptr->pose.pose, attached_frame_T_grasp_);
        attachment_type_ = icf_cptr->attachment_type;
      case interaction_cursor_msgs::InteractionCursorFeedback::KEEP_ALIVE:
        attached_ = true;
        break;
      case interaction_cursor_msgs::InteractionCursorFeedback::RELEASED:
      case interaction_cursor_msgs::InteractionCursorFeedback::LOST_GRASP:
        attached_ = false;
        break;
      default:
        break;
      }

    }

    // Used to initialize the button storage
    void setToolButtonCount(const unsigned int &count)
    {
        button_state_.resize(count, false);
        button_transition_.resize(count, LOW);
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

    virtual void recordButtonTransitions();

//    virtual void updateDevice()
//    {
//      // Here is where, for example, we:
//      // readDevicePosition();
//      // getAttachedFramePosition();
//      // computeVirtualCouplingForce();
//      // sendDeviceForce();
//    }


// Members

  ros::Publisher  publish_cursor_;
  ros::Subscriber subscribe_cursor_;

  something::AbstractHandle *handle_;

  Vector3 last_tool_force_;
  Vector3 last_tool_torque_;

  std::string attached_frame_id_;
  tf::Pose attached_frame_T_grasp_;
  bool attached_;
  uint8_t attachment_type_;
//  enum AttachmentType{
//    NONE = interaction_cursor_msgs::InteractionCursorFeedback::NONE,
//    POSITION = interaction_cursor_msgs::InteractionCursorFeedback::POSITION,
//    ORIENTATION = interaction_cursor_msgs::InteractionCursorFeedback::ORIENTATION,
//    POSITION_AND_ORIENTATION = interaction_cursor_msgs::InteractionCursorFeedback::POSITION_AND_ORIENTATION
//  } attachment_type_;

  enum buttonTransition {
    LOW = 0,
    HIGH = 1,
    HIGH_TO_LOW = 2,
    LOW_TO_HIGH = 3
  };

  float k_linear_;
  float k_angular_;
  std::vector<bool> button_state_;
  std::vector<buttonTransition> button_transition_;
  std::map<std::string, unsigned int> button_name_map_;


};

}  // namespace something

#endif
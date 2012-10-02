
#ifndef _CAT_MANIPULATOR_NODE_H_
#define _CAT_MANIPULATOR_NODE_H_

#include <cat_user_entity/abstract_interaction_tool.h>
#include <cat_user_entity/haptic_interaction_tool.h>
#include <cat_user_entity/hydra_interaction_tool.h>
#include <cat_user_entity/tf_scenegraph_object.h>

#include <Eigen/Geometry>


namespace something {

typedef tf::Vector3 Vector3;
typedef tf::Quaternion Quaternion;
typedef tf::Transform Transform;

class ManipulatorNode: public tf::SceneGraphNode {


public:

  enum DeviceType {
    NONE = 0,
    HAPTIC,
    INTERACTIVE_MARKER,
    HYDRA_LEFT,
    HYDRA_RIGHT
  };

  // Methods only!

  // Constructor
  // Must be defined in the header due to library issues in CHAI3D.
  ManipulatorNode(const std::string &frame_id,
               tf::TransformListener *tfl, tf::TransformBroadcaster *tfb, DeviceType device_type)
    : SceneGraphNode(frame_id, tfl, tfb),
      device_type_(device_type),
      tool_(0)
  {
      init();
  }

  virtual ~ManipulatorNode()
  {
      if(tool_) delete tool_;
  }

  // Must be defined in the header due to library issues in CHAI3D.
  void init()
  {

    switch (device_type_)
    {
    case(HYDRA_RIGHT):
    {
      tool_ = new something::HydraInteractionTool(transform_.child_frame_id_ + "_device", tfl_, tfb_, something::HydraInteractionTool::HYDRA_RIGHT);
      break;
    }
    case(HYDRA_LEFT):
    {
      tool_ = new something::HydraInteractionTool(transform_.child_frame_id_ + "_device", tfl_, tfb_, something::HydraInteractionTool::HYDRA_LEFT);
      break;
    }
    case(HAPTIC):
    {
      tool_ = new something::HapticInteractionTool(transform_.child_frame_id_ + "_device", tfl_, tfb_);
      break;
    }
    case(INTERACTIVE_MARKER):
    {
      //tool_ = new something::InteractiveMarkerInteractionTool(transform_.child_frame_id_ + "_device", tfl_, tfb_);
      break;
    }
    default:
      break;
    } // switch

    if(tool_)
      addChild(tool_);
    else
      ROS_ERROR("Constructing manipulator node with no tool type; this isn't supported!");

      button_name_map_["grab"] = 0;
  }

    // Get button stuff should go here, so user can query it?

    bool isGrabbing()
    {
      if( !tool_ ) return false;
      unsigned int index = button_name_map_["grab"];
      return tool_->getToolButtonState(index);
    }


protected:
    // Methods

protected:
    // Members

    DeviceType device_type_;
    something::AbstractInteractionTool *tool_;
    std::map<std::string, unsigned int> button_name_map_;
};

}  // namespace something

#endif

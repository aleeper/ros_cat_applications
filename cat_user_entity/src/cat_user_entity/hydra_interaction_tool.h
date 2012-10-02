#ifndef _CAT_HYDRA_INTERACTION_TOOL_H_
#define _CAT_HYDRA_INTERACTION_TOOL_H_

#include <cat_user_entity/abstract_interaction_tool.h>
#include <razer_hydra/Hydra.h>
#include <ros/ros.h>

namespace something {

class HydraInteractionTool: public AbstractInteractionTool{


public:

  enum PaddleSide {
    HYDRA_RIGHT = 27,
    HYDRA_LEFT
  };

  // Methods only!

    HydraInteractionTool(const std::string &frame_id,
                                                 tf::TransformListener *tfl,
                                                 tf::TransformBroadcaster *tfb, PaddleSide side)
        : AbstractInteractionTool(frame_id, tfl, tfb),
          workspace_radius_(0.5),
          paddle_side_(side)
    {
        // Finish other intialization stuff.
        init();
    }

    virtual ~HydraInteractionTool();

    void init();


protected:
// Methods

    void updateFromMsg(const razer_hydra::HydraConstPtr &calib);

// Members
    float workspace_radius_;
    ros::Subscriber hydra_sub_;

    PaddleSide paddle_side_;

};


}  // namespace something

#endif

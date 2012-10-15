#ifndef _CAT_HYDRA_INTERACTION_TOOL_H_
#define _CAT_HYDRA_INTERACTION_TOOL_H_

#include <cat_user_entity/abstract_interaction_tool.h>
#include <razer_hydra/Hydra.h>
#include <ros/ros.h>

namespace something {

class HydraInteractionTool: public AbstractInteractionTool{


public:

  enum PaddleSide {
    HYDRA_LEFT = 0,
    HYDRA_RIGHT = 1
  };

  // Methods only!

    HydraInteractionTool(const std::string &frame_id,
                                                 tf::TransformListener *tfl,
                                                 tf::TransformBroadcaster *tfb, PaddleSide side)
        : AbstractInteractionTool(frame_id, tfl, tfb),
          workspace_radius_(0.5),
          paddle_side_(side),
          paddle_index_(0)
    {
        // Finish other intialization stuff.
        init();
    }

    virtual ~HydraInteractionTool();

    void init();


    void setPaddleSide(HydraInteractionTool::PaddleSide side);

protected:
// Methods

    void updatePaddleIndex();

    void updateFromMsg(const razer_hydra::HydraConstPtr &calib);

// Members
    float workspace_radius_;
    ros::Subscriber hydra_sub_;

    PaddleSide paddle_side_;
    unsigned int paddle_index_;

};


}  // namespace something

#endif

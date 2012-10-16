
#include <cat_user_entity/hydra_interaction_tool.h>
#include <razer_hydra/Hydra.h>

#include <tf/tf.h>
#include <Eigen/Geometry>
#include <ros/ros.h>



namespace something {


HydraInteractionTool::~HydraInteractionTool()
{
}


void HydraInteractionTool::init()
{

//    setPosition(tf::Vector3(pos.x(), pos.y(), pos.z()));
//    setQuaternion(tf::createQuaternionFromYaw(M_PI)*tf::createQuaternionFromRPY(0, 0.4, 0));

    ros::NodeHandle nh;
    hydra_sub_ = nh.subscribe<razer_hydra::Hydra>("hydra_calib", 1, boost::bind(&HydraInteractionTool::updateFromMsg, this, _1));

    setToolButtonCount(7);
    // TODO this should come from a config file!
    button_name_map_["click"] = 0;
    button_name_map_["menu"] = 1;

    k_linear_ = 1;
    k_angular_ = 1;

    updatePaddleIndex();
}

void HydraInteractionTool::setPaddleSide(HydraInteractionTool::PaddleSide side)
{
  paddle_side_ = side;
  updatePaddleIndex();
}

/////////////////////////////////////////////////////////////////////
// PROTECTED FUNCTIONS LIVE UNDER HERE
/////////////////////////////////////////////////////////////////////

void HydraInteractionTool::updatePaddleIndex()
{
  if(paddle_side_ == HYDRA_RIGHT) paddle_index_ = razer_hydra::Hydra::RIGHT;
  else if(paddle_side_ == HYDRA_LEFT) paddle_index_ = razer_hydra::Hydra::LEFT;
}

void HydraInteractionTool::updateFromMsg(const razer_hydra::HydraConstPtr &calib)
{
    ROS_DEBUG_NAMED("hydra", "Got hydra update!");

    razer_hydra::HydraPaddle paddle = calib->paddles[paddle_index_];

    // Update pose info
    tf::Transform interaction_handle;
    tf::transformMsgToTF(paddle.transform, interaction_handle);
    handle_->setTransform(interaction_handle);

    // Update button info
    if(getToolButtonCount() != paddle.buttons.size())
        setToolButtonCount(paddle.buttons.size());
    for(size_t i = 0; i < getToolButtonCount(); ++i)
    {
        setToolButtonState(i, paddle.buttons[i]);
    }
}



}  // namespace something


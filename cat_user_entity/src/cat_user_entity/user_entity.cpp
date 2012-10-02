
#include <cat_user_entity/user_entity.h>



namespace something {


UserEntity::~UserEntity()
{
    if(right_)  delete right_;
    if(left_)   delete left_;
    if(view_)   delete view_;
}

void UserEntity::attachCoupling()
{

}

void UserEntity::update()
{
  updateClutch();

  ros::Time now = ros::Time::now();
  publishTransformTree(now);

  publishMarkers(true);
}

void UserEntity::changeParentFrameId(const std::string &parent_id)
{
  transform_.frame_id_ = parent_id;
}


bool UserEntity::getGrabState()
{
  return right_->isGrabbing();
}

void  UserEntity::updateClutch()
{
  // detect a rising edge on the clutch button
  if (!grabbing_ && getGrabState())
  {
    grabbing_ = true;
    tfl_->lookupTransform(getParentFrameId(), "user1_right_workspace_device_handle", ros::Time(0), grab_start_world_to_handle_);
  }
  // detect a falling edge on the clutch button
  else if (grabbing_ && !getGrabState())
  {
    grabbing_ = false;
  }
  // set the resulting clutched position and orientation of the device
  if (grabbing_)
  {
    tf::StampedTransform st_handle_to_user;
    tfl_->lookupTransform("user1_right_workspace_device_handle", getFrameId(), ros::Time(0), st_handle_to_user);
    tf::Transform world_to_user;
    tf::Transform world_to_handle(grab_start_world_to_handle_.getRotation(), grab_start_world_to_handle_.getOrigin());
    tf::Transform handle_to_user(st_handle_to_user.getRotation(), st_handle_to_user.getOrigin());
    world_to_user = world_to_handle * handle_to_user;

    setTransform(world_to_user);
  }
}

} // namespace

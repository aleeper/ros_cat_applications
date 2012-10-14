
#include <cat_user_entity/abstract_interaction_tool.h>

namespace something {

void AbstractInteractionTool::timerUpdate()
{
  recordButtonTransitions();

  interaction_cursor_msgs::InteractionCursorUpdate update;

  tf::poseTFToMsg(handle_->getTransform(), update.pose.pose);
  update.pose.header.frame_id = handle_->getTransform().frame_id_;
  update.pose.header.stamp = handle_->getTransform().stamp_;

  // process buttons
  if(button_transition_[button_name_map_["click"]] == HIGH)        update.button_state = interaction_cursor_msgs::InteractionCursorUpdate::KEEP_ALIVE;
  if(button_transition_[button_name_map_["click"]] == LOW_TO_HIGH)
  {
    update.button_state = interaction_cursor_msgs::InteractionCursorUpdate::GRAB;
    attached_ = true;
  }
  if(button_transition_[button_name_map_["click"]] == HIGH_TO_LOW) update.button_state = interaction_cursor_msgs::InteractionCursorUpdate::RELEASE;
  if(button_transition_[button_name_map_["click"]] == LOW)         update.button_state = interaction_cursor_msgs::InteractionCursorUpdate::NONE;
  if(button_transition_[button_name_map_["menu"]] == LOW_TO_HIGH)  update.button_state = interaction_cursor_msgs::InteractionCursorUpdate::QUERY_MENU;

  ROS_INFO("Sending cursor update! [%d]", update.button_state);
  publish_cursor_.publish(update);
}


void AbstractInteractionTool::updateVirtualCoupling()
{
  // Skip this if we aren't grabbing anything...
  if(attached_frame_id_ == "") return;

  // Make sure we are doing all calculations in the tool (e.g. device) frame.
  tf::StampedTransform tool_T_handle_stamped = handle_->getTransform();
  tf::StampedTransform tool_T_attached_stamped;
  tfl_->lookupTransform(getFrameId(), attached_frame_id_, ros::Time(0), tool_T_attached_stamped);

  tf::Transform tool_T_handle = tool_T_handle_stamped;
  tf::Transform tool_T_attached = tool_T_attached_stamped;
  tf::Transform tool_T_grasp = tool_T_attached * attached_frame_T_grasp_;

  // Should be at current location of grasp point, but we measure current location of handle.
  tf::Vector3 position_handle_to_grasp_point = tool_T_grasp.getOrigin() - tool_T_handle.getOrigin();

  // Get rotation from handle frame to grasp frame
  tf::Quaternion quaternion_handle_to_grasp = tool_T_handle.getRotation().inverse()*tool_T_grasp.getRotation();
  tf::Vector3 angle_axis_handle_to_grasp = quaternion_handle_to_grasp.getAngle()*quaternion_handle_to_grasp.getAxis();


  tf::Vector3 force = k_linear_*position_handle_to_grasp_point;
  tf::Vector3 torque = k_angular_*angle_axis_handle_to_grasp;

  setToolForce(force);
  setToolTorque(torque);
}

void AbstractInteractionTool::drawSelf(const ros::Time now, visualization_msgs::MarkerArray& array)
{


  tf::Vector3 tail_position = handle_->getPosition();
  tf::Vector3 tip_position = tail_position + last_tool_force_;

  if( (tip_position - tail_position).length() > 0.00001 )
  {
    // add markers for forces
    visualization_msgs::Marker marker;
    marker.header.frame_id = getFrameId();
    marker.header.stamp = now;
    //tf::Transform T = getTransform();
    //tf::pointTFToMsg(T.getOrigin(), marker.pose.position);
    //tf::quaternionTFToMsg(T.getRotation(), marker.pose.orientation);

    marker.action = marker.ADD;
    marker.ns = getFrameId();
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.2;
    marker.color.a = 0.7;


    geometry_msgs::Point tail_point, tip_point;
    tf::pointTFToMsg(tail_position, tail_point);
    tf::pointTFToMsg(tip_position, tip_point);

    marker.type = marker.ARROW;
    marker.points.push_back(tail_point);
    marker.points.push_back(tip_point);
    marker.scale.x = 0.05;
    marker.scale.y = 0.1;

    array.markers.push_back(marker);
  }
}

void AbstractInteractionTool::recordButtonTransitions()
{
  if(button_state_.size() != button_transition_.size())
  {
    ROS_ERROR("Button state and button transition vector are not same size, aborting!");
    return;
  }

  for(size_t i = 0; i < button_state_.size(); i++)
  {
    bool now = button_state_[i];
    bool previous = (button_transition_[i] == HIGH) || (button_transition_[i] == LOW_TO_HIGH);
    if(  now && !previous )  button_transition_[i] = LOW_TO_HIGH;
    if(  now &&  previous )  button_transition_[i] = HIGH;
    if( !now &&  previous )  button_transition_[i] = HIGH_TO_LOW;
    if( !now && !previous )  button_transition_[i] = LOW;
  }
}


} // namespace

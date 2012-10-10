
#include <cat_user_entity/abstract_interaction_tool.h>

namespace something {

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

  tf::Vector3 tail_position = handle_->getPosition();
  tf::Vector3 tip_position = tail_position + last_tool_force_;

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


} // namespace

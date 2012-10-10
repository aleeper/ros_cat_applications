
#include <cat_user_entity/abstract_handle.h>


namespace something {

void AbstractHandle::drawSelf(const ros::Time now, visualization_msgs::MarkerArray& array)
{
  // add markers handle
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


  marker.type = marker.ARROW;
  marker.scale.x = 1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  array.markers.push_back(marker);
}


}  // namespace

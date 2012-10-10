#ifndef _TF_SCENEGRAPH_OBJECT_H_
#define _TF_SCENEGRAPH_OBJECT_H_


#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>


namespace tf {

class SceneGraphNode{


public:
  // Methods

    SceneGraphNode(const std::string &frame_id, tf::TransformListener *tfl, tf::TransformBroadcaster *tfb, ros::Publisher* pub_markers = 0);

    virtual ~SceneGraphNode();

    virtual void setPosition(const tf::Vector3 &position);
    virtual void setQuaternion(const tf::Quaternion &quaternion);
    virtual void setTransform(const tf::Transform &transform);

    virtual tf::Vector3                 getPosition() const;
    virtual tf::Quaternion              getQuaternion() const;
    virtual const tf::StampedTransform& getTransform() const;

    tf::SceneGraphNode* accessChild(const std::string &key);

    void addChild(tf::SceneGraphNode *node);

    bool removeChild(tf::SceneGraphNode *node);

    bool removeChild(const std::string &key);

    void printChildren(const bool &recursive = false);

    std::string getFrameId();

    std::string getParentFrameId();

    void publishTransformTree(const ros::Time now);

    void addTransformsToVector(const ros::Time now, std::vector<tf::StampedTransform> &transforms);

    void addMarkersToArray(const ros::Time now, visualization_msgs::MarkerArray& array);

    // Derived classes should override this to add markers to the array in order to draw themselves.
    virtual void drawSelf(const ros::Time now, visualization_msgs::MarkerArray& array);

    virtual void publishMarkers( const bool &recursive);

protected:
    // Methods
    void setParent(tf::SceneGraphNode* const parent);

    void setParentFrameId(const std::string &parent_id);

  // Members
  tf::StampedTransform transform_;

  tf::TransformListener *tfl_;
  tf::TransformBroadcaster *tfb_;
  ros::Publisher* pub_markers_;

  tf::SceneGraphNode *parent_;
  std::map<std::string, tf::SceneGraphNode*> children_;

};


}  // namespace something

#endif // header

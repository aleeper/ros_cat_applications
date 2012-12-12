
#include <chai3d_conversions/tf_chai.h>


namespace tf
{

tf::Vector3 vectorChaiToTf(const cVector3d &c)
{
  tf::Vector3 t;
  t.setX(c.x());
  t.setY(c.y());
  t.setZ(c.z());
  return t;
}

cVector3d vectorTfToChai(const tf::Vector3 &t)
{
  return cVector3d(t.x(), t.y(), t.z());
}

tf::Matrix3x3 matrixChaiToTf(const cMatrix3d &rot)
{
    return tf::Matrix3x3( rot.getCol0().x(), rot.getCol1().x(), rot.getCol2().x(),
                          rot.getCol0().y(), rot.getCol1().y(), rot.getCol2().y(),
                          rot.getCol0().z(), rot.getCol1().z(), rot.getCol2().z());
}

tf::Quaternion quaternionChaiMatrixToTf(const cMatrix3d &m)
{
  tf::Quaternion q;
  matrixChaiToTf(m).getRotation(q);
  return q;
}

} // namespace chai_tools


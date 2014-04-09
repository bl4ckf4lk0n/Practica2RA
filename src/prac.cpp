#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("reconstruction", 1);	
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0, 0, 0.0) );
  transform.setRotation( tf::Quaternion(0, 0, 0) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "frameMapa", "base_link"));

cloud->header.frame_id="frameMapa";
pub.publish (*cloud);


  /*ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1);

  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = "some_tf_frame";
  msg->height = msg->width = 1;
  msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));

  ros::Rate loop_rate(4);
  while (nh.ok())
  {
    msg->header.stamp = ros::Time::now ();
    pub.publish (msg);
    ros::spinOnce ();
    loop_rate.sleep ();
  }*/
}
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <boost/foreach.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/nonfree/nonfree.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <pcl/visualization/pcl_visualizer.h>

using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
const float* depthImageFloat;
cv_bridge::CvImagePtr imageColormsg;
bool depthreceived;
bool imagereceived;


class Prac2 {
  ros::Subscriber imgSub;
  ros::Subscriber depthSub;
  pcl::visualization::PCLVisualizer::Ptr viewer;//objeto viewer

  public:Prac2(ros::NodeHandle& nh) { 

    imgSub = nh.subscribe("/camera/rgb/image_color", 1, &Prac2::imageCb, this);
    depthSub = nh.subscribe("/camera/depth/image", 1, &Prac2::imageCbdepth, this);

    viewer= pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters ();
  };


  void bucle() {
    ros::Rate rate(1); 
    while (ros::ok()) {
      ros::spinOnce(); // Se procesarán todas las llamadas pendientes, es decir, llamará a callBack
      rate.sleep(); // Espera a que finalice el ciclo
    }
  };

  void imageCbdepth(const sensor_msgs::ImageConstPtr& msg)
  {
    std::cerr<<" depthcb: "<<msg->header.frame_id<<" : "<<msg->header.seq<<" : "<<msg->header.stamp<<std::endl;
    depthImageFloat = reinterpret_cast<const float*>(&msg->data[0]);
    depthreceived=true;
    if(imagereceived && depthreceived)
    processRegistration();

  }


  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      imageColormsg = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    std::cerr<<" imagecb: "<<msg->header.frame_id<<" : "<<msg->header.seq<<" : "<<msg->header.stamp<<std::endl;
    imagereceived=true;

    if(imagereceived && depthreceived)
      processRegistration();
  }

  void processRegistration()
  {
    ros::NodeHandle nh;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p = getCloudfromColorAndDepth(imageColormsg->image, depthImageFloat);
    ros::Publisher pub = nh.advertise<PointCloud> ("reconstruction", 1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0, 0, 0.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "frameMapa", "base_link"));

    cloud->header.frame_id="frameMapa";
    pub.publish (*cloud);

    /*Visualizar*/
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(p);   //esto es el manejador de color de la nube "cloud"

    if (!viewer->updatePointCloud (p,rgb, "cloud")) //intento actualizar la nube y si no existe la creo.
      viewer->addPointCloud(p,rgb,"cloud");

    while (!viewer->wasStopped())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (10));
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloudfromColorAndDepth(const cv::Mat imageColor, const float* depthImage)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
    cloud->height = 480;
    cloud->width = 640;
    cloud->is_dense = false;

    cloud->points.resize(cloud->height * cloud->width);

    register float constant = 0.0019047619;
    cloud->header.frame_id = "/openni_rgb_optical_frame";

    register int centerX = (cloud->width >> 1);
    int centerY = (cloud->height >> 1);

    float bad_point = std::numeric_limits<float>::quiet_NaN();

    register int depth_idx = 0;
    int i,j;
    for (int v = -centerY,j=0; v < centerY; ++v,++j)
    {
      for (register int u = -centerX,i=0; u < centerX; ++u, ++depth_idx,++i)
      {
        pcl::PointXYZRGB& pt = cloud->points[depth_idx];

        float depthimagevalue=depthImage[depth_idx];

        if (depthimagevalue == 0)
        {
          // not valid
          pt.x = pt.y = pt.z = bad_point;
          continue;
        }
        pt.z = depthimagevalue;
        pt.x = u * pt.z * constant;
        pt.y = v * pt.z * constant;

        const Point3_<uchar>* p = imageColor.ptr<Point3_<uchar> >(j,i);
        pt.r=p->z;
        pt.g=p->y;
        pt.b=p->x;
      }
    }
    return cloud;
  }

};


int main(int argc, char **argv) {
  ros::init(argc, argv, "prac2"); // Inicializa un nuevo nodo llamado wander
  ros::NodeHandle nh;
  Prac2 prac2(nh); // Crea un objeto de esta clase y lo asocia con roscore
  prac2.bucle(); // Ejecuta el bucle
  return 0;
};


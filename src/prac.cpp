#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/*void imageCbdepth(const sensor_msgs::ImageConstPtr& msg)
  {
    depthmsg=msg;
    std::cerr<<" depthcb: "<<msg->header.frame_id<<" : "<<msg->header.seq<<" : "<<msg->header.stamp<<std::endl;
    //depthImagemsg = cv_bridge::toCvCopy(msg, enc::TYPE_32FC1);

    depthreceived=true;
    if(imagereceived && depthreceived)
   	 //processRegistration();

}*/

void callback(const PointCloud::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

pcl::PointXYZ getXYZfromXYDEPTH(int x, int y,const uint16_t* depthImage){
  register float constant = 0.0019047619;
  register int centerX = (640 >> 1);
  int centerY = (480 >> 1);

  float bad_point = std::numeric_limits<float>::quiet_NaN();

  int depth_idx = y*640+x;
    pcl::PointXYZ pt;

      if (depthImage[depth_idx] == 0)
    {
        // not valid
        pt.x = pt.y = pt.z = bad_point;
 
      }else{
    pt.z = depthImage[depth_idx] * 0.001;
    pt.x = (x-centerX) * pt.z * constant;
    pt.y = (y-centerY) * pt.z * constant;
      }
    //std::cerr<<x<<" "<<y<<": "<<depthImage[depth_idx]<<": "<<pt.x <<" "<<pt.y<<" "<<pt.z<<std::endl;
    return pt;

}

void callbackPCL(const sensor_msgs::ImageConstPtr& depthmsg){
	//calculo la depth de la imagen depth
    const uint16_t* depthImage = reinterpret_cast<const uint16_t*>(&depthmsg->data[0]);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);

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
          pcl::PointXYZ& pt = cloud->points[depth_idx];
          // Check for invalid measurements
          if (depthImage[depth_idx] == 0)
        {
        // not valid
        pt.x = pt.y = pt.z = bad_point;
        continue;
          }
          pt.z = depthImage[depth_idx] * 0.001;
          pt.x = u * pt.z * constant;
          pt.y = v * pt.z * constant;
          //std::cerr<<u<<" "<<v<<": "<<depthImage[depth_idx]<<": "<<pt.x <<" "<<pt.y<<" "<<pt.z<<std::endl;

        }
      }
}

/*void imageCb(const sensor_msgs::ImageConstPtr& msg)
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
}*/

int main(int argc, char** argv){
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("points2", 1, callback);
  ros::spin();
}

int publicar(int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud> ("reconstruction", 1);	
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
  
  //para hacer no se qué del rviz
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0, 0, 0.0) );
  transform.setRotation( tf::Quaternion(0, 0, 0) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "frameMapa", "base_link"));

  //publicar
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
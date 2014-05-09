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
#include <pcl/features/normal_3d.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/pfh.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/ransac.h>


using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

const float* depthImageFloat;
cv_bridge::CvImagePtr imageColormsg;
bool depthreceived;
bool imagereceived;
const float* depthImageFloat1;
cv_bridge::CvImagePtr imageColormsg1;
bool depthreceived1;
bool imagereceived1;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src (new pcl::PointCloud<pcl::PointXYZ>); 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt (new pcl::PointCloud<pcl::PointXYZ>); 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZ>); 



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

    

    boost::shared_ptr<pcl::visualization::PCLVisualizer> MView (new pcl::visualization::PCLVisualizer ("Aligning")); 
    MView->initCameraParameters (); 
        //View-Port1 
    int v1(0); 
    MView->createViewPort (0.0, 0.0, 0.5, 1.0, v1); 
    MView->setBackgroundColor (0, 0, 0, v1); 
    MView->addText ("Start:View-Port 1", 10, 10, "v1_text", v1); 
                            //PointCloud Farben...verschieben vor v1? 
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green (cloud_src, 0,255,0); 
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (cloud_tgt, 255,0,0); 

    MView->addPointCloud (cloud_src, green, "source", v1); 
    MView->addPointCloud (cloud_tgt, red, "target", v1); 
                            //View-Port2 
    int v2(0); 
    MView->createViewPort (0.5, 0.0, 1.0, 1.0, v2); 
    MView->setBackgroundColor (0, 0, 0, v2); 
    MView->addText ("Aligned:View-Port 2", 10, 10, "v2_text", v2); 

                            //MView->addPointCloud (cloud_tgt, red, "target2", v2); 
                            //MView->addPointCloud (cloud_src, green, "source2", v2); 
                    //Properties for al viewports 
    MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source"); 
    MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target"); 

    while (!viewer->wasStopped())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (10));
    }
}

/*void processRegistration2(){

    //HAY QUE QUITAR NARF Y UTILIZAR SIFT. Hay que separarlo por funciones y probar que funciona
    //Igualmente da un error de compilacion porque esta deprecated la funcion en la que da

   //   ros::NodeHandle nh;
    
    
    // ros::Publisher pub = nh.advertise<PointCloud> ("reconstruction", 1);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);

    // static tf::TransformBroadcaster br;
 //     tf::Transform transform;
 //     transform.setOrigin( tf::Vector3(0, 0, 0.0) );
 //     transform.setRotation( tf::Quaternion(0, 0, 0) );
 //     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "frameMapa", "base_link"));

    // cloud->header.frame_id="frameMapa";
    // pub.publish (*cloud);

 //  pcl::visualization::PCLVisualizer::Ptr viewer;//objeto viewer
 //  viewer= pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer ("3D Viewer"));

 //  viewer->setBackgroundColor (0, 0, 0);

 //  //viewer->addCoordinateSystem (1.0); //podriamos dibujar un sistema de coordenadas si quisieramos

 //  viewer->initCameraParameters ();
 //  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_src);   //esto es el manejador de color de la nube "cloud"

 //  if (!viewer->updatePointCloud (cloud_src,rgb, "cloud")) //intento actualizar la nube y si no existe la creo.
 //    viewer->addPointCloud(cloud_src,rgb,"cloud");

 //  while (!viewer->wasStopped())
 //  {
 //    viewer->spinOnce (100);
 //    boost::this_thread::sleep (boost::posix_time::microseconds (10));
 //  }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> MView (new pcl::visualization::PCLVisualizer ("Aligning")); 
    MView->initCameraParameters (); 
    //View-Port1 
    int v1(0); 
    MView->createViewPort (0.0, 0.0, 0.5, 1.0, v1); 
    MView->setBackgroundColor (0, 0, 0, v1); 
    MView->addText ("Start:View-Port 1", 10, 10, "v1_text", v1); 
                        //PointCloud Farben...verschieben vor v1? 
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green (cloud_src, 0,255,0); 
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (cloud_tgt, 255,0,0); 

    MView->addPointCloud (cloud_src, green, "source", v1); 
    MView->addPointCloud (cloud_tgt, red, "target", v1); 
                        //View-Port2 
    int v2(0); 
    MView->createViewPort (0.5, 0.0, 1.0, 1.0, v2); 
    MView->setBackgroundColor (0, 0, 0, v2); 
    MView->addText ("Aligned:View-Port 2", 10, 10, "v2_text", v2); 
    
                        //MView->addPointCloud (cloud_tgt, red, "target2", v2); 
                        //MView->addPointCloud (cloud_src, green, "source2", v2); 
                //Properties for al viewports 
    MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source"); 
    MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target"); 
                        //MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target2");    
    
        //remove NAN-Points 
    std::vector<int> indices1,indices2; 
    pcl::removeNaNFromPointCloud (*cloud_src, *cloud_src, indices1); 
    pcl::removeNaNFromPointCloud (*cloud_tgt, *cloud_tgt, indices2); 
        //Downsampling 
    PCL_INFO ("Downsampling \n"); 
                //temp clouds src & tgt 
    pcl::PointCloud<pcl::PointXYZ>::Ptr ds_src (new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr ds_tgt (new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::VoxelGrid<pcl::PointXYZ> grid; 
    grid.setLeafSize (0.05, 0.05, 0.05); 
    grid.setInputCloud (cloud_src); 
    grid.filter (*ds_src); 
    grid.setInputCloud (cloud_tgt); 
    grid.filter (*ds_tgt);  
    PCL_INFO (" Downsampling finished \n"); 

        // Normal-Estimation 
    PCL_INFO ("Normal Estimation \n"); 
    pcl::PointCloud<pcl::Normal>::Ptr norm_src (new pcl::PointCloud<pcl::Normal>); 
    pcl::PointCloud<pcl::Normal>::Ptr norm_tgt (new pcl::PointCloud<pcl::Normal>); 
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree<pcl::PointXYZ>()); 
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree<pcl::PointXYZ>()); 
    
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne; 
                //Source-Cloud 
    PCL_INFO (" Normal Estimation - Source \n");    
    ne.setInputCloud (ds_src); 
    ne.setSearchSurface (cloud_src); 
    ne.setSearchMethod (tree_src); 
    ne.setRadiusSearch (0.05); 
    ne.compute (*norm_src); 

                //Target-Cloud 
    PCL_INFO (" Normal Estimation - Target \n"); 
    ne.setInputCloud (ds_tgt); 
    ne.setSearchSurface (cloud_tgt); 
    ne.setSearchMethod (tree_tgt); 
    ne.setRadiusSearch (0.05); 
    ne.compute (*norm_tgt); 

        // Keypoints _ NARF 
    PCL_INFO ("NARF - Keypoint \n"); 
                //Probleme mit boost daher erstmal ohne 
    pcl::RangeImage range_src; 
    pcl::RangeImage range_tgt; 

                //Header-information for Range Image 
                float angularResolution = (float) (  0.2f * (M_PI/180.0f));  //   0.5 degree in radians 
                float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians 
                float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians 
                Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f); 
                pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME; 
                float noiseLevel = 0.00; 
                float minRange = 0.0f; 
                int borderSize = 1; 

                //Konvertieren der Point-Cloud in ein Range-Image d.h Cloud mit Range-Value und 3D-Koordinate 
                PCL_INFO (" NARF - Creating Range-Images\n"); 
                range_src.createFromPointCloud (*cloud_src, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize); 
                range_tgt.createFromPointCloud (*cloud_tgt, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);  

                //Visualisierung Range-Images 
                pcl::visualization::RangeImageVisualizer range_vis1 ("Range Image"); 
                range_vis1.showRangeImage (range_src); 
                pcl::visualization::RangeImageVisualizer range_vis2 ("Range Image"); 
                range_vis2.showRangeImage (range_tgt); 

                //Extracting NARF-Keypoints 
                pcl::RangeImageBorderExtractor range_image_ba; 
                float support_size = 0.2f; //? 
                        //Keypoints Source 
                pcl::NarfKeypoint narf_keypoint_src (&range_image_ba); 
                narf_keypoint_src.setRangeImage (&range_src); 
                narf_keypoint_src.getParameters ().support_size = support_size; 
                pcl::PointCloud<int> keypoints_ind_src; 
                PCL_INFO (" NARF - Compute Keypoints - Source\n"); 
                narf_keypoint_src.compute (keypoints_ind_src); 
                PCL_INFO (" NARF - Found %d Keypoints in Source\n", keypoints_ind_src.size()); 
                        //Keypoints Target 
                pcl::NarfKeypoint narf_keypoint_tgt (&range_image_ba); 
                narf_keypoint_tgt.setRangeImage (&range_tgt); 
                narf_keypoint_tgt.getParameters ().support_size = support_size; 
                pcl::PointCloud<int> keypoints_ind_tgt; 
                PCL_INFO (" NARF - Compute Keypoints - Target\n"); 
                narf_keypoint_tgt.compute (keypoints_ind_tgt); 
                PCL_INFO (" NARF - Found %d Keypoints in Target\n", keypoints_ind_tgt.size()); 
        //Umwandlung Keypoints von pcl::PointCloud<int> zu pcl::PointCloud<XYZ>
                pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_src (new pcl::PointCloud<pcl::PointXYZ>); 
                pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_tgt (new pcl::PointCloud<pcl::PointXYZ>); 
                
                keypoints_src->width = keypoints_ind_src.points.size(); 
                keypoints_src->height = 1; 
                keypoints_src->is_dense = false; 
                keypoints_src->points.resize (keypoints_src->width * keypoints_src->height);    
                
                keypoints_tgt->width = keypoints_ind_tgt.points.size(); 
                keypoints_tgt->height = 1; 
                keypoints_tgt->is_dense = false; 
                keypoints_tgt->points.resize (keypoints_tgt->width * keypoints_tgt->height); 
                
                int ind_count=0; 
                
                //source XYZ-CLoud  
                for (size_t i = 0; i < keypoints_ind_src.points.size(); i++) 
                {   
                    ind_count = keypoints_ind_src.points[i]; 
                        //float x = range_src.points[ind_count].x; 
                    
                    keypoints_src->points[i].x = range_src.points[ind_count].x; 
                    keypoints_src->points[i].y = range_src.points[ind_count].y; 
                    keypoints_src->points[i].z = range_src.points[ind_count].z; 
                } 

                //target XYZ-Cloud 
                for (size_t i = 0; i < keypoints_ind_tgt.points.size(); i++) 
                {   
                    ind_count = keypoints_ind_tgt.points[i]; 
                        //float x = range_src.points[ind_count].x; 
                    
                    keypoints_tgt->points[i].x = range_tgt.points[ind_count].x; 
                    keypoints_tgt->points[i].y = range_tgt.points[ind_count].y; 
                    keypoints_tgt->points[i].z = range_tgt.points[ind_count].z; 
                } 

                
        //Feature-Descriptor 
                PCL_INFO ("FPFH - Feature Descriptor\n"); 
                //FPFH  
                        //FPFH Source 
                        //pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est_src; 
                        //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_fpfh_src (new pcl::search::KdTree<pcl::PointXYZ>); 
                        //fpfh_est_src.setSearchSurface (ds_src);//<-------------- Use All Points for analyzing  the local structure of the cloud 
                        //fpfh_est_src.setInputCloud (keypoints_src); //<------------- But only compute features at the key-points 
                        //fpfh_est_src.setInputNormals (norm_src); 
                        //fpfh_est_src.setRadiusSearch (0.05);  
                        //pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src (new pcl::PointCloud<pcl::FPFHSignature33>); 
                        //PCL_INFO ("   FPFH - Compute Source\n"); 
                        //fpfh_est_src.compute (*fpfh_src); 
                
                        ////FPFH Target 
                        //pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est_tgt; 
                        //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_fpfh_tgt (new pcl::search::KdTree<pcl::PointXYZ>); 
                        //fpfh_est_src.setSearchSurface (ds_tgt); 
                        //fpfh_est_tgt.setInputCloud (keypoints_tgt); 
                        //fpfh_est_tgt.setInputNormals (norm_tgt); 
                        //fpfh_est_tgt.setRadiusSearch (0.05); 
                        //pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt (new pcl::PointCloud<pcl::FPFHSignature33>); 
                        //PCL_INFO ("   FPFH - Compute Target\n"); 
                        //fpfh_est_tgt.compute (*fpfh_tgt); 
                        //PCL_INFO ("   FPFH - finished\n"); 
                //PFH-Source 
                PCL_INFO ("PFH - started\n"); 
                pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_est_src; 
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_pfh_src (new pcl::search::KdTree<pcl::PointXYZ>()); 
                pfh_est_src.setSearchMethod (tree_pfh_src); 
                pfh_est_src.setRadiusSearch (0.1); 
                pfh_est_src.setSearchSurface (ds_src);   
                pfh_est_src.setInputNormals (norm_src); 
                pfh_est_src.setInputCloud (keypoints_src); 
                pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_src (new pcl::PointCloud<pcl::PFHSignature125>); 
                PCL_INFO (" PFH - Compute Source\n"); 
                pfh_est_src.compute (*pfh_src); 
                PCL_INFO (" PFH - finished\n"); 
                //PFH-Target 
                pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_est_tgt; 
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_pfh_tgt (new pcl::search::KdTree<pcl::PointXYZ>()); 
                pfh_est_tgt.setSearchMethod (tree_pfh_tgt); 
                pfh_est_tgt.setRadiusSearch (0.1); 
                pfh_est_tgt.setSearchSurface (ds_tgt);   
                pfh_est_tgt.setInputNormals (norm_tgt); 
                pfh_est_tgt.setInputCloud (keypoints_tgt); 
                pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_tgt (new pcl::PointCloud<pcl::PFHSignature125>); 
                PCL_INFO (" PFH - Compute Target\n"); 
                pfh_est_tgt.compute (*pfh_tgt); 
                PCL_INFO (" PFH - finished\n"); 

        //Correspondence Estimation 
                PCL_INFO ("Correspondence Estimation\n"); 
                pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125, pcl::PFHSignature125> corEst; 
                corEst.setInputCloud (pfh_src); 
                corEst.setInputTarget (pfh_tgt); 
                PCL_INFO (" Correspondence Estimation - Estimate C.\n"); 
                //pcl::Correspondences cor_all; 
                //Pointer erzeugen 
                boost::shared_ptr<pcl::Correspondences> cor_all_ptr (new pcl::Correspondences); 
                corEst.determineCorrespondences (*cor_all_ptr); 
                PCL_INFO (" Correspondence Estimation - Found %d Correspondences\n", cor_all_ptr->size());  
                
        //Reject false Correspondences 
                Eigen::Matrix4f transformation; 
                
                PCL_INFO ("Correspondence Rejection Features\n"); 

                //SAC 
                double epsilon_sac = 0.1; // 10cm 
                int iter_sac = 10000; 
                pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> sac; 
                //pcl::registration::corres 
                sac.setInputCloud (cloud_src); 
                sac.setTargetCloud (cloud_tgt); 
                sac.setInlierThreshold (epsilon_sac); 
                sac.setMaxIterations (iter_sac); 
                sac.setInputCorrespondences (cor_all_ptr); 

                boost::shared_ptr<pcl::Correspondences> cor_inliers_ptr (new pcl::Correspondences); 
                sac.getCorrespondences (*cor_inliers_ptr); 
                //int sac_size = cor_inliers_ptr->size(); 
                PCL_INFO (" RANSAC: %d Correspondences Remaining\n", cor_inliers_ptr->size ()); 

                transformation = sac.getBestTransformation(); 
                
                //pcl::registration::trans 
                //pcl::registration::TransformationEstimation<pcl::PointXYZ, pcl::PointXYZ>::Ptr trans_est (new pcl::registration::TransformationEstimation<pcl::PointXYZ, pcl::PointXYZ>); 
                //pcl:: 
                
                //Punktwolke Transformieren 
                pcl::transformPointCloud (*cloud_src, *cloud_tmp, transformation); 
                MView->addPointCloud (cloud_tmp, green, "tmp", v2); 
                MView->addPointCloud (cloud_tgt, red, "target_2", v2); 
                MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "tmp"); 
                MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_2"); 
                
                //Zeitdifferenz 
                end = time(0); 
                diff = difftime (end, start); 
                PCL_INFO ("\nDuration: %d", diff); 
                
        // Warten bis Viewer geschlossen wird 
                while (!MView->wasStopped()) 
                { 
                    MView->spinOnce(100); 
                } 

            }*/

            

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

      void imageCbdepth(const sensor_msgs::ImageConstPtr& msg)
      {
        std::cerr<<" depthcb: "<<msg->header.frame_id<<" : "<<msg->header.seq<<" : "<<msg->header.stamp<<std::endl;
        if(cloud_src==NULL){
         depthImageFloat = reinterpret_cast<const float*>(&msg->data[0]);
         depthreceived=true;
     } else{
         depthImageFloat1 = reinterpret_cast<const float*>(&msg->data[0]);
         depthreceived1=true;
     }

     depthreceived=true;
     if(imagereceived && depthreceived)
         if(cloud_src==NULL)
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src = getCloudfromColorAndDepth(imageColormsg->image, depthImageFloat);
      else if(cloud_tgt == NULL){
         if(imagereceived1 && depthreceived1)
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt = getCloudfromColorAndDepth(imageColormsg1->image, depthImageFloat1);
  } else {
     processRegistration();
 }

}


void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		if(cloud_src==NULL){
         imageColormsg = cv_bridge::toCvCopy(msg, enc::BGR8);
         imagereceived=true;
     }
     else{
         imageColormsg1 = cv_bridge::toCvCopy(msg, enc::BGR8);
         imagereceived1=true;
     }
 }
 catch (cv_bridge::Exception& e)
 {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
}

std::cerr<<" imagecb: "<<msg->header.frame_id<<" : "<<msg->header.seq<<" : "<<msg->header.stamp<<std::endl;


if(imagereceived && depthreceived)
 if(cloud_src==NULL)
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src = getCloudfromColorAndDepth(imageColormsg->image, depthImageFloat);
else if(cloud_tgt == NULL){
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt = getCloudfromColorAndDepth(imageColormsg1->image, depthImageFloat1);
} else {
 processRegistration();
}
}


void callback(const PointCloud::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);

  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
  printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

/*void detectKeypoints (const PointCloud::Ptr pcl_cloud, float min_scale, int nr_octaves, int nr_scales_per_octave)
{
	pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift_detect;
	sift_detect.setSearchMethod(pcl::KdTreeFLANN<PointT>::Ptr (new pcl::KdTreeFLANN<PointT>));
	sift_detect.setScales (min_scale, nr_octaves, nr_scales_per_octave); 
	sift_detect.setMinimumContrast (min_contrast); 
	sift_detect.setInputCloud (points);
	pcl::PointCloud<pcl::PointWithScale> keypoints_temp;
	sift_detect.compute (keypoints_temp);
	PointCloudPtr keypoints (new PointCloud); 
	pcl::copyPointCloud (keypoints_temp, *keypoints);
	//return (keypoints); 

	//ESTO PETAAAAAA
	PointCloud::Ptr sift_cloud (new PointCloud);
	printf("Creating KD-tree\n");
  	// Create a KD-Tree (for the search method of sift object)
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	printf("a\n");
	tree->setInputCloud (pcl_cloud);

  	pcl::SIFTKeypoint<PointT, PointT> sift_object;

  	printf("Setting input cloud\n");
  	sift_object.setInputCloud(pcl_cloud);

  	printf("Setting search method\n");
  	sift_object.setSearchMethod(tree);

  	printf("Setting octaves\n");
  	sift_object.setScales(0.5, 8, 5); //random initialization

  	printf("computing sift features\n");
  	sift_object.compute(*sift_cloud);

  	printf("we found %d SIFT features in the cloud\n", sift_cloud->height*sift_cloud->width);
}*/


};

int main(int argc, char** argv){
    ros::init(argc, argv, "sub_pcl");
    ros::NodeHandle nh;
    Prac2 prac2(nh);
    prac2.bucle();
    ros::spin();
}
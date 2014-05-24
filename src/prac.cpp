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
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>


#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/keypoints/susan.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/harris_6d.h>
#include <pcl/keypoints/agast_2d.h>

#include <pcl/features/shot.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>


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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src (new pcl::PointCloud<pcl::PointXYZRGB>); 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt (new pcl::PointCloud<pcl::PointXYZRGB>); 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptor_tgt;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_tgt;
ros::Time headerStamp;
boost::shared_ptr<pcl::visualization::PCLVisualizer> MView;
int v2(0); 
int v1(0); 



class Prac2 {
    ros::Subscriber imgSub;
    ros::Subscriber depthSub;
    pcl::visualization::PCLVisualizer::Ptr viewer;//objeto viewer
    pcl::visualization::PCLVisualizer::Ptr viewer2;//objeto viewer

    public:Prac2(ros::NodeHandle& nh) {
        imgSub = nh.subscribe("/camera/rgb/image_color", 1, &Prac2::imageCb, this);
        depthSub = nh.subscribe("/camera/depth/image", 1, &Prac2::imageCbdepth, this);

        viewer= pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        viewer->initCameraParameters ();

         viewer2= pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer2->setBackgroundColor (0, 0, 0);
        viewer2->initCameraParameters ();
        MView = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("Aligning")); 

    };


    void bucle() {
        ros::Rate rate(1); 
        while (ros::ok()) {
          ros::spinOnce(); // Se procesarán todas las llamadas pendientes, es decir, llamará a callBack
          rate.sleep(); // Espera a que finalice el ciclo
        }
    };

    void visualizar(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2){
        boost::shared_ptr<pcl::visualization::PCLVisualizer> MView (new pcl::visualization::PCLVisualizer ("Aligning")); 
        MView->initCameraParameters(); 
        //View-Port1 
        int v1(0); 
        MView->createViewPort (0.0, 0.0, 0.5, 1.0, v1); 
        MView->setBackgroundColor (0, 0, 0, v1); 
        MView->addText ("Start:View-Port 1", 10, 10, "v1_text", v1); 
                            //PointCloud Farben...verschieben vor v1? 
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green (cloud1, 0,255,0); 
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red (cloud2, 255,0,0); 

        MView->addPointCloud (cloud1, green, "source", v1); 
        MView->addPointCloud (cloud2, red, "target", v1); 
                            //View-Port2 
        int v2(0); 
        MView->createViewPort (0.5, 0.0, 1.0, 1.0, v2); 
        MView->setBackgroundColor (0, 0, 0, v2); 
        MView->addText ("Aligned:View-Port 2", 10, 10, "v2_text", v2); 
        
        //MView->addPointCloud (cloud2, red, "target2", v2); 
        //MView->addPointCloud (cloud1, green, "source2", v2); 
        //Properties for al viewports 
        MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source"); 
        MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target"); 
                            //MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target2");    
        while (!MView->wasStopped())
        {
            MView->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (10));
        }
    }

     pcl::PointCloud<pcl::PointXYZRGB>::Ptr reducirNube(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
        std::cerr<<"Reduciendo nube"<<std::endl;
        

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_src (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> grid; 
        float voxel_side_size=0.02; //2cm
        grid.setLeafSize (voxel_side_size, voxel_side_size, voxel_side_size);
        grid.setInputCloud (cloud); 
        grid.filter (*ds_src);
        return ds_src;
        //visualizar(ds_src, ds_tgt);
    }

    pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_src){
        PCL_INFO ("Normal Estimation \n"); 
        pcl::PointCloud<pcl::Normal>::Ptr norm_src (new pcl::PointCloud<pcl::Normal>);

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_src(new pcl::search::KdTree<pcl::PointXYZRGB>());

        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne; 
                //Source-Cloud 
        PCL_INFO (" Normal Estimation - Source \n");    
        ne.setInputCloud (ds_src); 
        ne.setSearchSurface (cloud); 
        ne.setSearchMethod (tree_src);
        ne.setRadiusSearch (0.05);
        ne.compute (*norm_src);

        return norm_src; 
    }


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractHarris3DKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
    {

      std::cerr<<"extracting keypoints with Harris3D"<<std::endl;
      pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI> detector; 
      //detector.setNonMaxSupression (true); 
  
  detector.setRadius (0.04);
  detector.setRadiusSearch (0.05); 

  detector.setInputCloud(cloud); 
  pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());

  detector.compute(*keypoints);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_src (new pcl::PointCloud<pcl::PointXYZRGB>); 
  keypoints_src->width = keypoints->points.size(); 
  keypoints_src->height = 1; 
  keypoints_src->is_dense = false; 
  keypoints_src->points.resize (keypoints_src->width * keypoints_src->height);
  //source XYZ-CLoud  
  for (size_t i = 0; i < keypoints->points.size(); i++) 
  {
    keypoints_src->points[i].x = keypoints->points[i].x; 
    keypoints_src->points[i].y = keypoints->points[i].y; 
    keypoints_src->points[i].z = keypoints->points[i].z;

    //falta meter lo de rgb
  } 
  return keypoints_src;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractHarris6DKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{

  std::cerr<<"extracting keypoints with Harris6D"<<std::endl;
  pcl::HarrisKeypoint6D<pcl::PointXYZRGB,pcl::PointXYZI> detector; 
  //detector.setNonMaxSupression (true); 
  detector.setRadius (0.02);    //Este parametro es pa tocarlo y bien
  detector.setRadiusSearch (0.03);
  detector.setInputCloud(cloud); 
  pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());

  detector.compute(*keypoints);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_src (new pcl::PointCloud<pcl::PointXYZRGB>); 
  keypoints_src->width = keypoints->points.size(); 
  keypoints_src->height = 1; 
  keypoints_src->is_dense = false; 
  keypoints_src->points.resize (keypoints_src->width * keypoints_src->height);
  //source XYZ-CLoud  
  for (size_t i = 0; i < keypoints->points.size(); i++) 
  {
    keypoints_src->points[i].x = keypoints->points[i].x; 
    keypoints_src->points[i].y = keypoints->points[i].y; 
    keypoints_src->points[i].z = keypoints->points[i].z;
  } 
  return keypoints_src;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractSUSANKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{

  std::cerr<<"extracting keypoints with SUSAN"<<std::endl;
  pcl::SUSANKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB>* susan3D = new pcl::SUSANKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB>;
  susan3D->setInputCloud(cloud);
  susan3D->setNonMaxSupression(true);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZRGB> ());
  susan3D->compute(*keypoints);
  return keypoints;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractSiftKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
  std::cerr<<"extracting keypoints"<<std::endl;
  const float min_scale = 0.005;
  const int nr_octaves = 4;
  const int nr_scales_per_octave = 5;
  const float min_contrast = 1;

  try{
    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;
            //search

    pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>::Ptr on(new pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>());
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());

    if(cloud_src->isOrganized() ){
     sift.setSearchMethod(on);
   }else{
     sift.setSearchMethod(tree);
   }

   pcl::PointCloud<pcl::PointWithScale>::Ptr sifts (new pcl::PointCloud<pcl::PointWithScale>);
   sift.setInputCloud(cloud);
            //sift.setSearchMethod (tree);
   sift.setScales(min_scale, nr_octaves, nr_scales_per_octave);
   sift.setMinimumContrast(min_contrast);
   sift.compute(*sifts);

   pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_src (new pcl::PointCloud<pcl::PointXYZRGB>); 
   keypoints_src->width = sifts->points.size(); 
   keypoints_src->height = 1; 
   keypoints_src->is_dense = false; 
   keypoints_src->points.resize (keypoints_src->width * keypoints_src->height);
            //source XYZ-CLoud  
   for (size_t i = 0; i < sifts->points.size(); i++) 
   {

    keypoints_src->points[i].x = sifts->points[i].x; 
    keypoints_src->points[i].y = sifts->points[i].y; 
    keypoints_src->points[i].z = sifts->points[i].z;

                //falta meter lo de rgb
  } 
  return keypoints_src;
} catch (Exception ex){
  throw ex;
}
}

pcl::PointCloud<pcl::SHOT1344>::Ptr createSHOTDescriptor( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
  pcl::PointCloud<pcl::Normal>::Ptr norm_src)
{
  // Setup the SHOT features
  //typedef pcl::FPFHSignature33 ShotFeature; // Can't use this, even despite: http://docs.pointclouds.org/trunk/structpcl_1_1_f_p_f_h_signature33.html
  PCL_INFO ("SHOT - started\n"); 
  pcl::SHOTColorEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> shotEstimation;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  shotEstimation.setInputCloud(cloud);
  shotEstimation.setInputNormals(norm_src);

  // Use the same KdTree from the normal estimation
  shotEstimation.setSearchMethod (tree);
  pcl::PointCloud<pcl::SHOT1344>::Ptr shotFeatures(new pcl::PointCloud<pcl::SHOT1344>);
  //spinImageEstimation.setRadiusSearch (0.2);
  //shotEstimation.setKSearch(10);
  shotEstimation.setKSearch(0);
  shotEstimation.setRadiusSearch(1);

  // Actually compute the spin images
  PCL_INFO (" SHOT - Compute Source\n"); 
  shotEstimation.compute (*shotFeatures);
  PCL_INFO (" SHOT - finished\n");
  return shotFeatures;
}

    pcl::PointCloud<pcl::PFHSignature125>::Ptr createDescriptor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr norm_src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_src){
         PCL_INFO ("PFH - started\n"); 
         pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh_est_src; 
         pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_pfh_src (new pcl::search::KdTree<pcl::PointXYZRGB>());
         //std::cerr<<cloud->size()<<std::endl;
         //std::cerr<<norm_src->size()<<std::endl;
         pfh_est_src.setSearchMethod (tree_pfh_src); 
         pfh_est_src.setRadiusSearch (0.5); 
         pfh_est_src.setSearchSurface (keypoints_src);   
         pfh_est_src.setInputNormals (norm_src); 
         pfh_est_src.setInputCloud (keypoints_src); 
         pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_src (new pcl::PointCloud<pcl::PFHSignature125>); 
         PCL_INFO (" PFH - Compute Source\n"); 
         pfh_est_src.compute (*pfh_src); 
         PCL_INFO (" PFH - finished\n");

         return pfh_src;
    }

    void processRegistration()
    {
        ros::NodeHandle nh;
        /*Visualizar*/
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_src);   //esto es el manejador de color de la nube "cloud"

        if (!viewer->updatePointCloud (cloud_src,rgb, "cloud")) //intento actualizar la nube y si no existe la creo.
            viewer->addPointCloud(cloud_src,rgb,"cloud");


        

        //visualizar(cloud_src, cloud_tgt);
        try{

            std::vector<int> indices1;
            cloud_src->is_dense = false;
            pcl::removeNaNFromPointCloud<pcl::PointXYZRGB>(*cloud_src, *cloud_src, indices1);

            //pcl::PointCloud<pcl::PointXYZRGB>::Ptr nube_src = reducirNube(cloud_src);


            //falta meter los puntos de rgb en esta funcion
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_src = extractSiftKeypoints(cloud_src);

            cout<<"culo "<<keypoints_src->size()<<endl;
            pcl::PointCloud<pcl::Normal>::Ptr normal_src = getNormals(keypoints_src, keypoints_src);

            //pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptor_src = createDescriptor(cloud_src, normal_src, keypoints_src);
            pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptor_src = createDescriptor(cloud_src, normal_src,keypoints_src );
            cout<<"ok"<<endl;
            if(!cloud_tgt->empty())
            {
                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(cloud_tgt);   //esto es el manejador de color de la nube "cloud"

                if (!viewer2->updatePointCloud (cloud_tgt,rgb2, "cloud2")) //intento actualizar la nube y si no existe la creo.
                    viewer2->addPointCloud(cloud_tgt,rgb2,"cloud2");

                /*Correspondencias */
                PCL_INFO ("Correspondence Estimation\n"); 
                pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125, pcl::PFHSignature125> corEst; 
                // corEst.setInputSource(descriptor_src); 
                // corEst.setInputTarget(descriptor_tgt); 
                corEst.setInputSource(descriptor_tgt); 
                corEst.setInputTarget(descriptor_src); 
                boost::shared_ptr<pcl::Correspondences> cor_all_ptr (new pcl::Correspondences);

                //cout<<"Tamaño vector: "<<cor_all_ptr->size()<<endl;
                //peta aqui
                //corEst.determineCorrespondences (*cor_all_ptr);
                corEst.determineReciprocalCorrespondences (*cor_all_ptr);

                Eigen::Matrix4f transformation; 
                PCL_INFO ("Correspondence Rejection Features\n"); 
                //SAC 
                double epsilon_sac = 0.1; // 10cm 
                int iter_sac = 10000;
                //pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (cloud_src));
                //pcl::RandomSampleConsensus<pcl::PointXYZRGB> sac (model_p); 
                pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> sac;
                sac.setInputSource (cloud_tgt); 
                sac.setInputTarget(cloud_src); 
                sac.setInlierThreshold (epsilon_sac); 
                sac.setMaximumIterations (iter_sac); 
                sac.setInputCorrespondences (cor_all_ptr); 

                boost::shared_ptr<pcl::Correspondences> cor_inliers_ptr (new pcl::Correspondences); 
                sac.getCorrespondences (*cor_inliers_ptr); 
                        //int sac_size = cor_inliers_ptr->size(); 
                PCL_INFO (" RANSAC: %d Correspondences Remaining\n", cor_inliers_ptr->size ()); 

                transformation = sac.getBestTransformation();


              // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
              pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::PFHSignature125> sac_ia_;
              sac_ia_.setMinSampleDistance (0.05f);
              sac_ia_.setMaxCorrespondenceDistance (0.01f);
              sac_ia_.setMaximumIterations (500);
              sac_ia_.setInputTarget (keypoints_src);
              sac_ia_.setTargetFeatures (descriptor_src);
              sac_ia_.setInputSource (keypoints_tgt);
              sac_ia_.setSourceFeatures (descriptor_tgt);
              pcl::PointCloud<pcl::PointXYZRGB> registration_output;
              sac_ia_.align (registration_output);
              transformation = sac_ia_.getFinalTransformation ();




                Eigen::Affine3f transTotal;

                ofstream myfile;

                //inicilialización en constructor

                transTotal.setIdentity();

                myfile.open ("traj_estimated.txt");

                //escribir en fichero la transformación estimada
                 transTotal=transTotal*transformation;
                 pcl::PointXYZRGB p0; //point at zero reference
                p0.x=0; p0.y=0; p0.z=0; p0.r=255; p0.g=0; p0.b=0;
                  pcl::PointXYZRGB pt_trans=pcl::transformPoint<pcl::PointXYZRGB>(p0,transTotal); //estimated position of the camera
                Eigen::Quaternion<float> rot2D( (transTotal).rotation());
                    myfile <<""<<headerStamp<<" "<<pt_trans.x<<" "<<pt_trans.y<<" "<<pt_trans.z<<" "<<rot2D.x()<<" "<<rot2D.y()<<" "<<rot2D.z()<<" "<<rot2D.w()<<std::endl;

                //destructor

                myfile.close();

                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green (cloud_src, 255,0,0); 
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red (cloud_tgt, 0,255,0);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::transformPointCloud (*cloud_tgt, *cloud_transformed,transformation);

                MView->updatePointCloud (cloud_transformed, green, "tmp"); 
                MView->updatePointCloud (cloud_src, red, "target_2"); 
                MView->updatePointCloud (cloud_src,red, "source");
                //MView->updatePointCloud (cloud_src,red, "source",v1);
                if (!MView->updatePointCloud (cloud_tgt,red, "target")){ //intento actualizar la nube y si no existe la creo.
                
                MView->initCameraParameters (); 
                //View-Port1 
                MView->createViewPort (0.0, 0.0, 0.5, 1.0, v1); 
                MView->setBackgroundColor (0, 0, 0, v1); 
                MView->addText ("Start:View-Port 1", 10, 10, "v1_text", v1); 
                                    //PointCloud Farben...verschieben vor v1? 

                //Cambiar rgb por red o green
                MView->addPointCloud (cloud_tgt, red, "target", v1); 
                MView->addPointCloud (cloud_src, green, "source", v1); 
                                    //View-Port2 
                
                MView->createViewPort (0.5, 0.0, 1.0, 1.0, v2); 
                MView->setBackgroundColor (0, 0, 0, v2); 
                MView->addText ("Aligned:View-Port 2", 10, 10, "v2_text", v2); 
                
                                    //MView->addPointCloud (cloud_tgt, red, "target2", v2); 
                                    //MView->addPointCloud (cloud_src, green, "source2", v2); 
                            //Properties for al viewports 
                MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source"); 
                MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target"); 
                                //MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target2");    
            
                //while (!viewer->wasStopped())
                //{
                    viewer->spinOnce (100);
                    boost::this_thread::sleep (boost::posix_time::microseconds (10));
                //}

                        //Punktwolke Transformieren 
                     //Cambiar rgb por red o green
                //pcl::transformPointCloud (*cloud_src, *cloud_tgt, transformation);


                pcl::transformPointCloud (*cloud_tgt, *cloud_transformed,transformation);

                MView->addPointCloud (cloud_transformed, green, "tmp", v2); 
                MView->addPointCloud (cloud_src, red, "target_2", v2); 
                MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "tmp"); 
                MView->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_2"); 

                }

                // Warten bis Viewer geschlossen wird 
                while (!MView->wasStopped()) 
                { 
                    MView->spinOnce(100); 
                } 
                MView->resetStoppedFlag();
            
          }
            cloud_tgt = cloud_src;
            descriptor_tgt = descriptor_src;
            keypoints_tgt = keypoints_src;
        } catch(Exception ex){

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

    void imageCbdepth(const sensor_msgs::ImageConstPtr& msg)
    {
      std::cerr<<" depthcb: "<<msg->header.frame_id<<" : "<<msg->header.seq<<" : "<<msg->header.stamp<<std::endl;
      if(!depthreceived){
        headerStamp = msg->header.stamp;
        depthImageFloat = reinterpret_cast<const float*>(&msg->data[0]);
        depthreceived=true;
      }

      while(!imagereceived && !depthreceived);

      if(imagereceived && depthreceived){
        cloud_src = getCloudfromColorAndDepth(imageColormsg->image, depthImageFloat);
        processRegistration();
        imagereceived = false;
        depthreceived = false;
      }
    }


    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
    	try
    	{
    		if(!imagereceived){
             imageColormsg = cv_bridge::toCvCopy(msg, enc::BGR8);
             imagereceived=true;
            }
        }
         catch (cv_bridge::Exception& e)
         {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        std::cerr<<" imagecb: "<<msg->header.frame_id<<" : "<<msg->header.seq<<" : "<<msg->header.stamp<<std::endl;

        while(!imagereceived && !depthreceived);


        /*if(imagereceived && depthreceived){
            cloud_src = getCloudfromColorAndDepth(imageColormsg->image, depthImageFloat);
            imagereceived = false;
            depthreceived = false;
            processRegistration();
        }*/
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
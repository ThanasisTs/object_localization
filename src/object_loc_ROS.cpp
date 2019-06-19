//ROS wrapper for object localization using the PCL library

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;


//Algorithm params
bool use_cloud_resolution_;
float model_ss_;
float scene_ss_;
float rf_rad_;
float descr_rad_;
float cg_size_;
float cg_thresh_;
float xmax_scn;
float xmin_scn;
float ymax_scn;
float ymin_scn;
std::string camera_frame;

ros::Publisher pub;

pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
void cut_scene(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloudPtr, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &scene);

//Object Localization Handler
void object_recon(const sensor_msgs::PointCloud2ConstPtr& cloud_ROS_msg)
{
  ros::NodeHandle n;

  pcl::PCLPointCloud2* scene_init = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr scenePtr(scene_init);
  pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr sceneInit (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
  pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());


  // Convert scene image to PoinCloud file
  pcl_conversions::toPCL(*cloud_ROS_msg, *scene_init);
  pcl::fromPCLPointCloud2(*scene_init, *sceneInit);

  // Preproccesing of the scene
  cut_scene(sceneInit, scene);


  //
  //  Compute Normals
  //
  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  norm_est.setKSearch (10);
  norm_est.setInputCloud (model);
  norm_est.compute (*model_normals);

  norm_est.setInputCloud (scene);
  norm_est.compute (*scene_normals);

  //
  //  Downsample Clouds using Uniform Sampling
  //

  pcl::UniformSampling<PointType> uniform_sampling;
  uniform_sampling.setInputCloud (model);
  uniform_sampling.setRadiusSearch (model_ss_);
  uniform_sampling.filter (*model_keypoints);
  std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

  uniform_sampling.setInputCloud (scene);
  uniform_sampling.setRadiusSearch (scene_ss_);
  uniform_sampling.filter (*scene_keypoints);
  std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;
  
  //
  //  Compute Descriptor for keypoints
  //
  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  descr_est.setRadiusSearch (descr_rad_);

  descr_est.setInputCloud (model_keypoints);
  descr_est.setInputNormals (model_normals);
  descr_est.setSearchSurface (model);
  descr_est.compute (*model_descriptors);

  descr_est.setInputCloud (scene_keypoints);
  descr_est.setInputNormals (scene_normals);
  descr_est.setSearchSurface (scene);
  descr_est.compute (*scene_descriptors);

  //
  //  Find Model-Scene Correspondences with KdTree
  //
  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud (model_descriptors);

  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
  for (size_t i = 0; i < scene_descriptors->size (); ++i)
  {
    std::vector<int> neigh_indices (1);
    std::vector<float> neigh_sqr_dists (1);
    if (!std::isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
    {
      continue;
    }
    int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    {
      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      model_scene_corrs->push_back (corr);
    }
  }
  std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

  //
  //  Actual Clustering
  //
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector<pcl::Correspondences> clustered_corrs;

  //
  //  Compute (Keypoints) Reference Frames only for Hough
  //
  pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
  pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

  pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
  rf_est.setFindHoles (true);
  rf_est.setRadiusSearch (rf_rad_);

  rf_est.setInputCloud (model_keypoints);
  rf_est.setInputNormals (model_normals);
  rf_est.setSearchSurface (model);
  rf_est.compute (*model_rf);

  rf_est.setInputCloud (scene_keypoints);
  rf_est.setInputNormals (scene_normals);
  rf_est.setSearchSurface (scene);
  rf_est.compute (*scene_rf);

  //  Clustering
  pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
  clusterer.setHoughBinSize (cg_size_);
  clusterer.setHoughThreshold (cg_thresh_);
  clusterer.setUseInterpolation (true);
  clusterer.setUseDistanceWeight (false);

  clusterer.setInputCloud (model_keypoints);
  clusterer.setInputRf (model_rf);
  clusterer.setSceneCloud (scene_keypoints);
  clusterer.setSceneRf (scene_rf);
  clusterer.setModelSceneCorrespondences (model_scene_corrs);

  clusterer.recognize (rototranslations, clustered_corrs);

  //
  //  Output results
  //
  std::cout << "Model instances found: " << rototranslations.size () << std::endl;

  for (size_t i = 0; i < rototranslations.size (); ++i)
  {
    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
    Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

    geometry_msgs::PointStamped camera_point;
    camera_point.header.frame_id = camera_frame;
    camera_point.header.stamp = cloud_ROS_msg->header.stamp;

    camera_point.point.x = translation(0);
    camera_point.point.y = translation(1);
    camera_point.point.z = translation(2);
    pub.publish(camera_point);
  }
}


void cut_scene(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloudPtr, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &scene){
  for (size_t i=0; i<cloudPtr->size(); ++i){
    if (cloudPtr->points[i].x < xmax_scn and cloudPtr->points[i].y < ymax_scn and cloudPtr->points[i].y > ymin_scn){
      scene->points.push_back(cloudPtr->points[i]);
    } 
  }
  scene->width = scene->points.size();
  scene->height = 1;
  scene->is_dense = cloudPtr->is_dense;
  scene->points.resize(scene->height*scene->width);
}


int main(int argc, char **argv){

  ros::init(argc, argv, "object_loc_ROS");
  ros::NodeHandle nh;
  

  //
  //  Load Model
  //
  std::string model_path; 
  nh.param("object_localization/model", model_path, std::string("/home/ttsitos/object_localization/milk.pcd"));
  pcl::io::loadPCDFile (model_path, *model);
  

  //
  //  Load Parameters
  //
  std::string camera_input;
  std::string output_topic;
  nh.param("object_localization/camera_frame", camera_frame, std::string("zed_left_camera_frame"));
  nh.param("object_localization/camera_input", camera_input, std::string("/zed/point_cloud/cloud_registered"));
  nh.param("object_localization/output_topic", output_topic, std::string("camera_pos"));
  nh.param("object_localization/model_ss_", model_ss_, float(0.01));
  nh.param("object_localization/scene_ss_", scene_ss_, float(0.01));
  nh.param("object_localization/rf_rad_", rf_rad_, float(0.015));
  nh.param("object_localization/descr_rad_", descr_rad_, float(0.02));
  nh.param("object_localization/cg_size_", cg_size_, float(0.01));
  nh.param("object_localization/cg_thresh_", cg_thresh_, float(5));
  nh.param("object_localization/xmax_scn", xmax_scn, float(1.5));
  nh.param("object_localization/xmin_scn", xmin_scn, float(0.0));
  nh.param("object_localization/ymax_scn", ymax_scn, float(0.5));
  nh.param("object_localization/ymin_scn", ymin_scn, float(-0.3));
  nh.param("object_localization/use_cloud_resolution_", use_cloud_resolution_, bool(false));

  //
  //  Initialize the Publisher
  //
  pub = nh.advertise<geometry_msgs::PointStamped> (output_topic, 1000);
  
  //
  //  Subcribe to topic
  //
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> (camera_input, 1, object_recon);
  ros::spin();
}
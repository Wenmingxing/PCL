#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>


int main(int argc,char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);

	//Fill in the cloud data
	pcl::PCDReader reader;
	reader.read("table_scene_lms400.pcd",*cloud_blob);
	
	std::cerr<<"pointcloud before filtering: "<<cloud_blob->width * cloud_blob->height<<"data points."<<std::endl;

	//Create the filtering object: downsample the dataset using a leaf size of 1 cm
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud_blob);
	sor.setLeafSize (0.01f,0.01f,0.01f);
	sor.filter(*cloud_filtered_blob);
	
	//Convert to the templated pointcloud
	pcl::fromPCLPointCloud2 (*cloud_filtered_blob,*cloud_filtered);
	
	std::cerr<<"Pointcloud after filtering: "<<cloud_filtered->width * cloud_filtered->height<<" data points ."<<std::endl;
	
	//Write the downsampled version to disk
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered,false);
	
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	
	//Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	//optional 
	seg.setOptimizeCoefficients(true);
	//Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.01);
	
	//Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	
	int i = 0, nr_points = (int) cloud_filtered->points.size();
	//While 30% of the original cloud is still there
	while (cloud_filtered->points.size() > 0.3 * nr_points)
	{
		//Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			std::cerr<<" cloud not estimate a planar model for the given dataset."<<std::endl;
			break;		
		}	
		
		//Extract te inliers
		extract.setInputCloud (cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative (false);
		extract.filter(*cloud_p);
		std::cerr<<"Pointcloud representing the plannar component:" <<cloud_p->width * cloud_p->height <<" data points."<<std::endl;
	
		std::stringstream ss;
		ss<<"table_scene_lms400_plane_"<<i<<".pcd";
		writer.write<pcl::PointXYZ> (ss.str(),*cloud_p,false);
		
		//Create the filtering object
		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloud_filtered.swap(cloud_f);	
		i++;
	}
	 /*pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  	 viewer.setBackgroundColor (1, 1, 1);
  	 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> range_image_color_handler (extract, 0, 0, 0);
 	 viewer.addPointCloud (extract, range_image_color_handler, "range image");
  	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
  //viewer.addCoordinateSystem (1.0f, "global");
  //PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
  //viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
  	viewer.initCameraParameters ();
	while(!viewer.wasStopped()){viewer.spinOnce();}*/
	return 0;	
}

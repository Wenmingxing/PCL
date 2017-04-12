#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	
	//Fill in th cloud 
	cloud->width = 5;
	cloud->height = 1;
	cloud->points.resize (cloud->width * cloud->height);
	
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);	
	}
		
	std::cerr<<"cloud before filtering: "<<std::endl;
	for (size_t i = 0; i < cloud->points.size();++i)
		std::cerr<<" "<< cloud->points[i].x<<" "<<cloud->points[i].y<<" "<<cloud->points[i].z<<std::endl;
	
	//Create a set of planar coefficients with x=y=0,z=1
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients ());
	coefficients->values.resize(4);
	coefficients->values[0] = coefficients->values[1]=0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;
	
	//Create the filtering object
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients (coefficients);
	proj.filter (*cloud_filtered);

	std::cerr<<"Cloud after filtering: "<<std::endl;
	
	for (size_t i=0;i<cloud_filtered->points.size();++i)
		std::cerr << " "<<cloud_filtered->points[i].x<<" "<<cloud_filtered->points[i].y<<" "<<cloud_filtered->points[i].z<<std::endl;
	
	return 0;
	
}

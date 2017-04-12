#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal> ());

	...read, pass in or create a point cloud with normals..
	...(note: you can create a single PointCloud<PointNormal> if you want)...

	//Create the PFH estimation class, and pass the input dataset+normals to it
	pcl::PFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::PFHSignature125> pfh;
	pfh.setInputCloud(cloud);
	pfh.setInputNormals(normals);
	
	//Alternatively, if cloud is of tpe PointNormal, fo pfh.setInputNormals(cloud);
	
	//Create an empty kdtree representation, and pass it to the PFH estimation object.
	//Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	
	//pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); -- older call for PCL 1.5.
	pfh.setSearchMethod(tree);	
	
	//Output datasets
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125> ());
	
	//use all neighbors in a sphere of radius 5cm
	//IMPORTANT : The radius used here has to be larger than the radius used to estimate the surface normals!
	
	pfh.setRadiusSearch(0.05);
	
	//Compite the features
	pfh.compute(*pfhs);
	
	//pfhs->points.size() should have the same size as the input cloud->points.size()*
}

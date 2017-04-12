//The follwing code snippet will estimate a set of surface normals for all the points in input dataset.
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	...read, pass in or create a point cloud ...
	
	//Create the normal estimation class, and pass the input dataset to it.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	
	ne.setInputCloud (cloud);
	
	//Create an empty kdtree representation, and pass it to the normal estimation object.
	//Its content will be filled inside the object, based on the given input dataset (as no other search surface is given)

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
	
	ne.setSearchMethod(tree);
	
	//Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	
	//use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch(0.03);
	
	//Compute the features
	ne.compute(*cloud_normals);
	
	//Cloud_normals->points.size() should have the same size as the input cloud->points.size()
	

}

//The following code snippet will estimate a set of surface normals for a subset of the points in the inpu dataset.
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

	
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	... read, pass in or create a point cloud...
	
	//Create a set of indices to be used.For simplicity, we're going to be usiing the first 10% of the points in cloud

	std::vector<int> indices (floor (cloud->point.size()/10));
	
	for (size_t i = 0; i < indices.size();++i)
		indices[i] = i;
	
	//create te normal estimation class, and pass the input dataset to it
	pc::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud);
	
	//Pass the indices
	boost::shared_ptr<std::vector<int>> indicesptr (new std::vector<int> (indices));
	ne.setIndices (indicesptr);
	
	//Create an empty kdtree representation, and pass it to the normal estimation obkect.
	//Its content will be filled inside te object, base on the give input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod(tree);	

	//Output dataset
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	
	//Use all neighbors in a sphere radius 3cm
	ne.setRadiusSearch(0.03);
	
	//Compute the features
	ne.compute(*cloud_normals);
	
	//Cloud_normals->points.size() should have the same size as the input indicesptr->size()

}

//Finally, the follwing code snippet will estimate a set of surface normal for all the points in the input dataset, but will estimate their nearest neighbors using another dataset. As previously mentioned, a good usecase for this is when the input is a downsampled version of the surface
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);
	
	...read, pass in or create a point cloud...
	...create a downsampled version of it...
	
	//Create the normal estimation class, and pass the input dataset to it
 	pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
	ne.setInputCloud(cloud_downsampled);
	
	//pass the original data (before downsampling )as the search surface 
	ne.setSearchSurface (cloud);
	
	//Create an empty kdtree representation, and pass it to the normal estimation object.
	//Its content will be filled inside the object, based on the given surface dataset.
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	
	//output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	
	//Use all the neighbors in a sphere of radius 3cm
	ne.setRadiusSearch(0.03);
	
	//Compute the features
	ne.compute(*cloud_normals);
	
	//Cloud_normals->points.size() should have the same size as the input cloud_downsampled->points.size()

} 

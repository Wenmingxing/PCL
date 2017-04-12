#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>

{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
	
	...read, pass in or create a point cloud with normals...
	...(note, you can create a single pointcloud<pointnormal> if you want)..

	//Create the FPFH estimation class, ans pass the input dataset+normals to it
	pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fpfh;
	fpgh.setInputCloud(cloud);
	fpfh.setInputNormals (normals);
	
	//Create an empty kdtree representation, and pass it to theFPFH estimation object.
	//Its content will be filled inside the object, based on the given input dataset (as no other search surface is given)
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	
	fpfh.setSearchMethod(tree);
	
	//Output dataset
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33> ());
	
	//Use all neighbors in a sphere of radius 5cm
	//IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!
	fpfhs.setRadiusSeach(0.05);
	
	//Compute the features 
	fpfh.compute (*fpfhs);
	
	//Fpfhs->points.size() should have the same size as the input cloud->points.size()*


	/*
	for each point p in cloud p
	1.pass 1:
		1.get the nearest neighbors of :math:'p'
		2.for each pair of:math:'p,p_k' (where :math:'p_k' is a neighbor of :math:'p', compute the  three angular values)
		3.bin all the results in an output SPFH histogram

	2.pass 2:
		1.get the nearest neighbors of :math:'p'
		3.use each SPFH of: math'p' with a weighting scheme to assemble the FPFH of :math'p'.
	*/

}

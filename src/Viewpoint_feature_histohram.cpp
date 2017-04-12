#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/io/pcd_io.h>
int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	pcl::io::loadPCDFile<pcl::PointXYZ>("Name_pcd_file",*cloud);
	pcl::VFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::VFHSignature308> vfh;
	vfh.setInputCloud(cloud);
	vfh.setInputNormals(normals);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(pcl::search::KdTree<pcl::PointXYZ>);
	
	vfh.setSearchMethod(tree);
	
	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308>);
	
	vfh.compute(*vfhs);




}

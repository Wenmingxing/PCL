#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/compression/octree_pointcloud_compression.h>

#include <stdio.h>
#include <sstream>
#include <stdlib.h>

#ifdef WIN32
# define sleep(x) Sleep((x) * 1000)
#endif

class SimpleOpenNIViewer
{
public:
	SimpleOpenNIViewer():viewer("Point cloud compression Example"){}

	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
	{
		if (!viewer.wasStopped())
		{
			//stringstream to store compression point cloud
			std::stringstream compressedData;
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGBA> ());
			
			//compress point cloud
			PointCloudEncoder->encodePointCloud (cloud, compressedData);
			
			//Decompress point cloud
			PointCloudDecoder->decodePointCloud (compressedData,cloudOut);

			//show decompressed point cloud
			viewer.showCloud(cloudOut);	
		}	
	
	}

	void run()
	{
		bool showStatistics = true;
		
		//for a fill list of profiles see: /io/include/pcl/compression/compression_profiles.h	
		pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
		
		//instantiate point cloud compression for encoding and decoding
		PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> (compressionProfile,showStatistics);
		
		PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYARGBA> ();
		
		//Create a new grabber for OpenNI devices
		pcl::Grabber* interface = new pcl::OpenNIGrabber();
		
		//Make calback function from member function 
		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind(&SimpleOpenNIViewer::cloud_cb_,this,_1);

		//Connect callback function for desired signal. in this case its a point cloud with colo values
		boost::signals2::connection c = interface->registerCallback(f);
	
		//Start receiving point cloud	
		interface->start();
		
		while (!viewer.wasStopped())
		{	
			sleep(1);		
		}

		interface->stop();

		delete (PointCloudEncoder);
		delete (PointCloudDecoder);


		
	}

	pcl::visualization::CloudViewer viewer;
	
	pcl::io::OctreePointCloudCompression<pcl:;PointXYZRGBA>* PointCloudEncoder;
	pcl::io::OctreePointCloudCompression<pcl::PointXYARGBA>* PointCloudDecoder;
	

};

int main(int argc, char** argv)
{
	SimpleOpenNIViewer v;
	v.run();
	
	return 0;

}


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <iostream>

using namespace std;

using namespace pcl;


class SimpleOpenNIViewer
{
public:
	void cloud_cb_(const PointCloud<PointXYZRGBA>::ConstPtr& cloud)
	{
		static unsigned count =0 ;
		static double last = getTime();
		
		if (++count == 30)
		{
			double now = getTime();
			cout<<"distance of center pixel: "<<cloud->points[(cloud->width>>1) * (cloud->eight + 1)].z <<"mm. Average framerate:  "<<double(count)/double(now - last) <<" Hz "<< endl;
			count = 0;
			last = now;
				
		}	
	}
	
	void run()
	{
		//create a new grabber for OpenNI devices
		Grabber* interface = new OpenNIGrabber();
		
		//Make callback function from member function
		boost::function<void (const PointCloud<PointXYZRGBA>::ConstPtr& )> f = boost::bind(&SimpleOpenNIViewer::cloud_cb_,this,_1);
		
		//connect callback function for desired signal, in this case it's a point cloud with color values
		boost::Signals2::connection c = interface->registerCallback (f);
		
		//Start receiving point clouds
		interface->start();	
		
		//Wait until user quits program with ctrl_c but no busy waiting -> sleep(1)
		while(true)
			boost::this_thread::sleep (boost::posix_time::second(1));
		
		//stop the grabber
		interface->stop();	
	}	

};

int main()
{
	SimpleOpenNIViewer v;
	v.run();
	return (0);

}

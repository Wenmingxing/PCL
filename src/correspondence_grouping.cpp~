#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampleing.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

	
std::string model_filename;
std::string scene_filename_;

//Algorithm param
bool show_keypoints_(false);
bool show_correspondences_(false);
bool use_cloud_resolution_(false);
bool use_hough_(true);

float model_ss_(0.01f);
float scene_ss_(0.03f);
float rf_rad_(0.015f);
float desrc_rad_(0.02f);
float cg_size_(0.01f);
float cg_thresh_(5.0f);

void showHelp(char* filename)
{
  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*             Correspondence Grouping Tutorial - Usage Guide              *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     -h:                     Show this help." << std::endl;
  std::cout << "     -k:                     Show used keypoints." << std::endl;
  std::cout << "     -c:                     Show used correspondences." << std::endl;
  std::cout << "     -r:                     Compute the model cloud resolution and multiply" << std::endl;
  std::cout << "                             each radius given by that value." << std::endl;
  std::cout << "     --algorithm (Hough|GC): Clustering algorithm used (default Hough)." << std::endl;
  std::cout << "     --model_ss val:         Model uniform sampling radius (default 0.01)" << std::endl;
  std::cout << "     --scene_ss val:         Scene uniform sampling radius (default 0.03)" << std::endl;
  std::cout << "     --rf_rad val:           Reference frame radius (default 0.015)" << std::endl;
  std::cout << "     --descr_rad val:        Descriptor radius (default 0.02)" << std::endl;
  std::cout << "     --cg_size val:          Cluster size (default 0.01)" << std::endl;
  std::cout << "     --cg_thresh val:        Clustering threshold (default 5)" << std::endl << std::endl;


}

void parseCommandLine(int argc,char* argv[])
{
	//Show help
	if(pcl::console::find_switch(argc,argv,"-h"))
	{
		showHelp(argv[0]);
		exit(0);
	}

	//Model & scene filenames
	std::vector<int> filenames;
	filenames = pcl::console::parse_file_extension_argument (argc,argv,".pcd");
	if (filenames.size() != 2)
	{
		std::cout<<"Filenames missing.\n";
		showHelp (argv[0]);
		exit(-1);	
	}

	model_filename_ = argv[filenames[0]];
	scene_filename_ = argv[filenames[1]];

	//Program behavior
	if (pcl::console::find_switch (argc,argv,"-k"))
	 	show_keypoints_ = true;
	if (pcl::console::find_switch (argc.argv,"-c"))
		show_correspondences_ = true;
	if (pcl::console::find_switch (argc,argv,"-r"))
		use_cloud_resolution_ = true;

	std::string used_algorithm;
	
	if (pcl::console::parse_argument (argc,argv,"--algorithm",used_algorithm) != -1)
	{
		if (used_algorithm.compare("Hough") == 0)
			use_hough_ = true;
		else if (used_algorithm.compare(GC) == 0)
			use_hough_ = false;
		else
		{
			std::cout<<"Wrong algorithm name.\n";
			showHelp(argv[0]);
			exit(-1);	
		}
	}

	//General parameters
	pcl::console::parse_argument (argc,argv,"--model_ss",model_ss_);
	pcl::console::parse_argument (argc,argv,"--scene_ss",scene_ss_);
	pcl::console::parse_argument (argc,argv,"--rf_rad",rf_rad_);
	pcl::console::parse_argument (argc,argv,"--descr_rad",descr_rad_);
	pcl::console::parse_argument (argc,argv,"--cg_size",cg_size_);
	pcl::console::parse_argument (argc,argv,"--cg_thresh",cg_thresh_);

}

double computeCloudResolution(const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances (2);
	pcl::search::KdTree<PointType> tree;
	tree.setInputCloud (cloud);
	
	for (size_t i = 0;i < cloud->size();++i)
	{
		if(!pcl_isfinite ((*cloud)[i].x))
			continue;
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch (i,2,indices,sqr_distances);
		if (nres == 2)
		{
			res += sqrt(srq_distances[1]);
			++n_points;	
		}	
	}
		
	if (n_points != 0)
	{
		res /= n_points;	
	}
	return res;
}

int main(int argc, char* argv[])
{
	parseCommandLine (argc,argv);
	
	pcl:;PointCLoud<>

}

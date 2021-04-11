#include <boost/thread/thread.hpp>
#include <pcl/common/common.h>
#include <pcl/console/parse.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include "math.h"
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/icp.h>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#pragma comment(lib, "opencv_highgui2412d.lib")
#pragma comment(lib, "opencv_core2412d.lib")

typedef pcl::PointXYZRGB P_pcl;
typedef pcl::PointCloud<P_pcl> point_cloud;
typedef point_cloud::Ptr ptr_cloud;
using namespace std;
using namespace cv;
const double camera_factor = 1;
const double camera_cx = 250.149174;
const double camera_cy = 238.218530;
const double camera_fx = 488.761670;
const double camera_fy = 487.702239;
// const double camera_cx = 314.1146;
// const double camera_cy = 244.0743;
// const double camera_fx = 599.5924;
// const double camera_fy = 599.7701;

// -----Main-----
int main(int argc, char const *argv[])
{
    string strSettingPath = argv[1];
	cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    Mat rawRGB, rawDepth, depthLucid;
    string RGBFile, depthFile;

    Mat points3d, K_rgb;
    fSettings["K_rgb"] >> K_rgb;



	cv::Mat img = cv::imread(fSettings["depth"], cv::IMREAD_ANYDEPTH);
	//ushort d = img.ptr<ushort>(100)[100];
	std::cout << img.type() << " is empty: " << img.empty() << std::endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int m = 0; m < img.rows; m++)
	{
		for (int n = 0; n < img.cols; n++)
		{
			ushort d = img.ptr<ushort>(m)[n];
			if (d == 0)
			{
				continue;
			}
			pcl::PointXYZRGB p;

			p.z = double(d) / camera_factor;
			p.x = (n - camera_cx) * p.z / camera_fx;
			p.y = (m - camera_cy) * p.z / camera_fy;
			p.r = 255;
			
			cloudPtr->points.push_back(p);
		}
	}

	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(cloudPtr);


	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	cloud = *cloudPtr;

	pcl::io::savePLYFile("computer_Depth_o.ply", cloud, true);
	std::cout << " saved " << std::endl;
	// getchar();
	return (0);
}
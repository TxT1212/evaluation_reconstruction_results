#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <glob.h>
#include <opencv2/imgproc.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "Geometry.h"
#include <numeric>    
#include <pcl/common/common.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/visualization/range_image_visualizer.h> //深度图可视化
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/io/vtk_lib_io.h>
// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
using namespace std;
using namespace cv;

std::vector<std::vector<std::string>>  ReadSpaceSeparatedText(std::string filepath);
Eigen::Matrix4d ParsePose(std::vector<std::string> pose);
int main(int, char**) {

    // pointcloud of chessboard
    string chessBoardPath = "/home/txt/HW3/result_point_cloud/calibBoard/pointcloud_EMLab3_calib_board.ply";
    string dataPath = "/media/txt/data1/data/EMLab3/out/";

    pcl::PointCloud<pcl::PointXYZ>::Ptr Pcloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile(chessBoardPath, *Pcloud) != 0)
    {
        PCL_ERROR("Could not load ply");
        return -1;
    }
    pcl::PolygonMesh m;
    // m.cloud.data;
	pcl::SACSegmentation<pcl::PointXYZ> sac;
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
	sac.setInputCloud(Pcloud);
	sac.setMethodType(pcl::SAC_RANSAC);
	sac.setModelType(pcl::SACMODEL_PLANE);
	sac.setDistanceThreshold(15);                  //Distance need to be adjusted according to the obj
	sac.setMaxIterations(100);
	sac.setProbability(0.95); 
	sac.segment(*inliers, *coefficients);
	// boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer);
	// view->addPointCloud(Pcloud);
	//Plane Model: ax+by+cz+d=0; saved in *coefficients
	float a, b, c, d;
	a = coefficients->values[0];
	b = coefficients->values[1];
	c = coefficients->values[2];
	d = coefficients->values[3];
	int Pcloudsize = Pcloud->size();
    GeoPoint p1(-d/a, 0, 0);
    GeoPoint p2(0, -d/b, 0);
    GeoPoint p3(0, 0, -d/c);
    Triangle planeChessBoard(p1, p2, p3, true);

	//------------------------ Projection ------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_proj(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ point;
    vector<float> planeFiterror;
    planeFiterror.reserve(Pcloudsize);
	for (int i = 0; i<Pcloudsize; i++)
	{
		float x, y, z;
        GeoPoint oneCloundPoint(Pcloud->at(i).x, Pcloud->at(i).y, Pcloud->at(i).z);
        planeFiterror.push_back( ptotDistance(planeChessBoard, oneCloundPoint));

	}    

    
    float averageError = accumulate(planeFiterror.begin(), planeFiterror.end(), 0.0)/planeFiterror.size();
    float varianceError = 0;
    for (size_t i = 0; i < Pcloudsize; i++)
    {
        varianceError += pow(planeFiterror[i]-averageError, 2);
    }
    varianceError /= Pcloudsize;
    float stdError = sqrt(varianceError);
    // iterate frames  
    string associationPath = dataPath + "associations.txt";
    string posePath = dataPath + "opt_pose.txt";
    std::vector<std::vector<std::string>> associations=ReadSpaceSeparatedText(associationPath);
    std::vector<std::vector<std::string>> poses=ReadSpaceSeparatedText(posePath);    
    std::vector<cv::Point2f> pointsColor;
    std::vector<GeoPoint> poinstProjected;
    std::vector<float> projectError;

    const cv::TermCriteria termCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::COUNT, 100, DBL_EPSILON);
    int countFound = 0;
    int iterStep = 10, iterIndex;
    const float fx = 976.7616704123305;
    const float fy = 975.7022391258554;
    const float cx = 1024.149174311481;
    const float cy = 776.2185301700285;
    vector<double> projectErrorsAllFrame;
    vector<double> stdProjectErrorsAllFrame;


    for(int i=0;i<poses.size();i+=10){

        vector<string> pose_vec=poses[i];
        vector<string>::const_iterator first=pose_vec.begin()+1;
        vector<string>::const_iterator end=pose_vec.begin()+8;
        vector<string> pose_vec_new(first,end);
        Eigen::Matrix4d cur_pose=ParsePose(pose_vec_new);
        Eigen::Matrix3d cur_pose_R = cur_pose.block<3,3>(0,0);
        Eigen::Vector3d cur_pose_t = cur_pose.block<3,1>(0,3);
        iterIndex = stoi(pose_vec[0]);
        string color_path=dataPath+associations[iterIndex][1];
        string depth_path=dataPath+associations[iterIndex][3];
        // cout<<"\ncolor_path = "<<color_path<<endl;
        // cout<<"depth_path = "<<depth_path<<endl;
        Mat colorRect = imread(color_path, IMREAD_GRAYSCALE);
        cv::Size boardDims(11,8);
        bool foundColor = cv::findChessboardCorners(colorRect, boardDims, pointsColor, cv::CALIB_CB_FAST_CHECK);

        if(foundColor)
        {
        // cout<<color_path<<  " " << foundColor << " " << colorRect.type()<< endl;

            countFound++;
            poinstProjected.clear();

            cv::cornerSubPix(colorRect, pointsColor, cv::Size(11, 11), cv::Size(-1, -1), termCriteria);

            for (size_t i_chessPoint = 0; i_chessPoint < pointsColor.size(); i_chessPoint++) // project pixles to the chess board plane
            {
                cv::Point2f nowPoint = pointsColor[i_chessPoint];
                Eigen::Vector3d nowEigenPoint_c((pointsColor[i_chessPoint].x - cx)/fx, (pointsColor[i_chessPoint].y - cy)/fy, 1);
                // cout << endl;
                Eigen::Vector3d nowEigenPoint_w = cur_pose_R * nowEigenPoint_c + cur_pose_t;

                GeoPoint curGeoPoint_c(nowEigenPoint_w(0), nowEigenPoint_w(1), nowEigenPoint_w.z());
                GeoPoint curGeoPose_t(cur_pose_t.x(), cur_pose_t.y(), cur_pose_t.z());
                Line curLine(curGeoPoint_c, curGeoPose_t, false);
                GeoPoint curGeoPointOnPlane(ltotInterGeoPoint(planeChessBoard, curLine));
                poinstProjected.push_back(curGeoPointOnPlane);
            }
            double projectError, varProjectError = 0;
            vector<double> projectErrors;
            vector<double> projectErrors_;
            for (size_t i_chessPoint = 0; i_chessPoint < pointsColor.size(); i_chessPoint++) // caculate error
            {
                if( i_chessPoint/11 == i_chessPoint/11.||i_chessPoint/8 == i_chessPoint/8.)
                // if(i_chessPoint/8 == i_chessPoint/8. )
                {
                    continue;
                }
                double distTwoPoints_err = 0.03 - distance(poinstProjected[i_chessPoint], poinstProjected[i_chessPoint-1]);
                projectErrors_.push_back(distTwoPoints_err);
                distTwoPoints_err = distTwoPoints_err < 0 ? -distTwoPoints_err : distTwoPoints_err;
                projectErrors.push_back(distTwoPoints_err);

            }
            projectError = accumulate(projectErrors.begin(), projectErrors.end(), 0.0)/projectErrors.size();
            for(auto iter: projectErrors)
            {
                varProjectError += pow(iter - projectError, 2);
            }
            varProjectError /= projectErrors.size();
            stdProjectErrorsAllFrame.push_back(sqrt(varProjectError));
            projectErrorsAllFrame.push_back(projectError);
            cout << color_path << " projectErrors: size " << projectErrors.size() << " error " << projectError << endl;
            if (projectError > 0.001)
            {
                // for (auto iter: projectErrors)
                // {
                //     cout << iter << " ";
                // }
                for (auto iter: projectErrors_)
                {
                    cout << iter << " ";
                }                
                cout << endl;
            }
            
        }
    }
    double projectErrorAllFrame = accumulate(projectErrorsAllFrame.begin(), projectErrorsAllFrame.end(), 0.0)/projectErrorsAllFrame.size();
    cout << "projection average error: " << projectErrorAllFrame << "\nmid: "<< projectErrorsAllFrame[projectErrorsAllFrame.size()/3]<<endl;
    cout << "projection std error mid: "<< stdProjectErrorsAllFrame[projectErrorsAllFrame.size()/3]<<endl;
    double varProjectError = 0;
    for (auto iter: projectErrorsAllFrame)
    {
        // varProjectError += pow(iter-projectErrorAllFrame)s
    }
    
    std::cout << "Hello, world! " << countFound << endl;
    cout << "plane fit average error: " << averageError << endl;
    cout << "plane fit stdError error: " << stdError << endl;

}

std::vector<std::vector<std::string>>  ReadSpaceSeparatedText(std::string filepath){
    //打开文件
    std::cout << "Begin reading space separated text file ..." << std::endl;
    std::ifstream file(filepath);
    if (!file) {
        std::cerr << "fail to open file" << filepath << std::endl;
        exit(1);
    }

    std::vector<std::vector<std::string>> file_content;    //存储每个数据的数据结构
    std::string file_line;

    while (std::getline(file, file_line)) {    //读取每一行数据
        std::vector<std::string> file_content_line;    //存储分割之后每一行的四个数据
        std::stringstream ss;    //利用内存流进行每行数据分割内存流
        std::string each_elem;
        ss << file_line;

        while (ss >> each_elem) {
            file_content_line.push_back(each_elem);
        }
        file_content.emplace_back(file_content_line);    //数据组装
    };
    return file_content;
}
Eigen::Matrix4d ParsePose(std::vector<std::string> pose){
    float w=stof(pose[6]);  float x=stof(pose[3]);
    float y=stof(pose[4]);  float z=stof(pose[5]);
    float cx=stof(pose[0]);  float cy=stof(pose[1]);float cz=stof(pose[2]);
    Eigen::Quaterniond q(w,x,y,z);
    Eigen::Matrix4d anwser;
    anwser.block<3,3>(0,0)=q.toRotationMatrix();
    anwser(0,3) = cx;
    anwser(1,3) = cy;
    anwser(2,3) = cz;
    anwser(3,3) = 1;
    anwser.block<1,3>(3,0) = Eigen::Vector3d::Zero();
    // cout << anwser << endl;
    return anwser;
}
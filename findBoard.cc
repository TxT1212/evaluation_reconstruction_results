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
using namespace std;
using namespace cv;

std::vector<std::vector<std::string>>  ReadSpaceSeparatedText(std::string filepath);
Eigen::Matrix4d ParsePose(std::vector<std::string> pose);
int main(int, char**) {
  
    std::vector<cv::Point2f> pointsColor;
    string color_path = "/home/txt/Documents/20200721/EMLab2/out/RGB/rgb05598.png";
    Mat colorRect = imread(color_path, IMREAD_GRAYSCALE);
    cv::Size boardDims(11,8);
    bool foundColor = cv::findChessboardCorners(colorRect, boardDims, pointsColor, cv::CALIB_CB_FAST_CHECK);
    const cv::TermCriteria termCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::COUNT, 100, DBL_EPSILON);

    if(foundColor)
    {
        cv::cornerSubPix(colorRect, pointsColor, cv::Size(11, 11), cv::Size(-1, -1), termCriteria);
    }
    cv::Mat colorDisp;
    cv::cvtColor(colorRect, colorDisp, CV_GRAY2BGR);
    cv::drawChessboardCorners(colorDisp, boardDims, pointsColor, foundColor);
    // cv::resize(colorDisp, colorDisp, cv::Size(), 0.5, 0.5);
    imshow("1",  colorDisp);
    cv::waitKey();
    // }


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
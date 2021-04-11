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
#include <numeric>    
#include <vector>
#include <algorithm>    // std::sort
using namespace std;
std::vector<std::vector<std::string>>  ReadSpaceSeparatedText(std::string filepath){
    //打开文件
    std::cout << "Begin reading space separated text file ..." << std::endl;
    std::ifstream file(filepath);
    if (!file) {
        std::cerr << "fail to open associate file" << std::endl;
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
    // cout << anwser << endl;
    return anwser;
}
int main(){
    string ws_folder("/media/txt/data1/data/kitchen2/out/");

    // cout << ws_folder << "associations.txt" << endl;
    std::vector<std::vector<std::string>> associations=ReadSpaceSeparatedText(ws_folder+"associations.txt");
    std::vector<std::vector<std::string>> poses=ReadSpaceSeparatedText(ws_folder+"opt_pose.txt");
    Eigen::Matrix4d old_pose,cur_pose;
    const Eigen::Vector3d vec_z = {0, 0, 1};
    vector<double> F2FAngles;
    F2FAngles.reserve(poses.size()/5);
    vector<double> F2FDistances;
    F2FDistances.reserve(poses.size()/5);
    vector<int> F2Findex;

    for(int i=0;i<poses.size();i+=3){
    // for(int i=0;i<5;i+=iterStep){

        vector<string> pose_vec=poses[i];
        // std::sort(poses.begin(), poses.begin());
        vector<string>::const_iterator first=pose_vec.begin()+1;
        vector<string>::const_iterator end=pose_vec.begin()+8;
        vector<string> pose_vec_new(first,end);
        old_pose = cur_pose;
        cur_pose=ParsePose(pose_vec_new);
        if (i == 0) continue;
        // double cur_angle = (cur_pose.block<3,3>(0,0).transpose()*old_pose.block<3,3>(0,0)*vec_z).sum();?
        F2FAngles.emplace_back(acos((cur_pose.block<3,3>(0,0).transpose()*old_pose.block<3,3>(0,0)*vec_z).z()));
        F2FDistances.emplace_back((cur_pose.block<3,1>(0,3)-old_pose.block<3,1>(0,3)).stableNorm());
        cout << stoi(pose_vec[0]) << " " << (cur_pose.block<3,1>(0,3)-old_pose.block<3,1>(0,3)).x() << " " << (cur_pose.block<3,1>(0,3)-old_pose.block<3,1>(0,3)).y() << " " << (cur_pose.block<3,1>(0,3)-old_pose.block<3,1>(0,3)).z() << "  angle: " << (cur_pose.block<3,3>(0,0).transpose()*old_pose.block<3,3>(0,0)*vec_z).z() << " acos:" <<  acos((cur_pose.block<3,3>(0,0).transpose()*old_pose.block<3,3>(0,0)*vec_z).z()) << endl;
        // int iterIndex = stoi(pose_vec[0]);
        F2Findex.emplace_back(stoi(pose_vec[0]));
        // string color_path=ws_folder+associations[iterIndex][1];
        // string depth_path=ws_folder+associations[iterIndex][3];
        // cout<<"\ncolor_path = "<<color_path<<endl;
        // cout<<"depth_path = "<<depth_path<<endl;
    }
    ofstream outFile;
    outFile.open(ws_folder + "pose_delta.txt");
    for (size_t i = 0; i < F2Findex.size(); i++)
    {
        outFile << F2Findex[i] << " " << F2FAngles[i] << " " << F2FDistances[i] << endl;
    }
    return 0;
}
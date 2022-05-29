﻿#include <iostream>
#include <opencv/cv.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <time.h>
#include <list>
#include <omp.h>
#include <stdio.h>
#include <ctime>
#include <pthread.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include "GCSLAM/frame.h"
#include "GCSLAM/MILD/loop_closure_detector.hpp"
#include "GCSLAM/MILD/BayesianFilter.hpp"
#include "GCSLAM/MultiViewGeometry.h"
#include "GCSLAM/GCSLAM.h"

#include "GCFusion/MapMaintain.hpp"
#include "GCFusion/MobileGUI.hpp"
#include "GCFusion/MobileFusion.h"


#include "BasicAPI.h"

#include "CHISEL/src/open_chisel/Chisel.h"
#include "CHISEL/src/open_chisel/ProjectionIntegrator.h"
#include "CHISEL/src/open_chisel/camera/PinholeCamera.h"
#include "CHISEL/src/open_chisel/Stopwatch.h"

#include "Tools/LogReader.h"
#include "Tools/LiveLogReader.h"
#include "Tools/RawLogReader.h"
#include "Tools/RealSenseInterface.h"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

ros::Publisher pub_keyframe_pose;
ros::Publisher pub_curr_frame;

using namespace std;
using namespace cv;

#define MULTI_THREAD 1


#define INPUT_SOURCE_DATABASE 0
#define INPUT_SOURCE_OPENNI 1
#define INPUT_SOURCE_REALSENSE 2

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "Flashfusion");
	ros::NodeHandle n("~");
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    int showCaseMode = 0;
    string basepath;
    float ipnutVoxelResolution = 0.005;
    int sensorType = 0;

    pub_keyframe_pose = n.advertise<nav_msgs::Odometry>("keyframe_pose", 1000);
    pub_curr_frame = n.advertise<sensor_msgs::Image>("curr_frame", 1000);

    MultiViewGeometry::CameraPara camera;
    BasicAPI::parseInput(argc,argv,showCaseMode,ipnutVoxelResolution,basepath,MultiViewGeometry::g_para,camera,sensorType);

    vector <string> rgb_files;
    vector <string> depth_files;
    Eigen::MatrixXd ground_truth;
    vector<double> time_stamp;
    std::vector<float> processingTimePerFrame;

    LogReader *logReader = NULL;

    rs2::pipeline pipe;
    int rs2active = 0;

    if(sensorType == INPUT_SOURCE_DATABASE) BasicAPI::initOfflineData(basepath, rgb_files, depth_files, time_stamp, ground_truth);
    if(sensorType == INPUT_SOURCE_OPENNI)       //BasicAPI::initOpenNICamera(logReader,camera);
    {

        logReader = new LiveLogReader("", 0, LiveLogReader::CameraType::OpenNI2);
        bool good = ((LiveLogReader *)logReader)->cam->ok();
 
        printf("%d\r\n", logReader != NULL);
    }
    if(sensorType == INPUT_SOURCE_REALSENSE)    rs2active = BasicAPI::initRS2Camera(pipe,camera);

    cout << "begin init rendering" << endl;
    MobileGUI gui(showCaseMode);
    MobileFusion gcFusion;
    gcFusion.initChiselMap(camera,ipnutVoxelResolution,
                           MultiViewGeometry::g_para.far_plane_distance);

    cout << "begin init gcSLAM" << endl;
    int maxFrameNum = 20000;
    gcFusion.initGCSLAM(maxFrameNum,MultiViewGeometry::g_para,camera);

    int i = 0;
    pangolin::GlTexture imageTexture(640,480,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);


#if MULTI_THREAD
    boost::thread map_thread(boost::bind(&MobileFusion::MapManagement,&gcFusion));
#endif

    float integrateLocalFrameNum = 6;

    int portableDeviceFlag = 0;
    // for portable devices
    if(std::thread::hardware_concurrency() < 4)
    {
        portableDeviceFlag = 1;
        integrateLocalFrameNum = 3;
    }

    printf("%d\r\n", logReader != NULL);
    while(!pangolin::ShouldQuit() && (i < rgb_files.size() || (logReader != NULL || rs2active)))
    {
        if(!gui.pause->Get() || pangolin::Pushed(*gui.step))
        {
            TICK("System::1::FrameTime");
            Frame f;

            TICK("System::FrameTime::1::LoadRawData");
            if(sensorType == INPUT_SOURCE_DATABASE) BasicAPI::LoadRawData(i,f,rgb_files,depth_files,time_stamp, camera);
            if(sensorType == INPUT_SOURCE_OPENNI) BasicAPI::LoadOnlineOPENNIData(f, logReader, camera);
            if(sensorType == INPUT_SOURCE_REALSENSE) rs2active = BasicAPI::LoadOnlineRS2Data(f,pipe, camera);
            TOCK("System::FrameTime::1::LoadRawData");

            TICK("System::FrameTime::2::UpdateFrame");
            int feature_num = MultiViewGeometry::g_para.max_feature_num;
            if(portableDeviceFlag)  feature_num = 600;
            BasicAPI::detectAndExtractFeatures(f,feature_num,camera);
            BasicAPI::extractNormalMapSIMD(f.refined_depth, f.normal_map,camera.c_fx,camera.c_fy,camera.c_cx, camera.c_cy);
            gcFusion.gcSLAM.update_frame(f);
            Frame &frame_current = gcFusion.gcSLAM.globalFrameList.back();
            if(frame_current.tracking_success &&!frame_current.is_keyframe)
            {
                int keyframeIndex = gcFusion.gcSLAM.GetKeyframeDataList().back().keyFrameIndex;
                BasicAPI::refineKeyframesSIMD(gcFusion.gcSLAM.globalFrameList[keyframeIndex],frame_current,camera);
                BasicAPI::refineNewframesSIMD(gcFusion.gcSLAM.globalFrameList[keyframeIndex],frame_current,camera);
            }
            BasicAPI::refineDepthUseNormalSIMD((float *)frame_current.normal_map.data, (float *)frame_current.refined_depth.data,camera.c_fx,camera.c_fy, camera.c_cx, camera.c_cy,
                                 camera.width, camera.height);
            TOCK("System::FrameTime::2::UpdateFrame");




            if(frame_current.is_keyframe )
            {
                BasicAPI::checkColorQuality(frame_current.normal_map,frame_current.colorValidFlag,camera.c_fx,camera.c_fy,camera.c_cx, camera.c_cy);
                gcFusion.clearRedudentFrameMemory(integrateLocalFrameNum);
#if MULTI_THREAD
                gcFusion.updateGlobalMap(gcFusion.gcSLAM.globalFrameList.size(),gcFusion.gcSLAM.globalFrameList.size() - 1);
#else

                gcFusion.tsdfFusion(gcFusion.gcSLAM.globalFrameList,
                                    gcFusion.gcSLAM.globalFrameList.size() - 1,
                                    gcFusion.gcSLAM.GetKeyframeDataList(),
                                    gcFusion.gcSLAM.GetKeyframeDataList().size() - 2);
#endif
            }

            imageTexture.Upload(gcFusion.gcSLAM.globalFrameList.back().rgb.data,GL_RGB,GL_UNSIGNED_BYTE);


            float memoryConsumption = 0;
            for(int k = 0; k < gcFusion.gcSLAM.globalFrameList.size(); k++)
            {
                memoryConsumption += gcFusion.gcSLAM.globalFrameList[k].GetOccupiedMemorySize();
            }
            cout << "memory for frames: " << memoryConsumption / 1024 / 1024 << " " << memoryConsumption / 1024 / 1024 /gcFusion.gcSLAM.globalFrameList.size() << endl;
            TOCK("System::1::FrameTime");
            printf("frame %d time: %f\r\n",gcFusion.gcSLAM.globalFrameList.back().frame_index,Stopwatch::getInstance().getTiming("System::1::FrameTime"));
            processingTimePerFrame.push_back(Stopwatch::getInstance().getTiming("System::1::FrameTime"));
            i++;

        }

        TICK("System::2::GUI");
        if(gui.followPose->Get())
        {
            Eigen::Matrix4f currPose;
            if(gcFusion.gcSLAM.globalFrameList.back().tracking_success && gcFusion.gcSLAM.globalFrameList.back().origin_index == 0)
            {
                currPose = gcFusion.gcSLAM.globalFrameList.back().pose_sophus[0].matrix().cast<float>();
            }
            gui.setModelView(currPose, camera.c_fy < 0);

        nav_msgs::Odometry odometry;
        odometry.header.stamp = ros::Time::now();
        odometry.header.frame_id = "map";
        odometry.pose.pose.position.x = currPose(0, 3);
        odometry.pose.pose.position.y = currPose(1, 3);
        odometry.pose.pose.position.z = currPose(2, 3);
        Eigen::Quaternionf q(currPose.block<3, 3>(0, 0));
        odometry.pose.pose.orientation.x = q.x();
        odometry.pose.pose.orientation.y = q.y();
        odometry.pose.pose.orientation.z = q.z();
        odometry.pose.pose.orientation.w = q.w();
        //printf("time: %f t: %f %f %f r: %f %f %f %f\n", odometry.header.stamp.toSec(), P.x(), P.y(), P.z(), R.w(), R.x(), R.y(), R.z());

        pub_keyframe_pose.publish(odometry);

        Frame &frame_current = gcFusion.gcSLAM.globalFrameList.back();
        cv::Mat projected_image = frame_current.rgb.clone();            
        printf("cols %d rows %d ", projected_image.cols, projected_image.rows);
        std_msgs::Header header;
        header.frame_id = "map";
        header.stamp = ros::Time::now();
        sensor_msgs::ImagePtr projected_imageMsg =
          cv_bridge::CvImage(header, "bgr8", projected_image).toImageMsg();
        pub_curr_frame.publish(projected_imageMsg);

        
        }
        gui.PreCall();
        if(gui.drawGlobalModel->Get())
        {
            gcFusion.MobileShow(gui.s_cam.GetProjectionModelViewMatrix(),
                                VERTEX_WEIGHT_THRESHOLD,
                                gui.drawUnstable->Get(),
                                gui.drawNormals->Get(),
                                gui.drawColors->Get(),
                                gui.drawPoints->Get(),
                                gui.drawWindow->Get(),
                                gui.drawTimes->Get(),
                                gcFusion.gcSLAM.globalFrameList.size(),
                                1,
                                gcFusion.gcSLAM.globalFrameList);
        }
        gui.DisplayImg(gui.RGB,&imageTexture);
        gui.PostCall();
        TOCK("System::2::GUI");
    }


    char fileName[2560];
    memset(fileName,0,2560);
    sprintf(fileName,"%s/trajectory.txt",basepath.c_str());
    BasicAPI::saveTrajectoryFrameList(gcFusion.gcSLAM.globalFrameList,fileName);

    float total_time = 0;
    for(int i = 0; i < processingTimePerFrame.size(); i++)
    {
        total_time += processingTimePerFrame[i];
    }
    cout << "average processing time per frame: " << total_time / processingTimePerFrame.size() << endl;
    memset(fileName,0,2560);
    sprintf(fileName,"%s/OnlineModel_%dmm.ply",basepath.c_str(),(int)(1000 *(gcFusion.GetVoxelResolution())));
    cout << "saving online model to:    " << fileName << endl;
    gcFusion.chiselMap->SaveAllMeshesToPLY(fileName);
    gcFusion.chiselMap->Reset();


    cout <<"offline re-integrate all frames" << endl;
    TICK("Final::IntegrateAllFrames");
    for(int i = 0; i < gcFusion.gcSLAM.globalFrameList.size(); i++)
    {
        gcFusion.IntegrateFrame(gcFusion.gcSLAM.globalFrameList[i]);
    }
    TOCK("Final::IntegrateAllFrames");
    gcFusion.chiselMap->UpdateMeshes(gcFusion.cameraModel);
    memset(fileName,0,2560);
    sprintf(fileName,"%s/finalModelAllframes_%dmm.ply",basepath.c_str(),(int)(1000 *(gcFusion.GetVoxelResolution())));
    cout << "saving offline model to:    " << fileName << endl;
    gcFusion.chiselMap->SaveAllMeshesToPLY(fileName);
    gcFusion.chiselMap->Reset();



    for(int i = 0; i < gcFusion.gcSLAM.KeyframeDataList.size(); i++)
    {
        MultiViewGeometry::KeyFrameDatabase kfd = gcFusion.gcSLAM.KeyframeDataList[i];
        Frame &f = gcFusion.gcSLAM.globalFrameList[kfd.keyFrameIndex];

        Eigen::MatrixXf transform = f.pose_sophus[0].matrix().cast<float>();
        transform = transform.inverse();
        Eigen::Matrix3f r = transform.block<3,3>(0,0);
        Eigen::MatrixXf t = transform.block<3,1>(0,3);

        memset(fileName,0,2560);
        sprintf(fileName,"%s/texture/%06d.cam",basepath.c_str(),i);
        FILE * fp = fopen(fileName,"w+");
        fprintf(fp,"%f %f %f %f %f %f %f %f %f %f %f %f\r\n",
                t(0),t(1),t(2),
                r(0,0),r(0,1),r(0,2),
                r(1,0),r(1,1),r(1,2),
                r(2,0),r(2,1),r(2,2));
        fprintf(fp,"%f %f %f %f %f %f",
                camera.c_fx / camera.width ,camera.d[0],camera.d[1],
                camera.c_fx / camera.c_fy, camera.c_cx / camera.width, camera.c_cy / camera.height);
        fclose(fp);
        memset(fileName,0,2560);
        sprintf(fileName,"%s/texture/%06d.png",basepath.c_str(),i);
        cv::imwrite(fileName,f.rgb);

    }

    Stopwatch::getInstance().printAll();

    cout << "program finish" << endl;

#if MULTI_THREAD
    map_thread.join();
#endif
}

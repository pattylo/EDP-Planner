#include <sstream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "include/run_yolo.h"
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <opencv2/tracking/kalman_filters.hpp>
using namespace std;
static cv::Mat frame;

static cv::String weightpath ="/home/patrick/camera_ws/src/camera/src/include/ourclass-best.weights";
static cv::String cfgpath ="/home/patrick/camera_ws/src/camera/src/include/yolov4-tiny608.cfg";
static cv::String classnamepath = "/home/patrick/camera_ws/src/camera/src/include/coco.names";

static run_yolo Yolonet(cfgpath, weightpath, classnamepath, float(0.75));

int main(int argc, char** argv)
{
    cout<<"Object detection..."<<endl;

    ros::init(argc, argv, "yolotiny");
    ros::NodeHandle nh;

    // >>>> Kalman Filter
    int stateSize = 4;
    int measSize = 2;
    int contrSize = 0;

    unsigned int type = CV_32F;
    cv::KalmanFilter kf(stateSize, measSize, contrSize, type);

    cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y]
    cv::Mat meas(measSize, 1, type);    // [z_x,z_y]
    //cv::Mat procNoise(stateSize, 1, type)
    // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1 0 dT 0  ]
    // [ 0 1 0  dT ]
    // [ 0 0 1  0  ]
    // [ 0 0 0  1  ]

    cv::setIdentity(kf.transitionMatrix);

    // Measure Matrix H
    // [ 1 0 0 0 ]
    // [ 0 1 0 0 ]

    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(5) = 1.0f;

    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0    ]
    // [ 0    Ey  0     0    ]
    // [ 0    0   Ev_x  0    ]
    // [ 0    0   0     Ev_y ]

    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(5) = 1e-2;
    kf.processNoiseCov.at<float>(10) = 5.0f;
    kf.processNoiseCov.at<float>(15) = 5.0f;


    // Measures Noise Covariance Matrix R
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
    // <<<< Kalman Filter

    // Camera Index
    int idx = 0;

    // Camera Capture
    cv::VideoCapture cap("/home/patrick/camera_ws/src/camera/src/include/test_.avi");
    if (!cap.isOpened())
    {
        cout << "Error opening video stream or file" << endl;
        return -1;
    }

    bool found = false;
    int notFoundCount = 0;
    cv::Mat frame;
    double dT;
    double tpf=0;
    int w = 200,h = 200;
    double ticks = 0;

    while(1)
    {
        cap >> frame;
        cv::Mat res;
        frame.copyTo( res );
        double precTick = ticks;
        ticks = (double) cv::getTickCount();

        double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds
        if (found)
        {
            // >>>> Matrix A
            kf.transitionMatrix.at<float>(2) = dT;
            kf.transitionMatrix.at<float>(7) = dT;
            // <<<< Matrix A
//            cout << "dT:" << endl << dT << endl;
//            cout << "State post:" << endl << state << endl;

            state = kf.predict();

            cv::Point center;
            center.x = state.at<float>(0);
            center.y = state.at<float>(1);

            cv::Rect predRect;
            predRect.width = w;
            predRect.height = h;
            predRect.x = state.at<float>(0) - w / 2;
            predRect.y = state.at<float>(1) - h / 2;
            cv::Scalar color(rand()&255, rand()&255, rand()&255);
            cv::rectangle(res, predRect, color, 2);
        }
        Yolonet.rundarknet(frame);
        Yolonet.display(frame);
        vector<objectinfo> temp = Yolonet.obj_vector;
        cout<<"temp size: "<<temp.size()<<endl;

        cv::Rect interested;
        vector<objectinfo> potential;
        vector<float> potential_c;
        vector<float> potential_c_area;
        bool got=false;
        if(temp.size()!=0)
        {
            for(auto stuff:temp)
            {
                if(stuff.classnameofdetection=="person")
                {
                    potential.push_back(stuff);
                    potential_c.push_back(stuff.confidence);
                    potential_c_area.push_back(stuff.boundingbox.area());
                }
            }
            cout<<"potential size: "<<potential.size()<<endl;

            if(potential.size()!=0)
            {
//                int maxElementIndex = max_element(potential_c.begin(),potential_c.end()) - potential_c.begin();
                int maxElementIndex = max_element(potential_c_area.begin(),potential_c_area.end()) - potential_c_area.begin();
                interested = potential[maxElementIndex].boundingbox;
                got = true;
                cv::rectangle(res, interested, CV_RGB(255,0,0), 2);
                cout<<"hi"<<endl;
                tpf = Yolonet.appro_fps;
                w=interested.width;
                h=interested.height;
            }
        }

        if(!got)
        {
            notFoundCount++;
            cout << "notFoundCount:" << notFoundCount << endl;
            if(notFoundCount>100)
            {
                found = false;
            }
        }
        else
        {
//            cout<<"hey"<<endl;
            notFoundCount = 0;
            meas.at<float>(0) = interested.x + interested.width /  2;
            meas.at<float>(1) = interested.y + interested.height / 2;
            if (!found) // First detection!
            {
                // >>>> Initialization
                kf.errorCovPre.at<float>(0) = 1; // px
                kf.errorCovPre.at<float>(5) = 1; // px
                kf.errorCovPre.at<float>(10) = 1;
                kf.errorCovPre.at<float>(14) = 1;

                state.at<float>(0) = meas.at<float>(0);
                state.at<float>(1) = meas.at<float>(1);
                state.at<float>(2) = 0;
                state.at<float>(3) = 0;
                // <<<< Initialization

                kf.statePost = state;

                found = true;
            }
            else
                kf.correct(meas); // Kalman Correction
        }
        cv::imshow("Tracking", res);
        cv::waitKey(20);




        if(frame.empty())
            break;

        char c = (char)cv::waitKey(25);
        if (c == 27)
            break;

    }

    ros::spin();
    return 0;
}

    // [



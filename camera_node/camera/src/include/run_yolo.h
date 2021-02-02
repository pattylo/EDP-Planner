
#include <iostream>
#include <fstream>
#include <istream>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <chrono>
#include <iomanip>

using namespace std;

class run_yolo
{
    cv::String cfg_file;
    cv::String weights_file;
    cv::String obj_file;

    float set_confidence;
    vector<std::string> classnames;

    void findboundingboxes(cv::Mat &frame);
    void findwhichboundingboxrocks(vector<cv::Mat> &netOut, cv::Mat &frame);
    void getclassname(vector<std::string> &classnames);

public:
    run_yolo(const cv::String cfgfile, const cv::String weightfile, const cv::String objfile, const float confidence);
    ~run_yolo();

    void rundarknet(cv::Mat &frame);
    void display(cv::Mat frame);
};

#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;

#define use_001_101 1
#define use_002_010 2
#define use_012_020 3
#define dataset use_001_101
//#define dataset use_002_010
//#define dataset use_012_020

#if dataset == use_001_101
#define dataset "001_101"
int img_amount = 11;
const char *imgs[] = { "001", "011", "021", "031", "041", "051",
                       "061", "071", "081", "091", "101"};
#elif dataset == use_002_010
#define dataset "002_010"
int img_amount = 9;
const char *imgs[] = { "002", "003", "004", "005", "006", "007",
                       "008", "009", "010"};
#elif dataset == use_012_020
#define dataset "012_020"
int img_amount = 9;
const char *imgs[] = { "012", "013", "014", "015", "016", "017",
                       "018", "019", "020"};
#endif

int track_amount = 0;
std::vector<int> track_num;

struct Feature_Point{
    int id;
    float u, v;
};

void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

cv::Mat prev_img, cur_img;
std::vector<cv::Point2f> cornersA, cornersB;
const int MAX_CORNERS = 500;
Feature_Point fps[MAX_CORNERS];
void opti_track_LK(const cv::Mat &_img)
{
    cv::Mat img;
    img = _img;

    cv::resize(img, img, cv::Size(img.cols*2, img.rows*2));

    if (cur_img.empty())
    {
        prev_img = cur_img = img;

        cv::goodFeaturesToTrack(prev_img, cornersA, MAX_CORNERS, 0.01, 20, cv::noArray(), 3, false, 0.04);
        cv::cornerSubPix(prev_img,cornersA, cv::Size(10, 10), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,20,0.03 ));

        for (int i=0; i < MAX_CORNERS; i++) fps[i].id = i;
    }
    else
    {
        cur_img = img;
    }

    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev_img, cur_img, cornersA, cornersB, status, err, cv::Size(21, 21), 3);

    cv::Mat img_bgr;
    cv::cvtColor(cur_img, img_bgr, CV_GRAY2BGR);
    int nega=0, posi=0;
    for (int i = 0; i < static_cast<int>(cornersA.size()); ++i)
    {
        if (!status[i]) continue;
        cv::circle(img_bgr, cornersB[i], 3, Scalar(0, 255, 0), -1);
        cv::line(img_bgr, cornersA[i], cornersB[i], cv::Scalar(0, 255, 0), 1);

        if (cornersB[i].x - cornersA[i].x < 0) nega++;
        else posi++;

        putText(img_bgr, std::to_string(fps[i].id), cornersB[i], cv::FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(0, 0, 255), 1, 8, 0);
        fps[i].u = cornersB[i].x;
        fps[i].v = cornersB[i].y;

//        std::stringstream text;
//        text << "(" << std::setprecision(3) << cornersB[i].x << ", " << std::setprecision(3) << cornersB[i].y << ")";
//        putText(img_bgr, text.str(), cornersB[i], cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1, 8, 0);
    }

    if (track_amount > 1)
    {
        std::cout << std::endl << "ID: pixel_u, pixel_v" << std::endl;
        for (int i=0; i < MAX_CORNERS; i++)
        {
            for (int j=0; j < track_amount; j++)
                if (track_num[j] == fps[i].id) std::cout << track_num[j] << ": " << fps[i].u << ", " << fps[i].v << std::endl;
        }
        std::cout << "================" << std::endl;
    }

    cv::imshow("source", img_bgr);
    waitKey();

    prev_img = cur_img;
    cornersA = cornersB;
}

int main(int argc, char **argv)
{
    track_amount = argc - 1;
    if (track_amount > 0)
    {
        for (int i=0; i < track_amount; i++)
            track_num.push_back(atoi(argv[i+1]));
    }

    for (int i=0; i < img_amount; i++)
    {
        char img_loca[100];
        sprintf(img_loca, "/home/zhehui/deformation_ws/data/%s/%s.tif", dataset, imgs[i]);
        Mat cv_img = imread(img_loca, CV_LOAD_IMAGE_GRAYSCALE);
        if (!cv_img.data)
        {
            std::cerr<<"Couldn't read image!" << std::endl;
            return -1;
        }
        else
            std::cout << "Read image: " << img_loca << std::endl;

        opti_track_LK(cv_img);
    }

    return 0;
}

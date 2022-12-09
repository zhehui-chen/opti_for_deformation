#include <iostream>
using namespace std;

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;

// input images
string dataset = "test";
int const img_amount = 11;

bool ROI = false;
int rois[4] = {0}; // lower_u, upper_u, lower_v, upper_v

// output results
bool save_result = true;
string save_path = "results.csv";

// feature tracker
struct Feature_Point{
    int id;
    float u[img_amount], v[img_amount];
};
const int MAX_CORNERS = 500;
Feature_Point fps[MAX_CORNERS]; // Feature PoitS

cv::Mat prev_img, cur_img;
std::vector<cv::Point2f> cornersA, cornersB;
void opti_track_LK(const cv::Mat &_img, int frame_num)
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
        fps[i].u[frame_num] = cornersB[i].x;
        fps[i].v[frame_num] = cornersB[i].y;

        if (ROI)
            if (cornersB[i].x < rois[0] || cornersB[i].x > rois[1] || cornersB[i].y < rois[2] || cornersB[i].y > rois[3])
                continue;

        cv::circle(img_bgr, cornersB[i], 3, Scalar(0, 255, 0), -1);
        cv::line(img_bgr, cornersA[i], cornersB[i], cv::Scalar(0, 255, 0), 1);

        if (cornersB[i].x - cornersA[i].x < 0) nega++;
        else posi++;

        putText(img_bgr, std::to_string(fps[i].id), cornersB[i], cv::FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(0, 0, 255), 1, 8, 0);

//        std::stringstream text;
//        text << "(" << std::setprecision(3) << cornersB[i].x << ", " << std::setprecision(3) << cornersB[i].y << ")";
//        putText(img_bgr, text.str(), cornersB[i], cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1, 8, 0);
    }

    cv::imshow("source", img_bgr);
    waitKey();

    prev_img = cur_img;
    cornersA = cornersB;
}

int main(int argc, char **argv)
{
    if (argc == 1 || argc == 5)
    {
        if (argc == 5)
        {
            ROI = true;
            for (int i=0; i < argc - 1; i++)
                rois[i] = atoi(argv[i+1]);
        }
    }
    else
    {
        cout << "Number of arguments fails to be used." << endl;
        return -1;
    }

    // optical flow
    for (int i=0; i < img_amount; i++)
    {
        char img_loca[100];
        sprintf(img_loca, "%s/%03d.tif", dataset.c_str(), i+1);
        Mat cv_img = imread(img_loca, CV_LOAD_IMAGE_GRAYSCALE);
        if (!cv_img.data)
        {
            std::cerr<<"Couldn't read image!" << std::endl;
            return -1;
        }
        else
            std::cout << "Read image: " << img_loca << std::endl;


        opti_track_LK(cv_img, i);
    }

    // save in spreadsheet
    ofstream ofile;
    ofile.open (save_path, ios::trunc);
    if (!ofile) cout << "Couldn't open the file to save.";

    for (int i = 0; i < MAX_CORNERS; ++i)
    {
        if (ROI)
            if (fps[i].u[0] < rois[0] || fps[i].u[0] > rois[1] || fps[i].v[0] < rois[2] || fps[i].v[0] > rois[3])
                continue;

        ofile << fps[i].id << ",";
        for (int j=0; j < img_amount; j++)
        {
            if (ROI)
                if (fps[i].u[j] < rois[0] || fps[i].u[j] > rois[1] || fps[i].v[j] < rois[2] || fps[i].v[j] > rois[3])
                    continue;
            ofile << fps[i].u[j] << "," << fps[i].v[j] << ",";
        }
        ofile << "\n";
    }

    ofile.close();

    return 0;
}

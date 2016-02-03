#include <iostream>

#include <opencv2/opencv.hpp>

#include <System.h>


int main(int argc, char **argv) {
    if(argc != 4) {
        std::cerr << std::endl <<
            "Usage: ./mono_odm vocabulary settings video" <<
            std::endl;
        return 1;
    }

    cv::VideoCapture cap(argv[3]);
    if(!cap.isOpened()) {
        std::cerr << "Failed to load video: " << argv[3] << std::endl;
        return -1;
    }

    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

    cout << "Start processing video ..." << endl;

    int T_micro = 300000;
    cv::Mat im;
    for(int ni = 0;; ++ni){
        cap >> im;

        if(im.empty()) {
            break;
        }

        SLAM.TrackMonocular(im, ni * T_micro);

        usleep(T_micro);
    }

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

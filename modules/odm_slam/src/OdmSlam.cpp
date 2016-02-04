#include <iostream>

#include <opencv2/opencv.hpp>

#include <System.h>


int main(int argc, char **argv) {
    if(argc != 4) {
        std::cerr << std::endl <<
            "Usage: " << argv[0] << " vocabulary settings video" <<
            std::endl;
        return 1;
    }

    cv::VideoCapture cap(argv[3]);
    if(!cap.isOpened()) {
        std::cerr << "Failed to load video: " << argv[3] << std::endl;
        return -1;
    }

    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, false);

    cout << "Start processing video ..." << endl;

    double T = 0.1;  // Seconds between frames
    cv::Mat im;
    for(int ni = 0;; ++ni){
        // Get frame
        cap >> im;
        if(im.empty()) break;

        // Save 
        double timestamp = ni * T;
        keyframe_timestamps << ni << " " << timestamp << std::endl;



        SLAM.TrackMonocular(im, timestamp);

        usleep(int(T * 1e6));
    }

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

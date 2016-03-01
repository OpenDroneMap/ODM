#include <iostream>

#include <opencv2/opencv.hpp>

#include <System.h>
#include <Converter.h>


void SaveKeyFrameTrajectory(ORB_SLAM2::Map *map, const string &filename, const string &tracksfile) {
    std::cout << std::endl << "Saving keyframe trajectory to " << filename << " ..." << std::endl;

    vector<ORB_SLAM2::KeyFrame*> vpKFs = map->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM2::KeyFrame::lId);

    std::ofstream f;
    f.open(filename.c_str());
    f << fixed;

    std::ofstream fpoints;
    fpoints.open(tracksfile.c_str());
    fpoints << fixed;

    for(size_t i = 0; i < vpKFs.size(); i++) {
        ORB_SLAM2::KeyFrame* pKF = vpKFs[i];

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << std::endl;

        for (auto point : pKF->GetMapPoints()) {
            auto coords = point->GetWorldPos();
            fpoints << setprecision(6)
                    << pKF->mTimeStamp
                    << " " << point->mnId
                    << setprecision(7)
                    << " " << coords.at<float>(0, 0)
                    << " " << coords.at<float>(1, 0)
                    << " " << coords.at<float>(2, 0)
                    << std::endl;
        }
    }

    f.close();
    fpoints.close();
    std::cout << std::endl << "trajectory saved!" << std::endl;
}


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

    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

    usleep(10 * 1e6);

    std::cout << "Start processing video ..." << std::endl;

    double T = 0.1;  // Seconds between frames
    cv::Mat im;
    int num_frames = cap.get(CV_CAP_PROP_FRAME_COUNT);
    for(int ni = 0;; ++ni){
        std::cout << "processing frame " << ni << "/" << num_frames << std::endl;
        // Get frame
        bool res = false;
        for (int trial = 0; !res && trial < 20; ++trial) {
            std::cout << "trial " << trial << std::endl;
            res = cap.read(im);
        }
        if(!res) break;

        double timestamp = ni * T;

        SLAM.TrackMonocular(im, timestamp);

        //usleep(int(T * 1e6));
    }

    SLAM.Shutdown();
    SaveKeyFrameTrajectory(SLAM.GetMap(), "KeyFrameTrajectory.txt", "MapPoints.txt");

    return 0;
}

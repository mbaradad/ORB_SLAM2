#include "KeyFrame.h"
#include "Converter.h"
#include "System.h"


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
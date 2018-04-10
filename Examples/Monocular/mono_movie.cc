/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free softpare: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include"System.h"
#include"Tracking.h"
#include"TrackerConfig.h"

#include"utils.cpp"

#include <ftw.h>

// do cleaner in a final version, but does the trick
vector<string> globalFiles;

int AnalizeDirectoryElement (const char *fpath,
                             const struct stat *sb,
                             int tflag,
                             struct FTW *ftwbuf) {
  if (tflag == FTW_F) {
    std::string strFileName(fpath);
    globalFiles.push_back(strFileName);
  }
  return 0;
}

bool endsWidth (std::string const &fullString, std::string const &ending) {
  if (fullString.length() >= ending.length()) {
    return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
  } else {
    return false;
  }
}

bool comp_gt(string i, string j) { return i > j; }

void listJpgImages(const char *pchFileName, vector<string> &vstrImageFilenames) {

  int nFlags = 0;

  if (nftw(pchFileName, AnalizeDirectoryElement, 20, nFlags) == -1) {
    perror("nftw");
  }

  for (uint i = 1; i < globalFiles.size(); i++){
    if (endsWidth(globalFiles[i], ".jpg"))
      vstrImageFilenames.push_back(globalFiles[i]);
  }

  std::sort(vstrImageFilenames.begin(), vstrImageFilenames.end());
  //std::sort(vstrImageFilenames.begin(), vstrImageFilenames.end(), comp_gt);
}

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

void slamOnImageFilenamesList(vector<string> &vstrImageFilenames, const string &strVocFile, const string &strSettingsFile, vector<double> &vTimestamps,
                              ORB_SLAM2::TrackerConfig &config){
  int nImages = vstrImageFilenames.size();

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM2::ORBVocabulary* mpVocabulary = ORB_SLAM2::System::createOrbVocabulary(strVocFile);

  ORB_SLAM2::System SLAM(mpVocabulary, strSettingsFile, config, ORB_SLAM2::System::MONOCULAR,true);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;

  // Main loop
  cv::Mat im;
  int firstFrame = -1;
  int n_lost = 0;
  int good_frames = 0;

  //#ni=240245
  for(int ni=0; ni<nImages; ni++)
  {
    // Read image from file
    im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
    double tframe = vTimestamps[ni];

    if(im.empty())
    {
      cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
      return;
    }
    cout << "Processing frame: " << ni << " of " << nImages << endl;

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

    // Pass the image to the SLAM system
    //SLAM.TrackMonocular(im,tframe);

    SLAM.SetFrameForTrack(im, tframe);
    std::vector<cv::Mat> keys = SLAM.GetCurrentFrame().GetMVKeys();
    cv::Mat descriptors = SLAM.GetCurrentFrame().GetMVDescriptors();
    std::vector<bool> keysToDrop;
    for (uint i= 0; i < keys.size(); ++i){
      keysToDrop.push_back(false);
    }
    SLAM.GetCurrentFrame().DropMVKeys(keysToDrop);
    SLAM.GetCurrentFrame().SetMVDescriptors(descriptors);
    SLAM.TrackCurrentFrame();

    int state = SLAM.GetTrackingState();
    cout << "State: " << state << endl;
    if (state == 2 and firstFrame == -1){
      firstFrame = ni;
    }else if(state == 2){
      good_frames = good_frames + 1;
    }else if (state == 3){
      cout << "Lost: " << n_lost;
      n_lost++;
    }
    if (good_frames == 10){
      cout << SLAM.GetAllPoses()[0].first << endl;
    }

    if (state == 3 and n_lost > 0){
      // SaveKeyFrameTrajectory(SLAM.GetMap(), "KeyFrameTrajectory.txt", "MapPoints" + to_string(firstFrame) + "_" + to_string(ni) + ".txt");

      // SLAM.SaveTrajectoryTUM("output_" + to_string(firstFrame) + "_" + to_string(ni));
      SLAM.Reset();
      firstFrame = -1;
      n_lost = 0;
    }
#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    vTimesTrack[ni]=ttrack;
  }
  // Stop all threads
  SLAM.Shutdown();

  // Tracking time statistics
  sort(vTimesTrack.begin(),vTimesTrack.end());
  float totaltime = 0;
  for(int ni=67000; ni<nImages; ni++)
  {
    totaltime+=vTimesTrack[ni];
  }
  cout << "-------" << endl << endl;
  cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
  cout << "mean tracking time: " << totaltime/nImages << endl;

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  SLAM.SaveTrajectoryKITTI("FrameTrajectory.txt");

}

int predefined_main(){
  // Retrieve paths to images
  vector<string> vstrImageFilenames;
  vector<double> vTimestamps;

  LoadImages(string("../local_data/russian_ark_bad_init_sequence"), vstrImageFilenames, vTimestamps);

  ORB_SLAM2::TrackerConfig config = ORB_SLAM2::TrackerConfig::defaultSettings();
  config.Camera_fx = 100;
  config.Camera_fy = 100;

  slamOnImageFilenamesList(vstrImageFilenames, "Vocabulary/ORBvoc.txt", "Examples/Monocular/movie.yaml", vTimestamps, config);
  return 1;
}

int main(int argc, char **argv)
{
  if(argc != 4)
  {
    cout << "No or incorrect number of arguments provided, using predefined main" << endl;
    return predefined_main();
  }

  // Retrieve paths to images
  vector<string> vstrImageFilenames;
  vector<double> vTimestamps;
  LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);

  LoadImages(string("../local_data/russian_ark_bad_init_sequence"), vstrImageFilenames, vTimestamps);

  ORB_SLAM2::TrackerConfig config = ORB_SLAM2::TrackerConfig::defaultSettings();
  config.Camera_fx = 550;
  config.Camera_fy = 550;

  slamOnImageFilenamesList(vstrImageFilenames, argv[1],argv[2], vTimestamps, config);
  return 1;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
  listJpgImages(strPathToSequence.c_str(), vstrImageFilenames);

  const int nFiles = vstrImageFilenames.size();

  for(int i=0; i<nFiles; i++)
  {
    vTimestamps.push_back(i/24.0);
  }
}

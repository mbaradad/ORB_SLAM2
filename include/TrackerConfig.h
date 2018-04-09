/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
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

#ifndef TRACKERCONFIG_H
#define TRACKERCONFIG_H

namespace ORB_SLAM2
{

class TrackerConfig
{
public:
    TrackerConfig(){}

    float Camera_fx;
    float Camera_fy;
    float Camera_cx;
    float Camera_cy;

    float Camera_k1;
    float Camera_k2;
    float Camera_p1;
    float Camera_p2;
    float Camera_k3;

    float Camera_bf;
    float Camera_fps;

    int Camera_RGB;

    int ORBextractor_nFeatures;
    float ORBextractor_scaleFactor;
    int ORBextractor_nLevels;
    int ORBextractor_iniThFAST;
    int ORBextractor_minThFAST;

    static TrackerConfig defaultSettings() {
      TrackerConfig config = TrackerConfig();
      config.Camera_fx = 550;
      config.Camera_fy = 550;
      config.Camera_cx = 360;
      config.Camera_cy = 202.5;

      config.Camera_k1 = 0;
      config.Camera_k2 = 0;
      config.Camera_p1 = 0;
      config.Camera_p2 = 0;
      config.Camera_k3 = 0;

      config.Camera_bf = 0;
      config.Camera_fps = 24.0;

      config.Camera_RGB = 1;

      config.ORBextractor_nFeatures = 2000;
      config.ORBextractor_scaleFactor = 1.2;
      config.ORBextractor_nLevels = 8;
      config.ORBextractor_iniThFAST = 20;
      config.ORBextractor_minThFAST = 7;

      return config;
    }
};

}// namespace ORB_SLAM

#endif // CONVERTER_H

/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <linux/videodev2.h>
#include <sys/ioctl.h>

#include "MER_driver.h"

namespace roborts_camera {
MERDriver::MERDriver(CameraInfo camera_info):CameraBase(camera_info)
{
  this->MER_camera = std::make_unique<GxCamera>();
}

void MERDriver::StartReadCamera(cv::Mat &img) {
  if(!camera_initialized_){
    // camera_info_.cap_handle.open(camera_info_.camera_path);
    // SetCameraExposure(camera_info_.camera_path, camera_info_.exposure_value);
    // ROS_ASSERT_MSG(camera_info_.cap_handle.isOpened(), "Cannot open %s .", cameras_[camera_num].video_path.c_str());

    MER_camera->camStatus = MER_camera->CameraInitOps();
    if(MER_camera->camStatus == GX_STATUS_SUCCESS)
      printf("Success init MER_camera.\n");
    MER_camera->camStatus = MER_camera->StreamOn();
    if(MER_camera->camStatus == GX_STATUS_SUCCESS)
      printf("Success stream on.\n");
    MER_camera->camStatus = MER_camera->SetExposureTime(3500);
    MER_camera->camStatus = MER_camera->SetFrameRate(100);

    camera_initialized_ = true;
  }
  else {
    MER_camera->GetColorImg(img);
  }
}

void MERDriver::StopReadCamera() {
  //TODO: To be implemented
}

void MERDriver::SetCameraExposure(std::string id, int val)
{
  // int cam_fd;
  // if ((cam_fd = open(id.c_str(), O_RDWR)) == -1) {
  //   std::cerr << "Camera open error" << std::endl;
  // }

  // struct v4l2_control control_s;
  // control_s.id = V4L2_CID_AUTO_WHITE_BALANCE;
  // control_s.value = camera_info_.auto_white_balance;
  // ioctl(cam_fd, VIDIOC_S_CTRL, &control_s);

  // control_s.id = V4L2_CID_EXPOSURE_AUTO;
  // control_s.value = V4L2_EXPOSURE_MANUAL;
  // ioctl(cam_fd, VIDIOC_S_CTRL, &control_s);

  // // Set exposure value
  // control_s.id = V4L2_CID_EXPOSURE_ABSOLUTE;
  // control_s.value = val;
  // ioctl(cam_fd, VIDIOC_S_CTRL, &control_s);
  // close(cam_fd);
}

MERDriver::~MERDriver() {
}

} //namespace roborts_camera

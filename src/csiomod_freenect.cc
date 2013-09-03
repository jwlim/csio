// csiomod_freenect.cc
//
// Authors: Jongwoo Lim (jongwoo.lim@gmail.com)
//

#include "csio_stream.h"

#include <fstream>
#include <iostream>
#include <map>
#include <vector>
#include <glog/logging.h>
#include <gflags/gflags.h>

#include "libfreenect/libfreenect.h"

using namespace std;

DEFINE_string(out, "", "File path for csio::OutputStream (- for stdout).");
DEFINE_int32(kinect_devno, 0, "Kinect device number to use.");
DEFINE_int32(kinect_tilt_angle, 0, "Kinect tilt angle (-30~30 deg.).");

class Freenect2CSIO {
 public:
  Freenect2CSIO() : context_(NULL), dev_(NULL), csio_os_(NULL) {}
  ~Freenect2CSIO() { Close(); }

  void SetCSIOOutstream(csio::OutputStream* csio_os,
                        int csio_chid_depth, int csio_chid_rgb) {
    CHECK_NOTNULL(csio_os);
    csio_os_ = csio_os;
    csio_chid_depth_ = csio_chid_depth;
    csio_chid_rgb_ = csio_chid_rgb;
  }

  bool Open(int devno);

  bool Start() {
    if (context_ == NULL || dev_ == NULL) return false;
    freenect_start_depth(dev_);
    freenect_start_video(dev_);
    return true;
  }

  bool Process() {
    if (context_ == NULL || dev_ == NULL) return false;
    return freenect_process_events(context_) >= 0;
  }

  bool Stop() {
    if (context_ == NULL || dev_ == NULL) return false;
    freenect_stop_depth(dev_);
    freenect_stop_video(dev_);
    return true;
  }

  void Close() {
    Stop();
    freenect_dev_to_obj_map_.erase(dev_);
    if (dev_) freenect_close_device(dev_);
    if (context_) freenect_shutdown(context_);
    dev_ = NULL;
    context_ = NULL;
  }

 private:
  static void DepthCallback(freenect_device* dev, void* depth_buf,
                            uint32_t timestamp);
  static void RGBCallback(freenect_device* dev, void* rgb_buf,
                          uint32_t timestamp);

  freenect_context* context_;
  freenect_device* dev_;
  csio::OutputStream* csio_os_;
  int csio_chid_depth_, csio_chid_rgb_;
  static map<freenect_device*, Freenect2CSIO*> freenect_dev_to_obj_map_;
};

map<freenect_device*, Freenect2CSIO*> Freenect2CSIO::freenect_dev_to_obj_map_;

bool Freenect2CSIO::Open(int devno) {
  LOG(INFO) << "trying to open the kinect " << devno << ".";
  if (freenect_init(&context_, NULL) < 0) {
    LOG(ERROR) << "freenect_init failed.";
    return false;
  }
  freenect_select_subdevices(
      context_,
      (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

  int num_devices = freenect_num_devices(context_);
  LOG(INFO) << "freenect: " << num_devices << " devices found.";

  if (num_devices <= 0) {
    LOG(ERROR) << "no kinect device found.";
    Close();
    return false;
  }
  if (freenect_open_device(context_, &dev_, devno) < 0) {
    LOG(ERROR) << "freenect_open_device(" << devno << "/" << num_devices
        << ") failed.";
    Close();
    return false;
  }
  if (-30 <= FLAGS_kinect_tilt_angle && FLAGS_kinect_tilt_angle <= 30) {
    freenect_set_tilt_degs(dev_, FLAGS_kinect_tilt_angle);
  } else {
    LOG(ERROR) << "invalid kinect_tilt_angle: " << FLAGS_kinect_tilt_angle;
  }
  freenect_set_led(dev_, LED_RED);

  freenect_set_depth_callback(dev_, Freenect2CSIO::DepthCallback);
  freenect_set_video_callback(dev_, Freenect2CSIO::RGBCallback);
  freenect_set_video_mode(dev_,
                          freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM,
                                                   FREENECT_VIDEO_RGB));
  freenect_set_depth_mode(dev_,
                          freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM,
                                                   FREENECT_DEPTH_REGISTERED));
//                                                   FREENECT_DEPTH_11BIT));
  freenect_dev_to_obj_map_.insert(make_pair(dev_, this));
  return true;
}

void Freenect2CSIO::DepthCallback(freenect_device* dev, void* depth_buf,
                             uint32_t timestamp) {
  map<freenect_device*, Freenect2CSIO*>::iterator it =
      freenect_dev_to_obj_map_.find(dev);
  if (it == freenect_dev_to_obj_map_.end()) return;

  Freenect2CSIO& freenect = *it->second;
  if (freenect.csio_os_ == NULL) return;
  freenect.csio_os_->Push(0, static_cast<char*>(depth_buf), 640 * 480 * 2);
  LOG(INFO) << "push depth_buf.";
}

void Freenect2CSIO::RGBCallback(freenect_device* dev, void* rgb_buf,
                           uint32_t timestamp) {
  map<freenect_device*, Freenect2CSIO*>::iterator it =
      freenect_dev_to_obj_map_.find(dev);
  if (it == freenect_dev_to_obj_map_.end()) return;

  Freenect2CSIO& freenect = *it->second;
  if (freenect.csio_os_ == NULL) return;
  freenect.csio_os_->Push(1, static_cast<char*>(rgb_buf), 640 * 480 * 3);
  LOG(INFO) << "push rgb_buf.";
}


int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  // Open a Kinect handle.
  Freenect2CSIO freenect2csio;
  if (freenect2csio.Open(FLAGS_kinect_devno) == false) return -1;

  vector<csio::ChannelInfo> channels;
  channels.push_back(csio::ChannelInfo(
          0, "image/x-csio-raw;pixel=gray16;size=640x480",
          "Kinect 16-bit depth image"));
  channels.push_back(csio::ChannelInfo(
          1, "image/x-csio-raw;pixel=rgb8;width=640;height=480",
          "Kinect RGB image"));
  map<string, string> config;

  LOG(INFO) << "setting up csio::OutputStream (out=" << FLAGS_out << ").";
  csio::OutputStream csio_os;
  if (csio_os.Setup(channels, config, FLAGS_out) == false) {
    LOG(ERROR) << "failed to open csio::OutputStream, out=" << FLAGS_out;
    return -1;
  }
  LOG(INFO) << "setup csio::OutputStream (out=" << FLAGS_out << ") complete.";
  freenect2csio.SetCSIOOutstream(&csio_os, 0, 1);

  LOG(INFO) << "starting freenect.";
  if (freenect2csio.Start() == false) {
    LOG(ERROR) << "failed to start freenect.";
    return -1;
  }
  LOG(INFO) << "enter processing.";
  while (freenect2csio.Process()) ;
  LOG(INFO) << "finished processing.";
  if (freenect2csio.Stop() == false) {
    LOG(ERROR) << "failed to start freenect.";
    return -1;
  }
  freenect2csio.Close();

  return 0;
}


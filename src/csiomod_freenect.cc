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
#include <pthread.h>
#include <cmath>

#include "libfreenect/libfreenect.hpp"

using namespace std;

DEFINE_string(out, "", "File path for csio::OutputStream (- for stdout).");
DEFINE_int32(kinect_devno, 0, "Kinect device number to use.");
DEFINE_int32(kinect_tilt_angle, 0, "Kinect tilt angle (-30~30 deg.).");

class myMutex {
  public:
    myMutex() {
      pthread_mutex_init( &m_mutex, NULL );
    }
    void lock() {
      pthread_mutex_lock( &m_mutex );
    }
    void unlock() {
      pthread_mutex_unlock( &m_mutex );
    }
  private:
    pthread_mutex_t m_mutex;
};

class FreenectWrapper : public Freenect::FreenectDevice {
 public:
  FreenectWrapper(freenect_context *_ctx, int _index)
      : Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(FREENECT_DEPTH_11BIT),
      m_buffer_rgb(FREENECT_VIDEO_RGB), m_gamma(2048), m_new_rgb_frame(false),
      m_new_depth_frame(false) {
      //m_new_depth_frame(false), depthMat(Size(640,480),CV_16UC1),
      //rgbMat(Size(640,480), CV_8UC3, Scalar(0)),
      //ownMat(Size(640,480),CV_8UC3,Scalar(0)) {
      
      for( unsigned int i = 0 ; i < 2048 ; i++) {
        float v = i/2048.0;
        v = std::pow(v, 3)* 6;
        m_gamma[i] = v*6*256;
      }

      m_rgb_data = new uint8_t[640*480*3];
      m_depth_data = new uint16_t[640*480];
  }

  // Do not call directly even in child
  void VideoCallback(void* _rgb, uint32_t timestamp) {
    LOG(INFO) << "RGB callback";
    m_rgb_mutex.lock();
    uint8_t* rgb = static_cast<uint8_t*>(_rgb);
    //rgbMat.data = rgb;
    memcpy(m_rgb_data, rgb, 640*480*3);
    m_new_rgb_frame = true;
    m_rgb_mutex.unlock();
  };
    
    // Do not call directly even in child
  void DepthCallback(void* _depth, uint32_t timestamp) {
    std::cout << "Depth callback" << std::endl;
    m_depth_mutex.lock();
    uint16_t* depth = static_cast<uint16_t*>(_depth);
    memcpy(m_depth_data, depth, 640*480*sizeof(uint16_t));
    //depthMat.data = (uchar*) depth;
    m_new_depth_frame = true;
    m_depth_mutex.unlock();
  }
    
  uint8_t* getVideo() {
    m_rgb_mutex.lock();
    if(m_new_rgb_frame) {
      m_new_rgb_frame = false;
      m_rgb_mutex.unlock();
      return m_rgb_data;
    } else {
      m_rgb_mutex.unlock();
      return NULL;
    }
  }
    
  uint16_t* getDepth() {
    m_depth_mutex.lock();
    if(m_new_depth_frame) {
      m_new_depth_frame = false;
      m_depth_mutex.unlock();
      return m_depth_data;
    } else {
      m_depth_mutex.unlock();
      return NULL;
    }
  } 
  private:
    std::vector<uint8_t> m_buffer_depth;
    std::vector<uint8_t> m_buffer_rgb;
    std::vector<uint16_t> m_gamma;
    myMutex m_rgb_mutex;
    myMutex m_depth_mutex;
    bool m_new_rgb_frame;
    bool m_new_depth_frame;
    uint8_t *m_rgb_data;
    uint16_t *m_depth_data; 
};

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

  char* m_buffer_rgb, m_buffer_depth;
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

  // Obtain resolution and set buffers.


  return true;
}

void Freenect2CSIO::DepthCallback(freenect_device* dev, void* depth_buf,
                             uint32_t timestamp) {
  map<freenect_device*, Freenect2CSIO*>::iterator it =
      freenect_dev_to_obj_map_.find(dev);
  if (it == freenect_dev_to_obj_map_.end()) return;

  Freenect2CSIO& freenect = *it->second;
  if (freenect.csio_os_ == NULL) return;
  freenect.csio_os_->Push(1, static_cast<char*>(depth_buf), 640 * 480 * 2);
  LOG(INFO) << "push depth_buf.";
}

void Freenect2CSIO::RGBCallback(freenect_device* dev, void* rgb_buf,
                           uint32_t timestamp) {
  map<freenect_device*, Freenect2CSIO*>::iterator it =
      freenect_dev_to_obj_map_.find(dev);
  if (it == freenect_dev_to_obj_map_.end()) return;

  Freenect2CSIO& freenect = *it->second;
  if (freenect.csio_os_ == NULL) return;
  freenect.csio_os_->Push(0, static_cast<char*>(rgb_buf), 640 * 480 * 3);
  LOG(INFO) << "push rgb_buf.";
}

int main(int argc, char** argv){
  bool die(false);

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  Freenect::Freenect freenect;
  FreenectWrapper& device = freenect.createDevice<FreenectWrapper>(0);
  device.startVideo();
  device.startDepth();

  vector<csio::ChannelInfo> channels;
  channels.push_back(csio::ChannelInfo(
          0, "image/x-csio-raw;pixel=rgb8;width=640;height=480",
          "Kinect RGB image"));
  channels.push_back(csio::ChannelInfo(
          1, "image/x-csio-raw;pixel=gray16;size=640x480",
          "Kinect 16-bit depth image"));
  map<string, string> config;

  LOG(INFO) << "setting up csio::OutputStream (out=" << FLAGS_out << ").";
  csio::OutputStream csio_os;
  if (csio_os.Setup(channels, config, FLAGS_out) == false) {
    LOG(ERROR) << "failed to open csio::OutputStream, out=" << FLAGS_out;
    return -1;
  }

  while(!die) {
    uint8_t* rgb = device.getVideo();
    uint16_t* depth = device.getDepth();
    csio_os.PushSyncMark(2);
    csio_os.Push(0, static_cast<uint8_t*>(rgb), 640 * 480 * 3);
    csio_os.Push(1, static_cast<uint16_t*>(depth), 640 * 480);
  }

  device.stopVideo();
  device.stopDepth();
  return 0;
}

#if 0
int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  // Open a Kinect handle.
  Freenect2CSIO freenect2csio;
  if (freenect2csio.Open(FLAGS_kinect_devno) == false) return -1;

  vector<csio::ChannelInfo> channels;
  channels.push_back(csio::ChannelInfo(
          0, "image/x-csio-raw;pixel=rgb8;width=640;height=480",
          "Kinect RGB image"));
  channels.push_back(csio::ChannelInfo(
          1, "image/x-csio-raw;pixel=gray16;size=640x480",
          "Kinect 16-bit depth image"));
  map<string, string> config;

  LOG(INFO) << "setting up csio::OutputStream (out=" << FLAGS_out << ").";
  csio::OutputStream csio_os;
  if (csio_os.Setup(channels, config, FLAGS_out) == false) {
    LOG(ERROR) << "failed to open csio::OutputStream, out=" << FLAGS_out;
    return -1;
  }
  csio_os.PushSyncMark(2);
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
#endif

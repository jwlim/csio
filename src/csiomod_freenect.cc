// csiomod_freenect.cc
//
// Authors: Jongwoo Lim (jongwoo.lim@gmail.com)
//

#include "csio_stream.h"
#include "csio_frame_parser.h"

#include <fstream>
#include <iostream>
#include <map>
#include <vector>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <pthread.h>
#include <cmath>

#ifdef APPLE
#include "libfreenect/libfreenect.hpp"
#else
#include <libfreenect.hpp>
#endif

using namespace std;

DEFINE_string(out, "-", "File path for csio::OutputStream (- for stdout).");
DEFINE_int32(kinect_devno, 0, "Kinect device number to use.");
DEFINE_int32(kinect_tilt_angle, 0, "Kinect tilt angle (-30~30 deg.).");

class Mutex {
public:
        Mutex() {
                pthread_mutex_init( &m_mutex, NULL );
        }
        void lock() {
                pthread_mutex_lock( &m_mutex );
        }
        void unlock() {
                pthread_mutex_unlock( &m_mutex );
        }

        class ScopedLock
        {
                Mutex & _mutex;
        public:
                ScopedLock(Mutex & mutex)
                        : _mutex(mutex)
                {
                        _mutex.lock();
                }
                ~ScopedLock()
                {
                        _mutex.unlock();
                }
        };
private:
        pthread_mutex_t m_mutex;
};


class FreenectWrapper : public Freenect::FreenectDevice {
 public:
  FreenectWrapper(freenect_context *_ctx, int _index)
      : Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(FREENECT_DEPTH_11BIT),
      m_buffer_rgb(FREENECT_VIDEO_RGB), m_gamma(2048), m_new_rgb_frame(false),
      m_new_depth_frame(false) {
      
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
    Mutex::ScopedLock lock(m_rgb_mutex);
    VLOG(1) << "In video callback";
    rgb = static_cast<uint8_t*>(_rgb);
    m_new_rgb_frame = true;
  }
    
  // Do not call directly even in child
  void DepthCallback(void* _depth, uint32_t timestamp) {
    Mutex::ScopedLock lock(m_depth_mutex);
    VLOG(1) << "In depth callback";
    depth = static_cast<uint8_t*>(_depth);
    //depth = (uint8_t*)static_cast<uint16_t*>(_depth);
    m_new_depth_frame = true;
  }
    
  bool getVideo(uint8_t* output) {
    m_rgb_mutex.lock();
    if(m_new_rgb_frame) {
      memcpy(output, rgb, 640*480*3);
      m_new_rgb_frame = false;
      m_rgb_mutex.unlock();
      return true;
    } else {
      m_rgb_mutex.unlock();
      return false;
    }
  }
    
  bool getDepth(uint8_t* output) {
    m_depth_mutex.lock();
    if(m_new_depth_frame) {
      memcpy(output, depth, 640*480*2);
      m_new_depth_frame = false;
      m_depth_mutex.unlock();
      return true;
    } else {
      m_depth_mutex.unlock();
      return false;
    }
  }

  private:
    std::vector<uint8_t> m_buffer_depth;
    std::vector<uint8_t> m_buffer_rgb;
    std::vector<uint16_t> m_gamma;
    Mutex m_rgb_mutex;
    Mutex m_depth_mutex;
    bool m_new_rgb_frame;
    bool m_new_depth_frame;

    uint8_t *m_rgb_data;
    uint16_t *m_depth_data; 

    uint8_t *rgb;
    uint8_t *depth;
};

int main(int argc, char** argv){

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  Freenect::Freenect freenect;
  FreenectWrapper* device = &freenect.createDevice<FreenectWrapper>(0);

  vector<csio::ChannelInfo> channels;
  channels.push_back(csio::ChannelInfo(
          0, csio::MakeImageTypeStr("rgb8", 640, 480), "rgb"));
  channels.push_back(csio::ChannelInfo(
          1, csio::MakeImageTypeStr("gray16", 640, 480), "depth"));
  map<string, string> config;
  LOG(INFO) << "setting up csio::OutputStream (out=" << FLAGS_out << ").";
  csio::OutputStream csio_os;
  if (csio_os.Setup(channels, config, FLAGS_out) == true) {
    LOG(INFO) << "csio::OutputStream opened (out=" << FLAGS_out << "),"
      << "w:640" << "h:480";
  } else {
    return -1;
  }

  device->setDepthFormat(FREENECT_DEPTH_REGISTERED);
  device->startVideo();
  device->startDepth();

  bool die(false);

  uint8_t *rgb = new uint8_t[640*480*3];
  uint8_t *depth = new uint8_t[640*480*2];

  while(!die) {
    if (device->getVideo(rgb) && device->getDepth(depth)) {
      csio_os.PushSyncMark(2);
      csio_os.Push(0, static_cast<uint8_t*>(rgb), 640 * 480 * 3);
      csio_os.Push(1, static_cast<uint8_t*>(depth), 640 * 480 * 2);
    }
  }

  device->stopVideo();
  device->stopDepth();
  return 0;
}

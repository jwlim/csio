#include "csiomod_camera1394.h"
#include "csio_stream.h"
#include "csio_frame_parser.h"

#include <fstream>
#include <iostream>
#include <gflags/gflags.h>
#include <glog/logging.h>

using namespace std;

DEFINE_string(out, "-", "File path for csio::OutputStream (- for stdout).");
DEFINE_bool(listdev, false, "Print list of available devices.");
DEFINE_int32(devno, 0, "Device number to use.");
DEFINE_string(modestr, "640x480.rgb8", 
  "Device mode (\"640x480.rgb8\" or \"fmt7.1.rgb8.644.482.0.0\")");
DEFINE_string(fratestr, "modeset", "Frame rate for camera.");

void ListDev(dc1394_t *d) {
  dc1394error_t err;
  dc1394camera_list_t *list;

  if (dc1394_camera_enumerate(d, &list) != DC1394_SUCCESS) {
    LOG(ERROR) << "Failed to enumerate cameras";
    exit(1);
  }

  if (list->num == 0) {
    LOG(ERROR) << "No cameras found";
    exit(1);
  }

  vector<dc1394camera_t*> camera_list(list->num);
  printf("# Camera list [%d]\n", camera_list.size());
  for (uint32_t i = 0; i<list->num; i++) {
    dc1394camera_t *camera = dc1394_camera_new(d, list->ids[i].guid);
    if (!camera) {
      LOG(ERROR) << "Failed to initialize camera with guid %llx",
        list->ids[i].guid;
      exit(1);
    }
    camera_list.push_back(camera);
    printf("%d. vendor: '%s'\n   model: '%s'\n   id: '%08lx.%08lx'\n",
      i, camera->vendor, camera->model, (uint32_t)(camera->guid>>32),
      camera->guid);
  }

  int input;
  printf("\nInput cam number : ");
  scanf("%d", &input);
  if (input >= list->num || input < 0) {
    printf("Error : out of index.\n");
  }

  dc1394camera_t *selected_cam = *(camera_list.begin()+1+input);
  vector<dc1394camera_t*>::iterator it;
  for (it = camera_list.begin()+1; it!=camera_list.end(); it++) {
    if (*it != selected_cam) {
      dc1394_camera_free(*it);
    }
  }
  dc1394_camera_free_list (list);
  
  Camera1394 cam1394(selected_cam);
  printf("\n# Features of '%s'\n", cam1394.cam->model);
  cam1394.ListAvailableFeatures();

  printf("\n# Supported modes\n");
  cam1394.ListSupportedModes();

  dc1394_camera_free(cam1394.cam);
  dc1394_free (d);
}

dc1394camera_t* GetDevice(dc1394_t *d, int32_t devno) {
  dc1394error_t err;
  dc1394camera_list_t *list;

  if (dc1394_camera_enumerate(d, &list) != DC1394_SUCCESS) {
    LOG(ERROR) << "Failed to enumerate cameras";
    exit(1);
  }

  if (list->num == 0) {
    LOG(ERROR) << "No cameras found";
    exit(1);
  }

  if (devno > list->num - 1) {
    LOG(ERROR) << "Device number is out of index.";
    exit(1);
  }
  
  dc1394camera_t *camera = dc1394_camera_new(d, list->ids[devno].guid);
  if (!camera) {
    LOG(ERROR) << "Failed to initialize camera with guid %llx",
      list->ids[devno].guid;
    exit(1);
  }
  dc1394_camera_free_list (list);

  return camera;
}

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  
  dc1394error_t err;
  dc1394_t *d = dc1394_new ();
  if (!d)
    return 1;

  if (FLAGS_listdev) {
    ListDev(d);
    return 0;
  } 

  Camera1394 cam1394 = Camera1394(GetDevice(d, FLAGS_devno));
  LOG(INFO) << "Camera vendor: '" << cam1394.cam->vendor << "', model: '"
    << cam1394.cam->model << "'";

  if (!cam1394.Setup(FLAGS_modestr, FLAGS_fratestr)) {
    dc1394_free(d);
    return 1;
  }
  LOG(INFO) << "Successfully setup the camera";
  if (cam1394.IsFmt7()) {
    LOG(INFO) << "mode='Format7." << cam1394.mode - DC1394_VIDEO_MODE_FORMAT7_0
      << "', coding='" << cam1394.codingstr 
      << "', size=" << cam1394.w << "x" << cam1394.h
      << "." << cam1394.x << "@" << cam1394.y;
  } else {
    LOG(INFO) << "mode='" << FLAGS_modestr 
      << "', frame rate='" << FLAGS_fratestr << "'";  
  }

  vector<csio::ChannelInfo> channels;
  channels.push_back(csio::ChannelInfo(
    0, csio::MakeImageTypeStr(cam1394.codingstr, cam1394.w, cam1394.h), "1394"));
  map<string, string> config;

  csio::OutputStream csio_os;
  if (csio_os.Setup(channels, config, FLAGS_out)) {
    LOG(INFO) << "csio::OutputStream opened (out=" << FLAGS_out << "),"
      << "w:" << cam1394.w << "h:" << cam1394.h;
  } else {
    dc1394_free(d);
    return 1;
  }

  if (!cam1394.StartCapture()) {
    dc1394_free(d);
    return 1;
  }
  LOG(INFO) << "Start capturing";

  dc1394video_frame_t *frame;
  bool die(false);
  while(!die){
    if (!cam1394.FetchFrame(&frame)) {
      dc1394_free(d);
      return 1;
    } 
    csio_os.Push(0, frame->image , frame->image_bytes);
    if (!cam1394.DropFrame(&frame)) {
      dc1394_free(d);
      return 1;
    }
  }

  if (!cam1394.StopCapture()) {
    dc1394_free(d);
    return 1;
  }
  LOG(INFO) << "Stop capturing";
  
  dc1394_capture_stop(cam1394.cam);
  dc1394_camera_free(cam1394.cam);
  dc1394_free (d);

  return 0;
}

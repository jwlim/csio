CSIO : C Standard IO library with GUI
====

## Building ##

```
sudo apt-get install freeglut3-dev subversion libpng12-dev libxmu-dev libxi-dev libjpeg-dev
cd csio
mkdir build
cd build
cmake ..
make -j
```

## Simple PLY Viewer ##

  * Run the program by passing the ply file as the programs argument eg.
  *
  * ./simple_ply_viewer result.ply
  *
  * Use the mouse to navigate, all three mouse buttons control zoom/pan/rotate.
  * Press ESC to exit the program, it will output a playback.bin file. You must
  * press ESC to save the OpenGL view states to generate the animation frames.
  * Now run the program again as follows
  *
  * ./simple_ply_viewer result.ply playback.bin
  *
  * This will dump a whole bunch of frame-xxxx.png images in the current
  * directory. I use mencoder to generate the video eg.
  *
  * mencoder mf://*.png -fps 25 -ovc x264 -o video.avi

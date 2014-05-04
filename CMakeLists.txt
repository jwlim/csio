#
# CSIO : The C Standard I/O Library for Computer Vision Research
#
# Author : Jongwoo Lim, Hyon Lim
#
cmake_minimum_required(VERSION 2.6)
project(csio)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
    "${PROJECT_SOURCE_DIR}/cmake_modules/")

# make release version
set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE release)
set(CMAKE_VERBOSE_MAKEFILE yes)

enable_testing()

# Default locations to search for on various platforms.
LIST(APPEND SEARCH_LIBS /usr/lib)
LIST(APPEND SEARCH_LIBS /usr/local/lib)
LIST(APPEND SEARCH_LIBS /opt/local/lib)

LIST(APPEND SEARCH_HEADERS /usr/include)
LIST(APPEND SEARCH_HEADERS /usr/local/include)
LIST(APPEND SEARCH_HEADERS /opt/local/include)

# Google libraries
message("-- Check for Google Log")
find_library(GLOG_LIB NAMES glog PATHS ${SEARCH_LIBS})
find_path(GLOG_INCLUDE NAMES glog/logging.h PATHS ${SEARCH_HEADERS})

message("-- Check for Google Flags")
find_library(GFLAGS_LIB NAMES gflags PATHS ${SEARCH_LIBS})
find_path(GFLAGS_INCLUDE NAMES gflags/gflags.h PATHS ${SEARCH_HEADERS})

message("-- Add dependencies for Google Test. You need internet connection.")
add_subdirectory(${CMAKE_MODULE_PATH}/gtest)
set(GTEST_LIBS ${GTEST_LIBS_DIR}/libgtest.a ${GTEST_LIBS_DIR}/libgtest_main.a)
set(GOOGLE_LIBRARIES ${GFLAGS_LIB} ${GLOG_LIB})

# Other libraries
MESSAGE("-- Check for PNG ")
find_package(PNG)

message("-- Check for libfreenect (optional)")
find_library(FREENECT_LIB NAMES freenect PATHS ${SEARCH_LIBS})
find_path(FREENECT_INCLUDE NAMES libfreenect/libfreenect.h
          PATHS ${SEARCH_HEADERS})

message("-- Check for GLUT (optional)")
if (APPLE)
  add_definitions(-DAPPLE)
# As Apple default GLUT doesn't support scroll event, we'll use FreeGLUT
# instead. You can enable this statement, if you have some trouble with it.
#  set(CMAKE_EXE_LINKER_FLAGS "-framework OpenGL -framework GLUT")
  set(CMAKE_EXE_LINKER_FLAGS "-framework OpenGL")
  find_package(FREEGLUT REQUIRED)
else (APPLE)  # Linux, not so much
  message(" -- Linux Detected")
  find_package(OpenGL REQUIRED)
  find_package(GLUT REQUIRED)
  include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})
  link_directories(${OpenGL_LIBRARY_DIRS} ${GLUT_INCLUDE_DIRS})
  add_definitions(${GLUT_DEFINITIONS})
  
  if(NOT OPENGL_FOUND)
    message(ERROR "    -- OpenGL not found!")
  else(NOT OPENGL_FOUND)
    message("    -- OpenGL found.")
  endif(NOT OPENGL_FOUND)

  if(NOT GLUT_FOUND)
    message(ERROR "    -- GLUT not found")
  else(NOT GLUT_FOUND)
    message("    -- GLUT found")
  endif(NOT GLUT_FOUND)
endif (APPLE)

# directories
set(CSIO_INCLUDE_DIR .)

# include directory
include_directories(${CSIO_INCLUDE_DIR})
include_directories(${PNG_INCLUDE_DIRS})
include_directories(${GLOG_INCLUDE})
include_directories(${GFLAGS_INCLUDE})
include_directories(${GTEST_INCLUDE_DIRS})

# use sse3 instruction set
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -msse3")  # -DNDEBUG

add_executable(csio_test src/csio_test.cc)
target_link_libraries(csio_test ${GOOGLE_LIBRARIES} ${GTEST_LIBS})
add_test(csio_test csio_test)
add_dependencies(csio_test googletest)

if (FREENECT_LIB)
  include_directories(${FREENECT_INCLUDE})
  add_executable(csiomod_freenect src/csiomod_freenect.cc)
  target_link_libraries(csiomod_freenect ${FREENECT_LIB} ${GOOGLE_LIBRARIES})
endif (FREENECT_LIB)

add_executable(csio_glviewer
    src/csio_glviewer.cc src/csio_glviewer_view.cc src/image_file.cc)

if (APPLE)
  target_link_libraries(csio_glviewer ${GOOGLE_LIBRARIES} ${PNG_LIBRARY})
else (APPLE)
  target_link_libraries(csio_glviewer ${GOOGLE_LIBRARIES} ${PNG_LIBRARY} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
endif (APPLE)

add_executable(spm2csio src/spm2csio.cc)
target_link_libraries(spm2csio ${GOOGLE_LIBRARIES} ${PNG_LIBRARY})

add_executable(simple_ply_viewer src/simple_ply_viewer.cc src/zpr.cc src/image_file.cc)

if (APPLE)
target_link_libraries(simple_ply_viewer ${GOOGLE_LIBRARIES} ${PNG_LIBRARY} ${OPENGL_LIBRARIES} ${FREEGLUT_LIBRARY})
else (APPLE)
target_link_libraries(simple_ply_viewer ${GOOGLE_LIBRARIES} ${PNG_LIBRARY} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
endif (APPLE)

// simple_ply_viewer.cc
//
// Author : Hyon Lim <limhyon@gmail.com>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <iostream>
#include <vector>
#include <cstdio>
#include "zpr.h"
#include "image_file.h"

#include <gflags/gflags.h>

using namespace std;
using namespace csio;

#define ENABLE_CONTROL_RECORDING // record key/mouse input for offline rendering into a movie

DEFINE_int32(fps, 25, "Frame per second");

// Drawing primitives
struct Point3f {
    float x, y, z;
};

struct Color3ub {
    unsigned char r, g, b;
};

struct Face {
    unsigned int a, b, c;
};

void draw_cameras(void);
void draw_axes(void);
void display(void);
void idle(void);
void keyboard(unsigned char key, int x, int y);
void load_ply(const char *filename, vector <Point3f> &points, vector <Color3ub> &colors, vector <Face> &faces);
void load_playback(const char *filename, double *projection_matrix, vector <double*> &states);

vector <Point3f> g_points;
vector <Color3ub> g_colors;
vector <Face> g_faces;
vector <double*> g_gl_states;

int g_time = 0;
int g_timebase = 0;
bool g_playback = false;
bool g_record_start = false;
bool g_capture = false;
double g_projection_matrix[16];

#define NUM_CAMERA_PARAMS 9
#define POLY_INVERSE_DEGREE 6

typedef struct {
    double R[9];     /* Rotation */
    double t[3];     /* Translation */
    double f;        /* Focal length */
    double k[2];     /* Undistortion parameters */
    double k_inv[POLY_INVERSE_DEGREE]; /* Inverse undistortion parameters */
    char constrained[NUM_CAMERA_PARAMS];
    double constraints[NUM_CAMERA_PARAMS];  /* Constraints (if used) */
    double weights[NUM_CAMERA_PARAMS];      /* Weights on the constraints */
    double K_known[9];  /* Intrinsics (if known) */
    double k_known[5];  /* Distortion params (if known) */

    char fisheye;            /* Is this a fisheye image? */
    char known_intrinsics;   /* Are the intrinsics known? */
    double f_cx, f_cy;       /* Fisheye center */
    double f_rad, f_angle;   
    
#define NUM_CAMERA_PARAMS 9
#define POLY_INVERSE_DEGREE 6/* Other fisheye parameters */
    double f_focal;          /* Fisheye focal length */

    double f_scale, k_scale; /* Scale on focal length, distortion params */
} camera_params_t;

typedef struct
{
    double pos[3];
    double color[3];
} point_t;

// Bundle file.
std::vector<camera_params_t> cameras;
std::vector<point_t> points;
double bundle_version;

void ReadBundleFile(char *bundle_file,
                    std::vector<camera_params_t> &cameras,
                    std::vector<point_t> &points, double &bundle_version) {
  FILE *f = fopen(bundle_file, "r");
  if (f == NULL) {
    printf("Error opening file %s for reading\n", bundle_file);
    return;
  }

  int num_images, num_points;

  char first_line[256];
  fgets(first_line, 256, f);
  if (first_line[0] == '#') {
    double version;
    sscanf(first_line, "# Bundle file v%lf", &version);

    bundle_version = version;
    printf("[ReadBundleFile] Bundle version: %0.3f\n", version);

    fscanf(f, "%d %d\n", &num_images, &num_points);
  } else if (first_line[0] == 'v') {
      double version;
      sscanf(first_line, "v%lf", &version);
      bundle_version = version;
      printf("[ReadBundleFile] Bundle version: %0.3f\n", version);

      fscanf(f, "%d %d\n", &num_images, &num_points);
  } else {
      bundle_version = 0.1;
      sscanf(first_line, "%d %d\n", &num_images, &num_points);
  }

  printf("[ReadBundleFile] Reading %d images and %d points...\n", num_images, num_points);

  /* Read cameras */
  for (int i = 0; i < num_images; i++) {
    double focal_length, k0, k1;
    double R[9];
    double t[3];

    if (bundle_version < 0.2) {
      /* Focal length */
      fscanf(f, "%lf\n", &focal_length);
    } else {
      fscanf(f, "%lf %lf %lf\n", &focal_length, &k0, &k1);
    }

    /* Rotation */
    fscanf(f, "%lf %lf %lf\n%lf %lf %lf\n%lf %lf %lf\n",
           R+0, R+1, R+2, R+3, R+4, R+5, R+6, R+7, R+8);
    /* Translation */
    fscanf(f, "%lf %lf %lf\n", t+0, t+1, t+2);

    // if (focal_length == 0.0)
    //     continue;

    camera_params_t cam;

    cam.f = focal_length;
    memcpy(cam.R, R, sizeof(double) * 9);
    memcpy(cam.t, t, sizeof(double) * 3);

    cameras.push_back(cam);
  }

  /* Read points */
  for (int i = 0; i < num_points; i++) {
    point_t pt;
    /* Position */
    fscanf(f, "%lf %lf %lf\n", pt.pos + 0, pt.pos + 1, pt.pos + 2);
    /* Color */
    fscanf(f, "%lf %lf %lf\n", pt.color + 0, pt.color + 1, pt.color + 2);

    int num_visible;
    fscanf(f, "%d", &num_visible);

    for (int j = 0; j < num_visible; j++) {
      int view, key;
      fscanf(f, "%d %d", &view, &key);

      double x, y;
      if (bundle_version >= 0.3)
        fscanf(f, "%lf %lf", &x, &y);
    }

    if (num_visible > 0) {
      points.push_back(pt);
    }
  }

  fclose(f);
}


int main(int argc, char **argv) {
  if(argc < 3) {
    printf("./simple_ply_viewer model.ply bundle.out [playback.bin]\n");
    return 0;
  }

  /* Read the bundle file */
  if(argc >= 3) {
    ReadBundleFile(argv[2], cameras, points, bundle_version);
  }

  if(argc > 3) {
    load_playback(argv[3], g_projection_matrix, g_gl_states);
  }

  /* Initialise GLUT and create a window */
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(640, 480);
  glutCreateWindow("GLT Mouse Zoom-Pan-Rotate");

  /* Configure GLUT callback functions */
  load_ply(argv[1], g_points, g_colors, g_faces);
  glScalef(0.25,0.25,0.25);

  zprInit();
  zprSelectionFunc(draw_axes);     /* Selection mode draw function */

  //glClearColor(1.0f, 1.0f, 1.0f, 0.0f); // Background white.
  glutDisplayFunc(display);
  glutKeyboardFunc(keyboard);
  glutIdleFunc(idle);

  glutMainLoop();

  return 0;
}

void idle(void) {
  if(g_playback) {
    int width = glutGet(GLUT_WINDOW_WIDTH );
    int height = glutGet(GLUT_WINDOW_HEIGHT);

    vector<char> img;
    img.resize(width*height*3);

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixd(g_projection_matrix);

    glMatrixMode(GL_MODELVIEW);

    for(unsigned int i=0; i < g_gl_states.size(); i++) {
      glLoadMatrixd(g_gl_states[i]);
      display();
      if(width % 4 != 0) glPixelStorei(GL_PACK_ALIGNMENT, width % 2 ? 1 : 2);
      glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, img.data());
      char file[512];
      sprintf(file, "frame-%06d.png", i);

      char* buf = img.data();
        for (int y = 0; y < height / 2; ++y) {
          char* p0 = &buf[y * width * 3];
          char* p1 = &buf[(height - 1 - y) * width * 3];
          for (int i = 0; i < width * 3; ++i, ++p0, ++p1) {
            char tmp = *p0;
            *p0 = *p1;
            *p1 = tmp;
          }
        }
        WriteRGB8ToPNG(img.data(), width, height, width * 3, file);
    }
    exit(0);
  } else if (g_capture) {
    int width = glutGet(GLUT_WINDOW_WIDTH );
    int height = glutGet(GLUT_WINDOW_HEIGHT);
    vector<char> img;
    img.resize(width*height*3);
    if(width % 4 != 0) glPixelStorei(GL_PACK_ALIGNMENT, width % 2 ? 1 : 2);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, img.data());
    char* buf = img.data();
      for (int y = 0; y < height / 2; ++y) {
        char* p0 = &buf[y * width * 3];
        char* p1 = &buf[(height - 1 - y) * width * 3];
        for (int i = 0; i < width * 3; ++i, ++p0, ++p1) {
          char tmp = *p0;
          *p0 = *p1;
          *p1 = tmp;
        }
      }
    WriteRGB8ToPNG(img.data(), width, height, width * 3, "capture.png");
    std::cout << "Captured. see capture.png" << std::endl;
    g_capture = false;
  }

  glutPostRedisplay();
}

void load_playback(const char *filename,
                   double *projection_matrix,
                   vector <double*> &states) {
  FILE *fp = fopen(filename, "r");

  if(!fp) {
    fprintf(stderr, "Error opening %s\n", filename);
    exit(1);
  }

  fread((void*)projection_matrix, sizeof(double)*16, 1, fp);

  while(!feof(fp)) {
    double *m = new double[16];
    fread((void*)&m[0], sizeof(double)*16, 1, fp);
    g_gl_states.push_back(m);
  }

  fclose(fp);
  printf("Playback images %d\n", g_gl_states.size());
  g_playback = true;
}

void display(void) {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  draw_cameras();
  draw_axes();

  glBegin(GL_TRIANGLES);
  for(unsigned int i=0; i < g_faces.size(); i++) {
    int a = g_faces[i].a;
    int b = g_faces[i].b;
    int c = g_faces[i].c;

    glColor3ub(g_colors[a].r, g_colors[a].g, g_colors[a].b);
    glVertex3f(g_points[a].x, g_points[a].y, g_points[a].z);

    glColor3ub(g_colors[b].r, g_colors[b].g, g_colors[b].b);
    glVertex3f(g_points[b].x, g_points[b].y, g_points[b].z);

    glColor3ub(g_colors[c].r, g_colors[c].g, g_colors[c].b);
    glVertex3f(g_points[c].x, g_points[c].y, g_points[c].z);
  }
  glEnd();

  glBegin(GL_POINTS);
  for(unsigned int i=0; i < g_points.size(); ++i) {
    glColor3ub(g_colors[i].r, g_colors[i].g, g_colors[i].b);
    glVertex3f(g_points[i].x, g_points[i].y, g_points[i].z);
  }
  glEnd();

  glLineWidth(2.5);
  glColor3f(1.0, 1.0, 1.0);
  glBegin(GL_LINES);
  for (unsigned int i = 1; i < cameras.size(); ++i) {
    glVertex3f(cameras[i-1].t[0], cameras[i-1].t[1], cameras[i-1].t[2]);
    glVertex3f(cameras[i].t[0], cameras[i].t[1], cameras[i].t[2]);
  }
  glEnd();

#ifdef ENABLE_CONTROL_RECORDING
  if(!g_playback && g_record_start) {
    g_time = glutGet(GLUT_ELAPSED_TIME);

    if ( (g_time - g_timebase) > (1000 / FLAGS_fps) ) {
      double *m = new double[16];
      glGetDoublev(GL_MODELVIEW_MATRIX, m);
#if 0
      printf("\n");
      printf("%f %f %f %f\n", m[0], m[1], m[2], m[3]);
      printf("%f %f %f %f\n", m[4], m[5], m[6], m[7]);
      printf("%f %f %f %f\n", m[8], m[9], m[10], m[11]);
      printf("%f %f %f %f\n", m[12], m[13], m[14], m[15]);
#endif
      g_gl_states.push_back(m);
      g_timebase = g_time;
    }
  }
#endif

  glutSwapBuffers();
}

void keyboard(unsigned char key, int x, int y) {
  // Press ESC key.
  if (key == 27) {
    FILE *fp = fopen("playback.bin", "wb+");
    if(!fp) {
      fprintf(stderr, "Error writing playback file\n");
      exit(1);
    }
    // Write the projection matrix
    double m[16];
    glGetDoublev(GL_PROJECTION_MATRIX, m);
    fwrite((void*)&m[0], sizeof(double)*16, 1, fp);

    // Write the modelview matrix
    for(unsigned int i=0; i < g_gl_states.size(); i++) {
      fwrite((void*)&g_gl_states[i][0], sizeof(double)*16, 1, fp);
    }

    fclose(fp);

		exit(0);
  } else if (key == 'c') {
    // Self capture
    g_capture = true;
  } else if (key == 's') {
    std::cout << "Record started\n";
    g_record_start = true;
  } else if (key == 'x') {
    g_record_start = false;
    std::cout << "Record paused\n";
  }
}

void draw_cameras(void) {
  GLfloat m[16];
  for (unsigned int i = 0; i < cameras.size(); ++i) {
    glPushMatrix();
      const double *t = cameras[i].t, *r = cameras[i].R;
#define M(row,col)  m[col*4+row]
    M(0, 0) = r[0]; M(0, 1) = r[1]; M(0, 2) = r[2]; M(0, 3) = 0.0;
    M(1, 0) = r[3]; M(1, 1) = r[4]; M(1, 2) = r[5]; M(1, 3) = 0.0;
    M(2, 0) = r[6]; M(2, 1) = r[7]; M(2, 2) = r[8]; M(2, 3) = 0.0;
    M(3, 0) = 0.0; M(3, 1) = 0.0; M(3, 2) = 0.0; M(3, 3) = 1.0;
#undef M
      glTranslatef(t[0], t[1], t[2]);
      glMultMatrixf(m);
      glColor3f(0.3, 0.3, 0.3);
      glutSolidSphere(0.01, 20, 20); 
        glPushMatrix();
          glPushName(1);
            glColor3f(1,0,0);
            glRotatef(90,0,1,0);
            glutSolidCone(0.01, 0.1, 20, 20);
          glPopName();
        glPopMatrix();
        glPushMatrix ();
          glPushName(2);            /* Green cone is 2 */
            glColor3f(0,1,0);
            glRotatef(-90,1,0,0);
            glutSolidCone(0.01, 0.1, 20, 20);
          glPopName();
        glPopMatrix();
          glColor3f(0,0,1);         /* Blue cone is 3 */
          glPushName(3);
            glutSolidCone(0.01, 0.1, 20, 20);
          glPopName();
    glPopMatrix();
  }
}

void draw_axes(void) {
  /* Name-stack manipulation for the purpose of
     selection hit processing when mouse button
     is pressed.  Names are ignored in normal
     OpenGL rendering mode.                    */

  glPushMatrix();
                                /* No name for grey sphere */

  glColor3f(0.3,0.3,0.3);
  glutSolidSphere(0.05, 20, 20);

  glPushMatrix();
  glPushName(1);            /* Red cone is 1 */
    glColor3f(1,0,0);
    glRotatef(90,0,1,0);
    glutSolidCone(0.03, 0.3, 20, 20);
  glPopName();
  glPopMatrix();

  glPushMatrix ();
  glPushName(2);            /* Green cone is 2 */
    glColor3f(0,1,0);
    glRotatef(-90,1,0,0);
    glutSolidCone(0.03, 0.3, 20, 20);
  glPopName();
  glPopMatrix();

  glColor3f(0,0,1);         /* Blue cone is 3 */
    glPushName(3);
      glutSolidCone(0.03, 0.3, 20, 20);
    glPopName();
  glPopMatrix();
}

void load_ply(const char *filename, 
              vector <Point3f> &points, 
              vector <Color3ub> &colors,
              vector <Face> &faces){
  points.clear();
  colors.clear();
  faces.clear();

  FILE *fp = fopen(filename, "r");

  if(!fp) {
    fprintf(stderr, "Error opening %s\n", filename);
    exit(1);
  }

  char line[1024];
  char dum_s[512];
  int n_vertex;
  int n_face;

  // WARNING:
  // THIS ASSUMES A VERY STRICT FORMAT FROM MESHLAB!
  // Change this if nothing renders correctly
  fgets(line, sizeof(line), fp); 
  fgets(line, sizeof(line), fp);
  fgets(line, sizeof(line), fp);
  sscanf(line, "%s %s %d", dum_s, dum_s, &n_face);
  fgets(line, sizeof(line), fp);
  fgets(line, sizeof(line), fp);
  sscanf(line, "%s %s %d", dum_s, dum_s, &n_vertex);
  fgets(line, sizeof(line), fp);
  fgets(line, sizeof(line), fp);
  fgets(line, sizeof(line), fp);
  fgets(line, sizeof(line), fp);
  fgets(line, sizeof(line), fp);
  fgets(line, sizeof(line), fp);
  fgets(line, sizeof(line), fp);

  points.resize(n_vertex);
  colors.resize(n_vertex);
  faces.resize(n_face);

  for(int i=0; i < n_vertex; i++) {
    fgets(line, sizeof(line), fp);

    Point3f pt;
    Color3ub col;

    sscanf(line, "%f %f %f %d %d %d", &pt.x, &pt.y, &pt.z, &col.r, &col.g, &col.b);

    points[i] = pt;
    colors[i] = col;
  }

  for(int i=0; i < n_face; i++) {
    int dum;
    Face face;
    fgets(line, sizeof(line), fp);
    sscanf(line, "%d %d %d %d", &dum, &face.a, &face.b, &face.c);
    faces[i] = face;
  }
}

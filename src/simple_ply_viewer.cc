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

#include <vector>
#include <cstdio>
#include "zpr.h"
#include "image_file.h"

using namespace std;
using namespace csio;

#define ENABLE_CONTROL_RECORDING // record key/mouse input for offline rendering into a movie

const int FPS_record = 25;

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
double g_projection_matrix[16];

int main(int argc, char **argv) {
  if(argc < 2) {
    printf("./simple_ply_viewer model.ply [playback.bin]\n");
    return 0;
  }

  if(argc == 3) {
    load_playback(argv[2], g_projection_matrix, g_gl_states);
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
      glReadPixels(0, 0, width, height, GL_BGR, GL_UNSIGNED_BYTE, img.data());

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


#ifdef ENABLE_CONTROL_RECORDING
  if(!g_playback) {
    g_time = glutGet(GLUT_ELAPSED_TIME);

    if ( (g_time - g_timebase) > (1000 / FPS_record) ) {
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
  glutSolidSphere(0.1, 20, 20);

  glPushMatrix();
  glPushName(1);            /* Red cone is 1 */
    glColor3f(1,0,0);
    glRotatef(90,0,1,0);
    glutSolidCone(0.1, 1.0, 20, 20);
  glPopName();
  glPopMatrix();

  glPushMatrix ();
  glPushName(2);            /* Green cone is 2 */
    glColor3f(0,1,0);
    glRotatef(-90,1,0,0);
    glutSolidCone(0.1, 1.0, 20, 20);
  glPopName();
  glPopMatrix();

  glColor3f(0,0,1);         /* Blue cone is 3 */
    glPushName(3);
      glutSolidCone(0.1, 1.0, 20, 20);
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

/*
TODOs: better division into public and private
TODOs: cleanup
TODOs: perception angle
TODOs: animate
*/

#include <algorithm>
#include <iostream>
#include <list>
#include <vector>
#include <math.h>
#include <cassert>
#include <stdio.h>
#include <stdlib.h>
/* Use glew.h instead of gl.h to get all the GL prototypes declared */
#include <GL/glew.h>
/* Using the GLUT library for the base windowing setup */
#include <GL/freeglut.h>
#include <math.h>
#include <unistd.h>

#include <chrono>
#include <thread>

#include "potential_functions.h"
#include "Boid.h"
#include "Vector2D.h"


using namespace std;


const float PI = 2 * std::acos(0.0);

GLuint program;
GLint attribute_coord2d;



const int DIMENSIONS = 2;

class Swarm {
    public:
        Swarm(int, float, float, float, float, float, float);
        int amount_boids;
        float size_x;
        float size_y;
        float base_velocity;
        float max_velocity;
        float perception_angle;
        float drive;

        list<Boid> boids; // = {};
    
        void move_boids(){
            update_state();
            for (auto& boid : boids){
                boid.move();
            }
            //  TODO: return positions and velocities?
        }

        void print_boids(){
            for (const auto& boid : boids){
                cout << boid.x << endl;
            }
        }

        // this can all be private.
        void update_state(){
            gather_positions();
            gather_velocities();
            calc_distances();
            calc_border_distances();
            calc_magnitudes();
            calc_vectors();
            update_boids();
        }

        void gather_positions(){
            // assert("Dimensions of boids and positions do not match", boids.size() != positions.values.size());
            int idx = 0;
            for (const auto& boid : boids){ // zip or enumerate possible?
                positions.values.at(idx).at(0) = boid.x; // ::operator[]?
                positions.values.at(idx).at(1) = boid.y;
                idx++;
            }
        }

        void gather_velocities(){
            // assert("Dimensions of boids and velocities do not match", boids.size() != velocities.values.size());
            int idx = 0;
            for (const auto& boid : boids){ // zip or enumerate possible?
                velocities.values.at(idx).at(0) = boid.vx;
                velocities.values.at(idx).at(1) = boid.vy;
                idx++;
            }
        }

        void calc_distances(){
            int size = positions.values.size();
            // assert("Dimensions of positions and distances do not match", size != distances.values.size());
            // assert("Dimensions of distances not square", distances.values.size() != distances[0].values.size());
            // assert("Dimensions of dimensions and distances do not match", DIMENSIONS != distances.values.size());
            for (int i=0; i<size; i++) {
                for (int j=0; j<size; j++) {
                    const float& x1 = positions.values.at(i).at(0);
                    const float& y1 = positions.values.at(i).at(1);
                    const float& x2 = positions.values.at(j).at(0);
                    const float& y2 = positions.values.at(j).at(1);
                    distances.values.at(i).at(j) = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
                    pos_diff_x.values.at(i).at(0) = x2 - x1;
                    pos_diff_y.values.at(i).at(0) = y2 - y1;
                }
            }
        }

        void calc_border_distances(){
            // left right top down
            int size = positions.values.size();
            for (int i=0; i<size; i++){
                const float& x = positions.values.at(i).at(0);
                const float& y = positions.values.at(i).at(1);
                
                if (x >= 0.) border_distances.values.at(i).at(0) = x;
                else border_distances.values.at(i).at(0) = 0.;

                if (size_x - x >= 0) border_distances.values.at(i).at(1) = size_x - x;
                else border_distances.values.at(i).at(1) = 0.;

                if (y >= 0.) border_distances.values.at(i).at(2) = y;
                else border_distances.values.at(i).at(2) = 0.;

                if (size_y - y >= 0) border_distances.values.at(i).at(3) = size_y - y;
                else border_distances.values.at(i).at(3) = 0.;
            }
        }

        void calc_angle_filter(){
            // TODO
        }

        void remove_boid(){
            // TODO
        }

        void add_boid(){
            // TODO
        }

        void calc_magnitudes() {
            // TODO: asserts for other pot funcs
            int size = distances.values.size();
            // assert("Dimensions of positions and separation_magnitudes do not match", size != separation_magnitudes.values.size());
            // assert("Dimensions of distances not square", distances.values.size() != distances.values[0].size());
            // assert("Dimensions of separation_magnitudes not square", separation_magnitudes.values.size() != separation_magnitudes[0].values.size());
            for (int i=0; i<size; i++) {
                for (int j=0; j<size; j++) {
                    if (i == j) {
                        separation_magnitudes.values.at(i).at(j) = 0;
                        alignment_magnitudes.values.at(i).at(j) = 0;
                        cohesion_magnitudes.values.at(i).at(j) = 0;
                    } else {
                        separation_magnitudes.values.at(i).at(j) = separation_potential_func(distances.values.at(i).at(j));
                        alignment_magnitudes.values.at(i).at(j) = alignment_potential_func(distances.values.at(i).at(j));
                        cohesion_magnitudes.values.at(i).at(j) = cohesion_potential_func(distances.values.at(i).at(j));
                    }
                }
            }
            for (int i=0; i<size; i++) {
                for (int j=0; j<4; j++) {
                    border_magnitudes.values.at(i).at(j) = border_potential_func(border_distances.values.at(i).at(j));
                }
            }
        }

        void calc_vectors(){
            // TODO: asserts
            int size = separation_magnitudes.values.size();
            float norm = 0;
            for (int i=0; i<size; i++) {
                separation_vectors.values.at(i).at(0) = 0.;
                separation_vectors.values.at(i).at(1) = 0.;
                cohesion_vectors.values.at(i).at(0) = 0.;
                cohesion_vectors.values.at(i).at(1) = 0.;
                alignment_vectors.values.at(i).at(0) = 0.;
                alignment_vectors.values.at(i).at(1) = 0.;
                for (int j=0; j<size; j++) {
                    if (distances.values.at(i).at(j) == 0) norm = 1.;
                    else norm = distances.values.at(i).at(j);

                    separation_vectors.values.at(i).at(0) -= separation_magnitudes.values.at(i).at(j) * pos_diff_x.values.at(i).at(0) / norm;
                    separation_vectors.values.at(i).at(1) -= separation_magnitudes.values.at(i).at(j) * pos_diff_y.values.at(i).at(0) / norm;

                    cohesion_vectors.values.at(i).at(0) += cohesion_magnitudes.values.at(i).at(j) * pos_diff_x.values.at(i).at(0) / norm;
                    cohesion_vectors.values.at(i).at(1) += cohesion_magnitudes.values.at(i).at(j) * pos_diff_y.values.at(i).at(0) / norm;

                    alignment_vectors.values.at(i).at(0) += alignment_magnitudes.values.at(i).at(j) * velocities.values.at(i).at(0);
                    alignment_vectors.values.at(i).at(1) += alignment_magnitudes.values.at(i).at(j) * velocities.values.at(i).at(1);

                    border_vectors.values.at(i).at(0) += border_magnitudes.values.at(i).at(0) * positions.values.at(i).at(0);
                    border_vectors.values.at(i).at(0) += border_magnitudes.values.at(i).at(1) * (positions.values.at(i).at(0) - 10);
                    border_vectors.values.at(i).at(1) += border_magnitudes.values.at(i).at(2) * positions.values.at(i).at(1);
                    border_vectors.values.at(i).at(1) += border_magnitudes.values.at(i).at(3) * (positions.values.at(i).at(1) - 10);

                }
            }
        }

        void update_boids(){
            int size = separation_magnitudes.values.size(); // TODO: get the size from somewhere else
            float vector_update_x = 0;
            float vector_update_y = 0;
            float new_vx = 0;
            float new_vy = 0;
            float norm = 1;
            int idx = 0;

            for (auto& boid : boids){
                vector_update_x = separation_vectors.values.at(idx).at(0) + 20 * alignment_vectors.values.at(idx).at(0) + cohesion_vectors.values.at(idx).at(0) + 300*border_vectors.values.at(idx).at(0);
                vector_update_y = separation_vectors.values.at(idx).at(1) + 20 * alignment_vectors.values.at(idx).at(1) + cohesion_vectors.values.at(idx).at(1) + 300*border_vectors.values.at(idx).at(1);
                norm = sqrt(pow(boid.vx, 2) + pow(boid.vy, 2));
                boid.vx /= norm / base_velocity;
                boid.vy /= norm / base_velocity;

                new_vx = drive * vector_update_x + boid.vx;
                new_vy = drive * vector_update_y + boid.vy;
                norm = sqrt(pow(new_vx, 2) + pow(new_vy, 2));
                if (norm >= max_velocity){
                    boid.vx = max_velocity * new_vx / norm;
                    boid.vy = max_velocity * new_vy / norm;
                } else {
                    boid.vx = drive * vector_update_x + boid.vx;
                    boid.vy = drive * vector_update_y + boid.vy;
                }
                idx++;
            }
        }
        
    private:
        Vector2D<float> positions = Vector2D<float> (DIMENSIONS, amount_boids);  // can I init without size?
        Vector2D<float> velocities = Vector2D<float> (DIMENSIONS, amount_boids);
        Vector2D<float> distances = Vector2D<float> (amount_boids, amount_boids);
        // TODO list for dimensions?
        Vector2D<float> pos_diff_x = Vector2D<float> (1, amount_boids);
        Vector2D<float> pos_diff_y = Vector2D<float> (1, amount_boids);
        Vector2D<float> border_distances = Vector2D<float> (2*DIMENSIONS, amount_boids);

        Vector2D<float> separation_magnitudes = Vector2D<float> (amount_boids, amount_boids);
        Vector2D<float> alignment_magnitudes = Vector2D<float> (amount_boids, amount_boids);
        Vector2D<float> cohesion_magnitudes = Vector2D<float> (amount_boids, amount_boids);
        Vector2D<float> border_magnitudes = Vector2D<float> (4, amount_boids);

        Vector2D<float> separation_vectors = Vector2D<float> (DIMENSIONS, amount_boids);
        Vector2D<float> alignment_vectors = Vector2D<float> (DIMENSIONS, amount_boids);
        Vector2D<float> cohesion_vectors = Vector2D<float> (DIMENSIONS, amount_boids);
        Vector2D<float> border_vectors = Vector2D<float> (DIMENSIONS, amount_boids);
};


Swarm::Swarm (int a, float b, float c, float d, float e, float f, float g) {
    amount_boids = a;
    size_x = b;
    size_y = c;
    base_velocity = d;
    max_velocity = e;
    perception_angle = f;
    drive = g;
    for (int i=0; i<amount_boids; i++){
        // Push given amount of boids to list.
        boids.push_back(Boid(base_velocity, size_x, size_y));
    }
    
    positions.reshape(DIMENSIONS, amount_boids);
    velocities.reshape(DIMENSIONS, amount_boids);
    distances.reshape(amount_boids, amount_boids);
    pos_diff_x.reshape(1, amount_boids);
    pos_diff_y.reshape(1, amount_boids);
    border_distances.reshape(2*DIMENSIONS, amount_boids);
    separation_magnitudes.reshape(amount_boids, amount_boids);
    alignment_magnitudes.reshape(amount_boids, amount_boids);
    cohesion_magnitudes.reshape(amount_boids, amount_boids);
    border_magnitudes.reshape(4, amount_boids);
    separation_vectors.reshape(DIMENSIONS, amount_boids);
    alignment_vectors.reshape(DIMENSIONS, amount_boids);
    cohesion_vectors.reshape(DIMENSIONS, amount_boids);
    border_vectors.reshape(2*DIMENSIONS, amount_boids);
}


Swarm swarm (20, 10., 10., 0.05, 0.2, 110., 0.01);


int init_resources()
{
  GLint compile_ok = GL_FALSE, link_ok = GL_FALSE;

  GLuint vs = glCreateShader(GL_VERTEX_SHADER);
  const char *vs_source =
#ifdef GL_ES_VERSION_2_0
    "#version 100\n"  // OpenGL ES 2.0
#else
    "#version 120\n"  // OpenGL 2.1
#endif
    "attribute vec2 coord2d;                  "
    "void main(void) {                        "
    "  gl_Position = vec4(coord2d, 0.0, 1.0); "
    "}";
  glShaderSource(vs, 1, &vs_source, NULL);
  glCompileShader(vs);
  glGetShaderiv(vs, GL_COMPILE_STATUS, &compile_ok);
  if (!compile_ok) {
    fprintf(stderr, "Error in vertex shader\n");
    return 0;
  }

  GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
  const char *fs_source =
#ifdef GL_ES_VERSION_2_0
    "#version 100\n"  // OpenGL ES 2.0
#else
    "#version 120\n"  // OpenGL 2.1
#endif
    "void main(void) {        "
    "  gl_FragColor[0] = 0.0; "
    "  gl_FragColor[1] = 0.0; "
    "  gl_FragColor[2] = 1.0; "
    "}";
  glShaderSource(fs, 1, &fs_source, NULL);
  glCompileShader(fs);
  glGetShaderiv(fs, GL_COMPILE_STATUS, &compile_ok);
  if (!compile_ok) {
    fprintf(stderr, "Error in fragment shader\n");
    return 0;
  }

  program = glCreateProgram();
  glAttachShader(program, vs);
  glAttachShader(program, fs);
  glLinkProgram(program);
  glGetProgramiv(program, GL_LINK_STATUS, &link_ok);
  if (!link_ok) {
    fprintf(stderr, "glLinkProgram:");
    return 0;
  }

  const char* attribute_name = "coord2d";
  attribute_coord2d = glGetAttribLocation(program, attribute_name);
  if (attribute_coord2d == -1) {
    fprintf(stderr, "Could not bind attribute %s\n", attribute_name);
    return 0;
  }

  return 1;
}

void makeBoidPolygon(const float& x, const float& y, const float& r, const float& alpha){
  glBegin(GL_POLYGON);//begin drawing of polygon
    float alpha_rad = (alpha * PI) / 180;
    float angle_incr = 0.174532925;
    float angle_90 = 1.570796327;
    glVertex3f(x + r * std::cos(alpha_rad), y + r * std::sin(alpha_rad), 0);
    for (int i=0; i<19; i++){
        glVertex3f(x + r * std::cos(alpha_rad + angle_90 + i * angle_incr), y + r * std::sin(alpha_rad + angle_90 + i * angle_incr), 0);
    }
  glEnd();//end drawing of polygon
}



void onDisplay()
{
  glClearColor(1.0, 1.0, 1.0, 1.0);
  glClear(GL_COLOR_BUFFER_BIT);

  glUseProgram(program);
  glEnableVertexAttribArray(attribute_coord2d);


  //clear color and depth buffer 
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();//load identity matrix
  glTranslatef(0.0f,0.0f,-4.0f);//move forward 4 units

  glColor3f(0.0f,0.0f,1.0f); //blue color

  // makeBoidPolygon(0.5, 0.5, 0.2, 15);

  glDisableVertexAttribArray(attribute_coord2d);
  glutSwapBuffers();
}


void onIdle(){

    float angle = 0;

    // Swarm swarm (20, 10., 10., 0.05, 0.2, 110., 0.01);
    for(int i=0; i<100; i++){
        swarm.move_boids();
        // usleep(3000);
        this_thread::sleep_for(chrono::milliseconds(50));
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glColor3f(0.0f,0.0f,1.0f);
        for (const auto& boid : swarm.boids){
            if (boid.vx > 0) {angle = atan(boid.vy / boid.vx);}
            if (boid.vx < 0 && boid.vy >= 0) {angle = atan(boid.vy / boid.vx) + PI;}
            if (boid.vx < 0 && boid.vy < 0) {angle = atan(boid.vy / boid.vx) - PI;}
            if (boid.vx == 0 && boid.vy > 0.) {angle = PI/2;}
            if (boid.vx == 0 && boid.vy < 0.) {angle = -PI/2;}
            angle *= 180 / PI;
            // makeBoidPolygon(boid.x/11., boid.y/11., 0.02, angle);
            makeBoidPolygon((boid.x)/5. - 1, (boid.y)/5 - 1, 0.02, angle);
            // makeBoidPolygon(boid.x/11., boid.y/11., 0.02, 45+i*3.6);
        }
        glutSwapBuffers();
    }
}


void free_resources()
{
  glDeleteProgram(program);
}


int main(int argc, char* argv[]) {


  glutInit(&argc, argv);
  glutInitContextVersion(2,0);
  glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);
  glutInitWindowSize(800, 800);
  glutCreateWindow("Swarm Sim");

  GLenum glew_status = glewInit();
  if (glew_status != GLEW_OK) {
    fprintf(stderr, "Error: %s\n", glewGetErrorString(glew_status));
    return 1;
  }
  cout << "BLAH!" << endl;
  
  if (init_resources()) {
    glutDisplayFunc(onDisplay);
    glutIdleFunc(onIdle);
    glutMainLoop();
  }

  free_resources();
  return 0;
}
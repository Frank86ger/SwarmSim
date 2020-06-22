#include <iostream>
#include <random>
#include <math.h>
#include "Boid.h"
using namespace std;


random_device rd;
mt19937 gen(rd());
uniform_real_distribution<float> uniform_0_1(0.0, 1.0);
uniform_real_distribution<float> uniform_m1_1(-1.0, 1.0);
uniform_int_distribution<int> discrete_0_1(0, 1);


Boid::Boid (float a, float b, float c) {
  velocity = a;
  size_x = b;
  size_y = c;
  x = size_x * uniform_0_1(gen);
  y = size_y * uniform_0_1(gen);
  vx = velocity * uniform_0_1(gen);
  vy = sqrt(pow(velocity, 2) - pow(vx, 2)) * (2 * discrete_0_1(gen) - 1);
}

float Boid::get_velocity() {return velocity;}
void Boid::set_velocity(float a) {velocity=a;}
float Boid::get_size_x() {return size_x;}
void Boid::set_size_x(float a) {size_x=a;}
float Boid::get_size_y() {return size_y;}
void Boid::set_size_y(float a) {size_y=a;}

void Boid::move(){
  x += vx;
  y += vy;
  if(x > size_x){x = size_x - 0.0001;}
  if(y > size_y){y = size_y - 0.0001;}
  if(x < 0){x = 0.0001;}
  if(y < 0){y = 0.0001;}
}

// template class Num<float>;
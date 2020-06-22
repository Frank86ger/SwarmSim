#include <math.h>

using namespace std;

float separation_potential_func(float& distance){
    return 0.1 / pow(distance+0.0001, 2);
}

float alignment_potential_func(float& distance){
    return 1.0 * exp(-pow(distance, 2));
}

float cohesion_potential_func(float& distance){
    return 1.0 / (distance + 1.0);
}

float border_potential_func(float& distance){
    return separation_potential_func(distance);
}

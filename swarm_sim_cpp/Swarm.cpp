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

#include "potential_functions.h"
#include "Boid.h"
#include "Vector2D.h"


using namespace std;

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
            // # calculate magnitudes from potentials
            // separation_magnitudes = separation_potential_func(distances)  # steer AWAY from weighted center of mass
            // alignment_magnitudes = alignment_potential_func(distances)  # adjust to weighted average of directions
            // cohesion_magnitudes = cohesion_potential_func(distances)  # steer TOWARDS weighted center of mass
            // border_magnitudes = border_potential_func(border_distances)
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


            
        // vector_update = separation_vectors + 20*alignment_vectors + cohesion_vectors + 300*border_repulsion_vectors
        // vector_update = vector_update.T

        // # update velocities of boids
        // for idx, boid in enumerate(self.boids):
        //     norm = np.sqrt(boid.vx**2 + boid.vy**2)
        //     boid.vx /= norm / self.base_velocity
        //     boid.vy /= norm / self.base_velocity

        //     new_x = self.drive * vector_update[idx, 0] + boid.vx
        //     new_y = self.drive * vector_update[idx, 1] + boid.vy
        //     norm = np.sqrt(new_x**2 + new_y**2)

        //     # if velocity bigger than `max_velocity`, norm it to `max_velocity
        //     if norm >= self.max_velocity:
        //         boid.vx = self.max_velocity * new_x / norm
        //         boid.vy = self.max_velocity * new_y / norm
        //     else:
        //         boid.vx = self.drive * vector_update[idx, 0] + boid.vx
        //         boid.vy = self.drive * vector_update[idx, 1] + boid.vy
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
        Vector2D<float> border_vectors = Vector2D<float> (2*DIMENSIONS, amount_boids);
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

int main() {
    cout << "BLAH" << endl;
    Swarm swarm (20, 10., 10., 0.05, 0.2, 110., 0.01);
    for(int i=0; i<100; i++){
        swarm.move_boids();
    }
    swarm.print_boids();

    return 0;
}
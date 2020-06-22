// #ifndef Boid_H
// #define Boid_H

class Boid {
    public:
        Boid(float, float, float);
        float x;
        float y;
        float vx;
        float vy;
        float get_velocity();
        void set_velocity(float a);
        float get_size_x();
        void set_size_x(float a);
        float get_size_y();
        void set_size_y(float a);
        void move();
    private:
        float velocity;
        float size_x;
        float size_y;
};

// #endif

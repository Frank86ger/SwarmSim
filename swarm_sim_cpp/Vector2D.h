
#include <iostream>
#include <vector>
#include <math.h>

using namespace std;

template <typename T>
class Vector2D { // ::operator[a, b]?
    public:
        Vector2D(int, int);
        vector<vector<T> > values;

        void insert_x(int position, vector<T>& values);
        void insert_y(int position, vector<T>& values);
        void delete_x(int position);
        void delete_y(int position);
        void fill_diagonal(T value);
        void transpose();
        void pairwise_difference();
        void pairwise_distance();

        void reshape(int a, int b);

    private:
        int dim_x;
        int dim_y;
};

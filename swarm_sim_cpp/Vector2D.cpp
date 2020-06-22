#include <iostream>
#include <vector>
#include <math.h>
// #include <Vector2D.h>
#include "Vector2D.h"

using namespace std;

// Constructor
template <typename T>
Vector2D<T>::Vector2D(int a, int b){
    dim_x = a;
    dim_y = b;
    values.resize(dim_y);
    for (auto& slice : values) {
        slice.resize(dim_x);
    }     
}

template <typename T>
void Vector2D<T>::insert_x(int position, vector<T>& values) {};
template <typename T>
void Vector2D<T>::insert_y(int position, vector<T>& values) {};
template <typename T>
void Vector2D<T>::delete_x(int position) {};
template <typename T>
void Vector2D<T>::delete_y(int position) {};

template <typename T>
void Vector2D<T>::reshape(int a, int b) {
    dim_x = a;
    dim_y = b;
    values.resize(dim_y);
    for (auto& slice : values) {
        slice.resize(dim_x);
    }   
};

template <typename T>
void Vector2D<T>::fill_diagonal(T value) {
    // assert(("Matrix is not square", values.size() != values[0].size()));
    for (int i=0; i<values.size(); i++){
        values.at(i).at(i) = value;
    }
};

template <typename T>
void Vector2D<T>::transpose() {};
template <typename T>
void Vector2D<T>::pairwise_difference() {};
template <typename T>
void Vector2D<T>::pairwise_distance() {};

template class Vector2D<float>;

// int main(){
//     Vector2D<float> blah (2, 2);
//     cout << blah.values.at(0).at(0) << endl;
//     return 0;
// }

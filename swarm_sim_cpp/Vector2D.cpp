#include <iostream>
#include <vector>
#include <math.h>
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

// TODO: insert and remove slices of 2D Vector.
template <typename T>
void Vector2D<T>::insert_x(int position, vector<T>& values) {};
template <typename T>
void Vector2D<T>::insert_y(int position, vector<T>& values) {};
template <typename T>
void Vector2D<T>::delete_x(int position) {};
template <typename T>
void Vector2D<T>::delete_y(int position) {};
// TODO: see if needed
template <typename T>
void Vector2D<T>::transpose() {};
template <typename T>
void Vector2D<T>::pairwise_difference() {};
template <typename T>
void Vector2D<T>::pairwise_distance() {};

// reshape vector
template <typename T>
void Vector2D<T>::reshape(int a, int b) {
    dim_x = a;
    dim_y = b;
    values.resize(dim_y);
    for (auto& slice : values) {
        slice.resize(dim_x);
    }   
};

// fill diagonal with specific value
template <typename T>
void Vector2D<T>::fill_diagonal(T value) {
    // assert(("Matrix is not square", values.size() != values[0].size()));
    for (int i=0; i<values.size(); i++){
        values.at(i).at(i) = value;
    }
};

template class Vector2D<float>;

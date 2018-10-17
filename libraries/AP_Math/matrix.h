/*
 * matrix.hpp
 *
 *  Created on: 2018-8-10
 *      Author: liwh1
 */

#pragma once

#include "math.h"
#include <stdint.h>
#include "vectorN.h"

template <typename T, uint8_t N>
class VectorN;


template <typename T, uint8_t  M, uint8_t N>
class Matrix {

    friend class VectorN<T,N>;

public:

    // Constructors
     Matrix() : _data() {}

     Matrix(const T data_[][N]) : _data()
     {
         memcpy(_data, data_, sizeof(_data));
     }

     Matrix(const T *data_) : _data()
     {
         memcpy(_data, data_, sizeof(_data));
     }

     Matrix(const Matrix &other) : _data()
     {
         memcpy(_data, other._data, sizeof(_data));
     }


     /**
      * Accessors/ Assignment etc.
      */

     T *data()
     {
         return _data[0];
     }

     const T *data() const
     {
         return _data[0];
     }

     inline T operator()(uint8_t i, uint8_t j) const
     {
         return _data[i][j];
     }

     inline T &operator()(uint8_t i, uint8_t j)
     {
         return _data[i][j];
     }

     Matrix<T, M, N> & operator=(const Matrix<T, M, N> &other)
     {
         if (this != &other) {
             memcpy(_data, other._data, sizeof(_data));
         }
         return (*this);
     }


     /**
      * Matrix Operations
      */

     // this might use a lot of programming memory
     // since it instantiates a class for every
     // required mult pair, but it provides
     // compile time size_t checking
     template<uint8_t P>
     Matrix<T, M, P> operator*(const Matrix<T, N, P> &other) const
     {
         const Matrix<T, M, N> &self = *this;
         Matrix<T, M, P> res;
         res.setZero();

         for (size_t i = 0; i < M; i++) {
             for (size_t k = 0; k < P; k++) {
                 for (size_t j = 0; j < N; j++) {
                     res(i, k) += self(i, j) * other(j, k);
                 }
             }
         }

         return res;
     }

     Matrix<T, M, N> emult(const Matrix<T, M, N> &other) const
     {
         Matrix<T, M, N> res;
         const Matrix<T, M, N> &self = *this;

         for (uint8_t i = 0; i < M; i++) {
             for (uint8_t j = 0; j < N; j++) {
                 res(i, j) = self(i, j)*other(i, j);
             }
         }

         return res;
     }

     Matrix<T, M, N> edivide(const Matrix<T, M, N> &other) const
     {
         Matrix<T, M, N> res;
         const Matrix<T, M, N> &self = *this;

         for (uint8_t i = 0; i < M; i++) {
             for (uint8_t j = 0; j < N; j++) {
                 res(i, j) = self(i, j)/other(i, j);
             }
         }

         return res;
     }

     Matrix<T, M, N> operator+(const Matrix<T, M, N> &other) const
     {
         Matrix<T, M, N> res;
         const Matrix<T, M, N> &self = *this;

         for (uint8_t i = 0; i < M; i++) {
             for (uint8_t j = 0; j < N; j++) {
                 res(i, j) = self(i, j) + other(i, j);
             }
         }

         return res;
     }

     Matrix<T, M, N> operator-(const Matrix<T, M, N> &other) const
     {
         Matrix<T, M, N> res;
         const Matrix<T, M, N> &self = *this;

         for (uint8_t i = 0; i < M; i++) {
             for (uint8_t j = 0; j < N; j++) {
                 res(i, j) = self(i, j) - other(i, j);
             }
         }

         return res;
     }

     // unary minus
     Matrix<T, M, N> operator-() const
     {
         Matrix<T, M, N> res;
         const Matrix<T, M, N> &self = *this;

         for (uint8_t i = 0; i < M; i++) {
             for (uint8_t j = 0; j < N; j++) {
                 res(i, j) = -self(i, j);
             }
         }

         return res;
     }

     void operator+=(const Matrix<T, M, N> &other)
     {
         Matrix<T, M, N> &self = *this;
         self = self + other;
     }

     void operator-=(const Matrix<T, M, N> &other)
     {
         Matrix<T, M, N> &self = *this;
         self = self - other;
     }

     template<uint8_t P>
     void operator*=(const Matrix<T, N, P> &other)
     {
         Matrix<T, M, N> &self = *this;
         self = self * other;
     }

     /**
      * Scalar Operations
      */

     Matrix<T, M, N> operator*(T scalar) const
     {
         Matrix<T, M, N> res;
         const Matrix<T, M, N> &self = *this;

         for (uint8_t i = 0; i < M; i++) {
             for (uint8_t j = 0; j < N; j++) {
                 res(i, j) = self(i, j) * scalar;
             }
         }

         return res;
     }

     inline Matrix<T, M, N> operator/(T scalar) const
     {
         return (*this)*(1/scalar);
     }

     Matrix<T, M, N> operator+(T scalar) const
     {
         Matrix<T, M, N> res;
         const Matrix<T, M, N> &self = *this;

         for (uint8_t i = 0; i < M; i++) {
             for (uint8_t j = 0; j < N; j++) {
                 res(i, j) = self(i, j) + scalar;
             }
         }

         return res;
     }

     inline Matrix<T, M, N> operator-(T scalar) const
     {
         return (*this) + (-1*scalar);
     }

     void operator*=(T scalar)
     {
         Matrix<T, M, N> &self = *this;

         for (uint8_t i = 0; i < M; i++) {
             for (uint8_t j = 0; j < N; j++) {
                 self(i, j) = self(i, j) * scalar;
             }
         }
     }

     void operator/=(T scalar)
     {
         Matrix<T, M, N> &self = *this;
         self = self * (1.0f / scalar);
     }

     inline void operator+=(T scalar)
     {
         *this = (*this) + scalar;
     }

     inline void operator-=(T scalar)
     {
         *this = (*this) - scalar;
     }

private:
    T _data[M][N];
};

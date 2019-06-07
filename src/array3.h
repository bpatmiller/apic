#pragma once

#include <cstring> // used for memset
#include <glm/glm.hpp>

template <class T> struct Array3 {
  int sx, sy, sz;
  int size;
  T *data;

  Array3() {
    sx = 0;
    sy = 0;
    sz = 0;
    size = 0;
  }

  Array3(int sx_, int sy_, int sz_)
      : sx(sx_), sy(sy_), sz(sz_), size(0), data(0) {
    init();
  }

  void init() {
    size = sx * sy * sz;
    data = new T[size];
    clear();
  }

  void init(int sx_, int sy_, int sz_) {
    sx = sx_;
    sy = sy_;
    sz = sz_;
    size = sx * sy * sz;
    data = new T[size];
    clear();
  }

  T infnorm() const {
    T n = 0;
    for (int i = 0; i < size; i++) {
      if (std::fabs(data[i]) > n)
        n = std::fabs(data[i]);
    }
    return n;
  }

  // index (i,j,k cell index), coords (0-1f uv coords)
  T trilerp(glm::ivec3 index, glm::vec3 coords) {
    return (1 - coords.x) * (1 - coords.y) * (1 - coords.z) *
               (*this)(index.x, index.y, index.z) +
           (1 - coords.x) * (1 - coords.y) * (coords.y) *
               (*this)(index.x, index.y, index.z + 1) +
           (1 - coords.x) * (coords.y) * (1 - coords.z) *
               (*this)(index.x, index.y + 1, index.z) +
           (1 - coords.x) * (coords.y) * (coords.y) *
               (*this)(index.x, index.y + 1, index.z + 1) +
           (coords.x) * (1 - coords.y) * (1 - coords.z) *
               (*this)(index.x + 1, index.y, index.z) +
           (coords.x) * (1 - coords.y) * (coords.y) *
               (*this)(index.x + 1, index.y, index.z + 1) +
           (coords.x) * (coords.y) * (1 - coords.z) *
               (*this)(index.x + 1, index.y + 1, index.z) +
           (coords.x) * (coords.y) * (coords.y) *
               (*this)(index.x + 1, index.y + 1, index.z + 1);
  }

  void clear() { std::memset(data, 0, size * sizeof(T)); }

  T &operator()(int i, int j, int k) { return data[i + sx * j + sx * sy * k]; }
};

typedef Array3<float> Array3f;
typedef Array3<int> Array3i;
typedef Array3<glm::vec3> Array3v3;
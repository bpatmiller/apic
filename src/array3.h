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

  void clear() { std::memset(data, 0, size * sizeof(T)); }

  T &operator()(int i, int j, int k) { return data[i + sx * j + sx * sy * k]; }
};

typedef Array3<float> Array3f;
typedef Array3<int> Array3i;
typedef Array3<glm::vec3> Array3v3;
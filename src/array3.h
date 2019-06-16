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

  ~Array3() {
    delete[] data;
    data = 0;
    sx = 0;
    sy = 0;
    sz = 0;
    size = 0;
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

  double dot(Array3 &other) {
    double d = 0;
    for (int i = 0; i < size; i++)
      d += data[i] * other.data[i];
    return d;
  }

  void inc(double s, Array3 &other) {
    for (int i = 0; i < size; i++) {
      data[i] += s * other.data[i];
    }
  }

  void scale_inc(double s, Array3 &other) {
    for (int i = 0; i < size; i++) {
      data[i] = s * data[i] + other.data[i];
    }
  }

  void copy(Array3 &other) const {
    std::memcpy(other.data, data, other.size * sizeof(T));
  }

  T trilerp(glm::ivec3 index, glm::vec3 coords, int range) {
    int max_w = (2 * range) - 1;                         // (2 * 1) - 1 = 1
    float scale = 1.0f / (float)(max_w * max_w * max_w); // 1 * 1 * 1 = 1
    T sum = 0;
    float w;

    for (int i = 1 - range; i <= range; i++) { // {0,1}
      for (int j = 1 - range; j <= range; j++) {
        for (int k = 1 - range; k <= range; k++) {
          if (index.x + i < 0 || index.x + i >= sx || index.y + j < 0 ||
              index.y + j >= sy || index.z + k < 0 || index.z + k >= sz)
            continue;
          w = (max_w - std::fabs(i - coords.x)) * // {1 - coords.x | coords.x}
              (max_w - std::fabs(j - coords.y)) * // {""}
              (max_w - std::fabs(k - coords.z)) * scale; //{""}
          sum += (*this)(index.x + i, index.y + j, index.z + k) * w;
        }
      }
    }
    return sum;
  }

  // index (i,j,k cell index), coords (0-1f uv coords)
  T trilerp(glm::ivec3 index, glm::vec3 coords) {
    return (1 - coords.x) * (1 - coords.y) * (1 - coords.z) *
               (*this)(index.x, index.y, index.z) +
           (1 - coords.x) * (1 - coords.y) * (coords.z) *
               (*this)(index.x, index.y, index.z + 1) +
           (1 - coords.x) * (coords.y) * (1 - coords.z) *
               (*this)(index.x, index.y + 1, index.z) +
           (1 - coords.x) * (coords.y) * (coords.z) *
               (*this)(index.x, index.y + 1, index.z + 1) +
           (coords.x) * (1 - coords.y) * (1 - coords.z) *
               (*this)(index.x + 1, index.y, index.z) +
           (coords.x) * (1 - coords.y) * (coords.z) *
               (*this)(index.x + 1, index.y, index.z + 1) +
           (coords.x) * (coords.y) * (1 - coords.z) *
               (*this)(index.x + 1, index.y + 1, index.z) +
           (coords.x) * (coords.y) * (coords.z) *
               (*this)(index.x + 1, index.y + 1, index.z + 1);
  }

  void clear() { std::memset(data, 0, size * sizeof(T)); }

  T &operator()(int i, int j, int k) {
    return data[i + (sx * j) + (sx * sy * k)];
  }
};

typedef Array3<double> Array3d;
typedef Array3<float> Array3f;
typedef Array3<int> Array3i;
typedef Array3<glm::vec3> Array3v3;
typedef Array3<glm::dvec4> Array3v4;
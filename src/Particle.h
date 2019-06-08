#pragma once

#include <glm/glm.hpp>

class Particle {
public:
  glm::vec3 position;
  float pad0_;
  glm::vec3 velocity;
  float pad1_;

  Particle(glm::vec3 p) {
    position = p;
    velocity = glm::vec3(0, -2.5, 0);
  }
};
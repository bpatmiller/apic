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
    velocity = glm::vec3(0, 0, 0);
  }

  Particle(glm::vec3 p, glm::vec3 v) {
    position = p;
    velocity = v;
  }
};
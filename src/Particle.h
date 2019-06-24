#pragma once

#include <glm/glm.hpp>

class Particle {
public:
  glm::vec3 position;
  float mass;
  glm::vec3 velocity;
  float pad1_;

  Particle(glm::vec3 p) {
    position = p;
    mass = 1.0f;
    velocity = glm::vec3(0, 0, 0);
  }

  Particle(glm::vec3 p, glm::vec3 v) {
    position = p;
    velocity = v;
    mass = 1.0f;
  }
};
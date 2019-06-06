#pragma once

#include <glm/glm.hpp>

class Particle {
public:
  glm::vec3 position;
  glm::vec3 velocity;

  Particle(glm::vec3 p) {
    position = p;
    velocity = glm::vec3(0);
  }
};
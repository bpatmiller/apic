#pragma once

#include <glm/glm.hpp>

class Particle {
public:
  glm::vec3 position;
  float mass = 1.0f;
  glm::vec3 velocity;
  int id = 1;

  Particle(glm::vec3 p) {
    position = p;
    velocity = glm::vec3(0, 0, 0);
  }

  Particle(glm::vec3 p, glm::vec3 v) {
    position = p;
    velocity = v;
  }
};
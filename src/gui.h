#pragma once

#include <glad/glad.h>

#include "gl/program.h"
#include "gl/vao.h"
#include "simulation.h"
#include <GLFW/glfw3.h>

class GUI {
public:
  GUI(GLFWwindow *w) { window = w; }
  void init(float lx_, int nx_, int ny_, int nz_);
  void update();

  // simulation data
  Simulation simulation;

  // gl variables
  GLFWwindow *window;
  glm::ivec2 window_dims;

  // camera variables
  glm::quat orientation = glm::quat(glm::mat4(1));
  glm::vec3 eye = glm::vec3(.5, .5, 4);
  glm::vec3 base_eye = glm::vec3(.5, .5, 4);
  glm::vec3 focus = glm::vec3(.5, .5, 0);
  glm::mat4 view_matrix;
  glm::mat4 projection_matrix;

  // gl program/vao
  Program fluid_program;
  VAO fluid;
  std::vector<glm::uvec3> sphere_indices;

  // helper functions
  void create_sphere(float Radius, std::vector<glm::vec3> &s_vertices);
};
#pragma once

#include <glad/glad.h>

#include "gl/program.h"
#include "gl/vao.h"
#include "simulation.h"
#include <GLFW/glfw3.h>
#include <map>

class GUI {
public:
  GUI(GLFWwindow *w) { window = w; }
  void init(float lx_, int nx_, int ny_, int nz_);
  void update();
  void update_camera();

  // simulation data
  Simulation simulation;

  // gl variables
  GLFWwindow *window;
  glm::ivec2 window_dims;

  // key controls
  std::map<int, bool> keyHeld;

  // mouse controls
  bool mouse_pressed = false;
  glm::vec2 mouse_pos = glm::vec2(-1, -1);
  glm::vec2 mouse_pos_prev = glm::vec2(-1, -1);
  glm::vec2 mouse_diff = glm::vec2(0, 0);

  // camera variables
  glm::quat orientation = glm::quat(glm::mat4(1));
  glm::vec3 eye = glm::vec3(0, 0, 4);
  glm::vec3 base_eye = glm::vec3(0, 0, 4);
  glm::vec3 focus = glm::vec3(0, 0, 0);
  glm::mat4 view_matrix;
  glm::mat4 projection_matrix;
  float pitch = 0;
  float yaw = 0;

  // gl program/vao
  Program fluid_program;
  VAO fluid;
  std::vector<glm::uvec3> sphere_indices;

  Program grid_program;
  VAO grid_vao;
  std::vector<glm::uvec3> box_indices;
  std::vector<glm::vec4> grid_offsets;
  bool draw_grid = true;

  // helper functions
  void create_sphere(float Radius, std::vector<glm::vec3> &s_vertices);
};
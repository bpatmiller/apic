#include "gui.h"

const glm::vec3 FORWARD(0, 0, -1);
const glm::vec3 SIDE(1, 0, 0);
const glm::vec3 UP(0, 1, 0);

void GUI::create_sphere(float Radius, std::vector<glm::vec3> &s_vertices) {
  int Stacks = 4;
  int Slices = 4;
  s_vertices.clear();

  for (int i = 0; i <= Stacks; ++i) {
    float V = i / (float)Stacks;
    float phi = V * glm::pi<float>();
    for (int j = 0; j <= Slices; ++j) {
      float U = j / (float)Slices;
      float theta = U * (glm::pi<float>() * 2);
      float x = cosf(theta) * sinf(phi);
      float y = cosf(phi);
      float z = sinf(theta) * sinf(phi);
      s_vertices.push_back(glm::vec3(x, y, z) * Radius);
    }
  }

  for (int i = 0; i < Slices * Stacks + Slices; ++i) {
    sphere_indices.emplace_back(glm::uvec3(i, i + Slices + 1, i + Slices));
    sphere_indices.emplace_back(glm::uvec3(i + Slices + 1, i, i + 1));
  }
}

void GUI::init(float lx_, int nx_, int ny_, int nz_) {
  // set up simulation
  simulation.init(lx_, nx_, ny_, nz_);
  simulation.add_particle_box();

  // compile shaders
  fluid_program =
      Program("src/shaders/particle.vs", "", "src/shaders/particle.fs", "");

  // set up particle VAO
  std::vector<glm::vec3> sphere_vertices;
  create_sphere(simulation.grid.h * 0.1, sphere_vertices);
  fluid.setLayout({3}, false);
  fluid.setLayout({3, 1, 3, 1}, true);
  fluid.vb.set(sphere_vertices);
  fluid.ib.set(simulation.particles);

  // some gl settings
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glEnable(GL_DEPTH_TEST);
  //   glEnable(GL_CULL_FACE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDepthFunc(GL_LESS);
  //   glCullFace(GL_BACK);
  //   glLineWidth(2.0f);
}

void GUI::update() {
  // update camera vars
  glfwGetWindowSize(window, &window_dims.x, &window_dims.y);
  eye =
      focus + glm::vec3(glm::mat4_cast(orientation) * glm::vec4(base_eye, 1.0));
  view_matrix = glm::lookAt(eye, focus, UP);
  projection_matrix = glm::perspective(
      glm::radians(60.0f), ((float)window_dims.x) / window_dims.y, 0.01f, 30.f);

  // clear the renderer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // step the simulation, copy the new particle data
  float frame_time = 0.05;
  simulation.step_frame(frame_time);
  fluid.ib.update(simulation.particles, 0);

  // use/render the fluid
  fluid_program.use();
  fluid_program.setMat4("projection", projection_matrix);
  fluid_program.setMat4("view", view_matrix);
  fluid.bind();
  glDrawElementsInstanced(GL_TRIANGLES, sphere_indices.size() * 3,
                          GL_UNSIGNED_INT, sphere_indices.data(),
                          simulation.particles.size());
}

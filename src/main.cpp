#include <glad/glad.h>

#include "gui.h"
#include <GLFW/glfw3.h>
#include <getopt.h>
#include <iostream>

// ERROR CALLBACK
void glfw_error_callback(int error, const char *description) {
  fprintf(stderr, "Error: %s\n", description);
}

// KEY CALLBACKS
void KeyCallback(GLFWwindow *window, int key, int scancode, int action,
                 int mods) {

  GUI *gui = (GUI *)glfwGetWindowUserPointer(window);

  if (key == GLFW_KEY_Q) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
    return;
  } else if (key == GLFW_KEY_E && action == GLFW_RELEASE) {
    std::cout << "exporting particles" << std::endl;
    gui->simulation.save_voxels(std::string("unnamed"));
  } else if (key == GLFW_KEY_G && action == GLFW_RELEASE) {
    gui->draw_grid = !gui->draw_grid;
  } else if (key == GLFW_KEY_V && action == GLFW_RELEASE) {
    gui->draw_velocity = !gui->draw_velocity;
  } else if (key == GLFW_KEY_B && action == GLFW_RELEASE) {
    gui->draw_particles = !gui->draw_particles;
  } else if (key == GLFW_KEY_1 && action == GLFW_RELEASE) {
    gui->simulation.mode = PIC_MODE;
    std::cout << "PIC mode" << std::endl;
  } else if (key == GLFW_KEY_2 && action == GLFW_RELEASE) {
    gui->simulation.mode = PIC_FLIP_MODE;
    std::cout << "PIC/FLIP mode (" << gui->simulation.flip_blend << ")"
              << std::endl;
  } else if (key == GLFW_KEY_3 && action == GLFW_RELEASE) {
    gui->simulation.mode = APIC_MODE;
    std::cout << "APIC mode" << std::endl;
  } else if (key == GLFW_KEY_O && action == GLFW_RELEASE) {
    std::cout << "resetting simulation" << std::endl;
    gui->simulation.reset();
    // manually update one frame
    gui->update(true);
  }

  if (action == GLFW_PRESS) {
    gui->keyHeld[key] = true;
  } else if (action == GLFW_RELEASE) {
    gui->keyHeld[key] = false;
  }
}

void MouseButtonCallback(GLFWwindow *window, int button, int action, int mods) {
  GUI *gui = (GUI *)glfwGetWindowUserPointer(window);
  if (button == GLFW_MOUSE_BUTTON_LEFT && action != GLFW_RELEASE) {
    gui->mouse_pressed = true;
  } else {
    gui->mouse_pressed = false;
  }
}

void MousePosCallback(GLFWwindow *window, double mouse_x, double mouse_y) {
  GUI *gui = (GUI *)glfwGetWindowUserPointer(window);
  gui->mouse_pos = glm::vec2(mouse_x, mouse_y);
}

int main(int argc, char *argv[]) {
  int opt;
  bool g = false;
  float t = 0.0f;
  bool e = false;
  bool a = false;
  std::string o;
  int m = 2;

  // -g to enable graphical mode
  // -t to set a time limit
  // -e to export files
  // -a to export each frame
  // -o to set output directory
  // -m to set mode (1 pic, 2 pic/flip, 3 apic)

  while ((opt = getopt(argc, argv, "t:o:m:gea")) != -1) {
    switch (opt) {
    case 'g':
      g = true;
      std::cout << ":: graphical mode enabled" << std::endl;
      break;
    case 't':
      t = std::atof(optarg);
      std::cout << ":: simulation time set to " << optarg << " seconds"
                << std::endl;
      break;
    case 'e':
      e = true;
      std::cout << ":: exporting frame enabled" << std::endl;
      break;
    case 'a':
      a = true;
      std::cout << ":: exporting all frames enabled" << std::endl;
      break;
    case 'o':
      o = std::string(optarg);
      std::cout << ":: output filename set to " << o << std::endl;
      break;
    case 'm':
      m = std::atoi(optarg);
      std::cout << ":: mode set to " << m << std::endl;
      break;
    }
  }

  if (g) {
    // create window/init glfw
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
      exit(EXIT_FAILURE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    GLFWwindow *window = glfwCreateWindow(640, 480, "apic", NULL, NULL);
    if (!window) {
      throw std::runtime_error("glfwCreateWindow error");
    }
    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
    glfwSwapInterval(1);
    GUI gui(window);
    glfwSetWindowUserPointer(window, &gui);
    // key / mouse callbacks
    glfwSetKeyCallback(window, KeyCallback);
    glfwSetCursorPosCallback(window, MousePosCallback);
    glfwSetMouseButtonCallback(window, MouseButtonCallback);

    int grid_res = 20;
    gui.init(2.0f, grid_res, grid_res, grid_res);
    gui.simulation.mode = m;
    while (!glfwWindowShouldClose(window)) {
      gui.update();
      glfwSwapBuffers(window);
      glfwPollEvents();
    }
  }
  // no gui mode
  else {
    Simulation sim;
    sim.mode = m;
    int grid_res = 50;
    std::cout << ":: initializing grid" << std::endl;
    sim.init(2.0f, grid_res, grid_res, grid_res);
    std::cout << ":: initializing particles" << std::endl;
    sim.add_particle_box();
    std::cout << ":: " << sim.particles.size() << " particles added"
              << std::endl;
    std::cout << ":: running simulation" << std::endl;
    if (a) {
      sim.step_and_save(t, o);
    } else if (e) {
      sim.step_frame(t);
      sim.save_particles(o);
    }
  }
  std::cout << ":: simulation complete" << std::endl;
}
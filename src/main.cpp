#include <glad/glad.h>

#include "geom.h"
#include "gui.h"
#include <GLFW/glfw3.h>
#include <getopt.h>
#include <iostream>

// ERROR CALLBACK
void glfw_error_callback(int error, const char *description) {
  fprintf(stderr, "Error %i: %s\n", error, description);
}

// KEY CALLBACKS
void KeyCallback(GLFWwindow *window, int key, int scancode, int action,
                 int mods) {

  GUI *gui = (GUI *)glfwGetWindowUserPointer(window);

  if (key == GLFW_KEY_Q) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
    return;
  } else if (key == GLFW_KEY_M && action == GLFW_RELEASE) {
    std::cout << "polygonizing particles and exporting mesh\n";
    gui->simulation.generate_mesh();
    gui->simulation.save_mesh(std::string("mesh.ply"));
  } else if (key == GLFW_KEY_E && action == GLFW_RELEASE) {
    std::cout << "exporting particles" << std::endl;
    gui->simulation.save_voxels(std::string("unnamed.ply"));
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
  } else if (key == GLFW_KEY_MINUS && action == GLFW_RELEASE) {
    std::cout << "cycling example type" << std::endl;
    gui->simulation.example_type = ((gui->simulation.example_type + 1) % 3);
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
  int m = APIC_MODE;
  int x = 0;
  int r = 30;

  std::string help_text = "  -g to enable graphical mode\n\
  -t to set a time limit (for non graphical mode)\n\
  -e to export files (for non-graphical mode)\n\
  -a to export each frame (for non-graphical mode)\n\
  -o to set output filename (to be created in the out/ directory)\n\
  -m to set mode (1 pic, 2 pic/flip, 3 apic)\n\
  -x to choose example type (1 dam break, 2 center drop, 3 for opposing corners)\n\
  -r to set grid resolution (default 30)\n\
  -h to display help text\n";

  while ((opt = getopt(argc, argv, "t:o:m:x:r:geah")) != -1) {
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
      m = std::atoi(optarg) - 1;
      std::cout << ":: mode set to " << m << std::endl;
      break;
    case 'x':
      x = std::atoi(optarg) - 1;
      std::cout << ":: example type set to " << x << std::endl;
      break;
    case 'r':
      r = std::atoi(optarg);
      std::cout << ":: grid resolution set to " << r << std::endl;
      break;
    case 'h':
      std::cout << help_text;
      return 0;
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

    gui.init(2.0f, r, r, r, x);
    gui.simulation.mode = m;
    voxelize_mesh("mesh/dragon.ply", gui.simulation.grid,
                  glm::vec3(0.5f, 0.1f, 1.0f), SOLID_CELL);

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
    sim.example_type = x;
    std::cout << ":: initializing grid" << std::endl;
    sim.init(2.0f, r, r, r);
    // std::cout << ":: importing mesh (solid)\n";
    // voxelize_mesh("mesh/dragon.ply", sim.grid, glm::vec3(0.5f, 0.1f, 1.0f),
    //               FLUID_CELL);
    std::cout << ":: importing mesh (fluid)\n";
    voxelize_mesh("mesh/dragon_l.ply", sim.grid, glm::vec3(1.0f, 0.75f, 1.0f),
                  FLUID_CELL);
    sim.reseed_particles();

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
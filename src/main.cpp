#include <glad/glad.h>

#include "gui.h"
#include <GLFW/glfw3.h>
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
  } else if (key == GLFW_KEY_G && action == GLFW_RELEASE) {
    gui->draw_grid = !gui->draw_grid;
  } else if (key == GLFW_KEY_V && action == GLFW_RELEASE) {
    gui->draw_velocity = !gui->draw_velocity;
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

  int grid_res = 25;
  gui.init(2.0f, grid_res, grid_res, grid_res);
  while (!glfwWindowShouldClose(window)) {
    gui.update();
    glfwSwapBuffers(window);
    glfwPollEvents();
  }
}
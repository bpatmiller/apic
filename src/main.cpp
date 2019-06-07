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
  if (key == GLFW_KEY_Q) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
    return;
  }
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

  gui.init(2.0f, 25, 25, 25);
  while (!glfwWindowShouldClose(window)) {
    gui.update();
    glfwSwapBuffers(window);
    glfwPollEvents();
  }
}
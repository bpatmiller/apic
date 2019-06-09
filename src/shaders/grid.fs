#version 430 core
out vec4 fragment_color;

in float p;

void main() {
  if (p <= 0) {
    fragment_color = vec4(1.0);
  } else {
    fragment_color = vec4(0.0, 0.5, (2 - p) * 0.25, clamp(3 - p, 0.15, 0.5));
  }
}
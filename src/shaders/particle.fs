#version 430 core
out vec4 fragment_color;

in vec3 v;

void main() { fragment_color = vec4(vec3(0.5) + v / 4.0, 1.0); }
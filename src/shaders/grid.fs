#version 430 core
out vec4 fragment_color;

in float p;

void main() { fragment_color = vec4(0.0, 0.5, (1-p)*0.25, 2 - p); }
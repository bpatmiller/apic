#version 430 core
out vec4 fragment_color;

in float p;

void main() { fragment_color = vec4(0.0, 0.5, (2 - p) * 0.5, clamp(3 - p,0.2,0.8)); }
#version 430 core
layout(location = 0) in vec3 vertex_position;
layout(location = 1) in vec4 offset_v;

uniform mat4 projection;
uniform mat4 view;

out float p;

void main() {
  if (p != 0) {
    p = offset_v.w;
    gl_Position = projection * view * vec4(offset_v.xyz + vertex_position, 1.0);
  } else {
    p = offset_v.w;
    gl_Position = projection * view * vec4(0.0, 0.0, 0.0, 1.0);
  }
}

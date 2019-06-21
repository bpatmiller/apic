#pragma once

#include "aabb_triangle_overlap.h"
#include "simulation.h"

// filename of .ply file
// grid to export voxels to
// offset of the file in grid space
void voxelize_mesh(std::string fname, Grid &grid, glm::vec3 offset) {
  std::vector<glm::vec3> vertices;
  std::vector<glm::uvec3> indices;

  std::ifstream in;
  std::string ln;
  int vertex_num = 0;
  int tri_num = 0;

  std::cout << "   loading mesh...";
  in.open(fname);

  // parse header
  while (std::getline(in, ln)) {
    std::stringstream ss(ln);
    std::string first;
    ss >> first;
    if (first == "element") {
      int n;
      ss >> first >> n;
      if (!vertex_num) {
        vertex_num = n;
      } else {
        tri_num = n;
      }
    } else if (first == "end_header") {
      break;
    }
  }

  // parse vertices
  for (int i = 0; i < vertex_num; i++) {
    float x, y, z;
    std::getline(in, ln);
    std::stringstream ss(ln);
    ss >> x >> y >> z;
    vertices.push_back(glm::vec3(x, y, z));
  }

  // parse indices
  for (int i = 0; i < tri_num; i++) {
    uint t, a, b, c;
    std::getline(in, ln);
    std::stringstream ss(ln);
    ss >> t >> a >> b >> c;
    indices.push_back(glm::uvec3(a, b, c));
  }

  std::cout << "done\n   voxelizing mesh...";

  // for each grid node, do AABB intersection
  for (int i = 0; i < grid.marker.sx; i++) {
    for (int j = 0; j < grid.marker.sy; j++) {
      for (int k = 0; k < grid.marker.sz; k++) {
      }
    }
  }

  std::cout << "done\n";
}
#pragma once
#include "simulation.h"

void save_mesh(std::string fname, std::vector<glm::uvec3> &indices,
               std::vector<glm::vec3> &vertices) {
  std::ofstream ofile(std::string("out/m_") + fname);
  int tri_count = indices.size();

  ofile << "ply\n";
  ofile << "format ascii 1.0\n";
  ofile << "element vertex " << vertices.size() << "\n";
  ofile << "property float x\n";
  ofile << "property float y\n";
  ofile << "property float z\n";
  ofile << "element face " << indices.size() << "\n";
  ofile << "property list uchar int vertex_index\n";
  ofile << "end_header\n";

  for (auto &v : vertices) {
    ofile << v.x << " " << v.z << " " << v.y << "\n";
  }
  for (auto &i : indices) {
    ofile << "3 " << i.x << " " << i.y << " " << i.z << "\n";
  }

  ofile.close();
  std::cout << "saved out/m_" << fname << " containing " << vertices.size()
            << " vertices\n";
}
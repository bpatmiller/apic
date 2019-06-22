#include "simulation.h"

void Simulation::save_particles(std::string fname) {
  std::ofstream ofile(std::string("out/p_") + fname);
  // header
  ofile << "ply\n";
  ofile << "format ascii 1.0\n";
  ofile << "element vertex " << particles.size() << "\n";
  ofile << "property float x\n";
  ofile << "property float y\n";
  ofile << "property float z\n";
  ofile << "end_header\n";

  for (auto &p : particles) {
    ofile << p.position.x << " " << p.position.z << " " << p.position.y << "\n";
  }
  ofile.close();
  std::cout << "   saved " << std::string("out/") + fname << std::endl;
}

void Simulation::save_voxels(std::string fname) {
  std::ofstream ofile(std::string("out/") + fname);
  int n = 0;
  for (int x = 0; x < grid.marker.size; x++) {
    if (grid.marker.data[x] == FLUID_CELL)
      n++;
  }
  // header
  ofile << "ply\n";
  ofile << "format ascii 1.0\n";
  ofile << "element vertex " << n << "\n";
  ofile << "property float x\n";
  ofile << "property float y\n";
  ofile << "property float z\n";
  ofile << "end_header\n";

  for (int i = 0; i < grid.marker.sx; i++) {
    for (int j = 0; j < grid.marker.sy; j++) {
      for (int k = 0; k < grid.marker.sz; k++) {
        if (grid.marker(i, j, k) == FLUID_CELL)
          ofile << i * grid.h << " " << k * grid.h << " " << j * grid.h << "\n";
      }
    }
  }
  ofile.close();
  std::cout << "   saved " << std::string("out/") + fname << std::endl;
}

void Simulation::save_mesh(std::string fname) {
  std::ofstream ofile(std::string("out/m_") + fname);

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

void Simulation::generate_mesh() {
  std::vector<glm::vec3> positions(8);
  std::vector<float> values(8);
  int vert_count = 0;
  float offs = 0.5f * grid.h;

  vertices.clear();
  indices.clear();

  int elements[8] = {0, 3, 4, 7, 1, 2, 5, 6};

  for (int i = 0; i < grid.phi.sx - 1; i++) {
    for (int j = 0; j < grid.phi.sy - 1; j++) {
      for (int k = 0; k < grid.phi.sz - 1; k++) {
        positions.clear();
        values.clear();
        // pass in locations and values
        // of 8 neighboring sample points
        positions[elements[0]] =
            glm::vec3(i * grid.h + offs, j * grid.h + offs, k * grid.h + offs);
        positions[elements[1]] = glm::vec3(i * grid.h + offs, j * grid.h + offs,
                                           (k + 1) * grid.h + offs);
        positions[elements[2]] = glm::vec3(
            i * grid.h + offs, (j + 1) * grid.h + offs, k * grid.h + offs);
        positions[elements[3]] =
            glm::vec3(i * grid.h + offs, (j + 1) * grid.h + offs,
                      (k + 1) * grid.h + offs);
        positions[elements[4]] = glm::vec3(
            (i + 1) * grid.h + offs, j * grid.h + offs, k * grid.h + offs);
        positions[elements[5]] =
            glm::vec3((i + 1) * grid.h + offs, j * grid.h + offs,
                      (k + 1) * grid.h + offs);
        positions[elements[6]] =
            glm::vec3((i + 1) * grid.h + offs, (j + 1) * grid.h + offs,
                      k * grid.h + offs);
        positions[elements[7]] =
            glm::vec3((i + 1) * grid.h + offs, (j + 1) * grid.h + offs,
                      (k + 1) * grid.h + offs);

        values[elements[0]] = glm::clamp(grid.phi(i, j, k), -0.5f, 0.5f);
        values[elements[1]] = glm::clamp(grid.phi(i, j, k + 1), -0.5f, 0.5f);
        values[elements[2]] = glm::clamp(grid.phi(i, j + 1, k), -0.5f, 0.5f);
        values[elements[3]] =
            glm::clamp(grid.phi(i, j + 1, k + 1), -0.5f, 0.5f);
        values[elements[4]] = glm::clamp(grid.phi(i + 1, j, k), -0.5f, 0.5f);
        values[elements[5]] =
            glm::clamp(grid.phi(i + 1, j, k + 1), -0.5f, 0.5f);
        values[elements[6]] =
            glm::clamp(grid.phi(i + 1, j + 1, k), -0.5f, 0.5f);
        values[elements[7]] =
            glm::clamp(grid.phi(i + 1, j + 1, k + 1), -0.5f, 0.5f);

        polygonize(positions, values, vert_count, indices, vertices);
      }
    }
  }
}
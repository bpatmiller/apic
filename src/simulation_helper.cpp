#include "simulation.h"

// helper methods for seeding particles/creating examples

void Simulation::populate_particles() {
  reseed_count = 0;
  particles.clear();
  cx.clear();
  cy.clear();
  cz.clear();
  add_dam_break();
  // add_pool();
  // add_center_drop();
  // add_raleigh_taylor();
}

void Simulation::reseed_cell(int i, int j, int k, int id, float mass,
                             float viscosity) {
  if (grid.marker(i, j, k) != SOLID_CELL) {
    float base_x = i * grid.h;
    float base_y = j * grid.h;
    float base_z = k * grid.h;
    for (int i = 0; i < 8; i++) {
      float jitter_x = glm::linearRand(0 + EPS, grid.h - EPS);
      float jitter_y = glm::linearRand(0 + EPS, grid.h - EPS);
      float jitter_z = glm::linearRand(0 + EPS, grid.h - EPS);
      // add particles
      Particle p(
          glm::vec3(base_x + jitter_x, base_y + jitter_y, base_z + jitter_z),
          glm::vec3(0, 0, 0));
      p.id = id;
      p.mass = mass;
      // p.viscosity = viscosity;
      particles.push_back(p);
      // APIC vectors
      cx.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
      cy.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
      cz.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
    }
    dirty = true;
  }
}

void Simulation::reseed_particles() {
  reseed_count = 0;
  particles.clear();
  cx.clear();
  cy.clear();
  cz.clear();
  for (int i = 0; i < grid.marker.sx; i++) {
    for (int j = 0; j < grid.marker.sy; j++) {
      for (int k = 0; k < grid.marker.sz; k++) {
        if (grid.marker(i, j, k) == FLUID_CELL) {
          reseed_cell(i, j, k);
        }
      }
    }
  }
}

void Simulation::emit_particles() {
  for (auto &e : emitters) {
    for (int i = 0, imax = (int)e.rate; i < imax; i++) {
      float base_x = e.position.x;
      float base_y = e.position.y;
      float base_z = e.position.z;
      for (int i = 0; i < 8; i++) {
        float jitter_x = glm::linearRand(-e.radius, e.radius) * e.scale.x;
        float jitter_y = glm::linearRand(-e.radius, e.radius) * e.scale.y;
        float jitter_z = glm::linearRand(-e.radius, e.radius) * e.scale.z;
        // add particles
        particles.push_back(Particle(
            glm::vec3(base_x + jitter_x, base_y + jitter_y, base_z + jitter_z),
            e.direction));
        // APIC vectors
        cx.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
        cy.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
        cz.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
      }
    }
  }
}

void Simulation::add_dam_break() {
  float dens = 8.0f;
  float visc = 1.0f;
  int id = grid.add_fluid(dens, visc);
  for (int x = grid.nx * 0.75; x < grid.nx * 0.95; x++) {
    for (int y = grid.ny * 0.1; y < grid.ny * 0.6; y++) {
      for (int z = grid.nz * 0.1; z < grid.nz * 0.9; z++) {
        // for each cell, add 8 new jittered particles
        reseed_cell(x, y, z, id, 1.0f, visc);
      }
    }
  }
  dens = 4.0f;
  visc = 0.5f;
  id = grid.add_fluid(dens, visc);
  for (int x = grid.nx * 0.05; x < grid.nx * 0.25; x++) {
    for (int y = grid.ny * 0.1; y < grid.ny * 0.6; y++) {
      for (int z = grid.nz * 0.1; z < grid.nz * 0.9; z++) {
        // for each cell, add 8 new jittered particles
        reseed_cell(x, y, z, id, 1.0f, visc);
      }
    }
  }
}

void Simulation::add_center_drop() {
  for (int x = grid.nx * 0.3; x < grid.nx * 0.7; x++) {
    for (int y = grid.ny * 0.5; y < grid.ny * 0.8; y++) {
      for (int z = grid.nz * 0.3; z < grid.nz * 0.7; z++) {
        // for each cell, add 8 new jittered particles
        reseed_cell(x, y, z);
      }
    }
  }
}

void Simulation::add_pool() {
  for (int x = 1; x < grid.nx - 1; x++) {
    for (int y = 1; y < grid.ny * 0.2; y++) {
      for (int z = 1; z < grid.nz - 1; z++) {
        // for each cell, add 8 new jittered particles
        reseed_cell(x, y, z);
      }
    }
  }
}

void Simulation::add_raleigh_taylor() {
  for (int x = 1; x < grid.nx - 1; x++) {
    for (int y = 1; y < grid.ny * 0.2; y++) {
      for (int z = 1; z < grid.nz - 1; z++) {
        // for each cell, add 8 new jittered particles
        reseed_cell(x, y, z, 1);
      }
    }
  }
  for (int x = 1; x < grid.nx - 1; x++) {
    for (int y = grid.ny * 0.2; y < grid.ny * 0.4; y++) {
      for (int z = 1; z < grid.nz - 1; z++) {
        // for each cell, add 8 new jittered particles
        reseed_cell(x, y, z, 2);
      }
    }
  }
}
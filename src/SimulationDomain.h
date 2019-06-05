#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "Particle.h"
#include "Cell.h"

class SimulationDomain {
    glm::vec3 min_bounds;
    glm::vec3 max_bounds;

    // we will have n^3 total cells
    int divisions;

    std::vector<Particle> particles;

    std::vector<Cell> cells;

    // add particles in a given bounding box
    void add_particle_box(glm::vec3 min, glm::vec3 max);


    

};
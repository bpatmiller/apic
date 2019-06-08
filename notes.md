
high level overview (animating sand as fluid, 2005)

- transfer to grid (trilinear interpolation)
    * for each particle:
        * get all grid nodes (velocity) within 2h distance by L1
        * for each of these nodes:
            * trilinearly interpolate particle velocity with grid node
- add gravity
    * for each v grid node (y velocity):
        * add (g * dt) to v
- compute distance to fluid (phi)
    * "init phi"
    * for i, 2 times:
        "sweep phi"
- extend velocity
    * for i, 4 times:
        * "sweep velocity"
- apply boundary conditions
    * set all boundary cells to solid
    * set flow to 0 on boundary
- make incompressible(project)
    * find discrete divergence
    * solve pressure
    * add the pressure gradient to the velocity field
- extend velocity (again)
    * call the same function
- transfer to particles
    * pretty much the same as transfer to grid
- advect (RK2 (RK4?))
    * yeet
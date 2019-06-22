## done
-camera controls
-show velocity field
-extend velocity
-compute density
-compute A,b
-pressure solve
-velocity from pressure gradient
-add affine transform
-fix numerical dissapation (gravity? boundary conditions?) spoiler alert it was something dumb like my sweep functions
-switch to eigen for pressure solve
-getopt
    -g to enable graphical mode
    -t to set a time limit
    -e to export files
    -a to export each frame
    -o to set output directory
-change cmake error flags
-refactor sweep code (did not do this, sweep code is fine)
-import obstacles

## todo
-emitters
-refactor export code
-optimization
-viscosity
-particle level set method
-mesh cleanup (remove doubles, correct normals)
-explicitly compute weight gradient
-load solid/fluid files
-fix boundary conditions (clinging)

## further work
-set up partio
-use openVDB
-mesh loading
# RigidBodyPhysics

This program simulates the completely elastic collisions between perfect spheres enclosed by a rectangular box, subjected to a uniform gravitational field. The simulation is recorded as vertical (time axis) data for each particle in a text file. The user specifies the number of frames to record and a time parameter, as well as the initial positions of particles. Several useful functions have been implemented to that end.

Unlike the vast majority of implementations, this simulation is exact to the precision of single-precision floating point numbers. Most implementations operate with a constant (or variable, depending on the speed of the fastest particle), time step, whereas this implementation has no specific time step at all. With the formula for the time until collision between spheres of radii r1 and r2, one can find the next ensuing collision from any point in the simulation. It iterates to the next collision and then proceeds to calculate when the next collision is going to be. The current implementation stores an NxN array of potential collisions in order to reduce the necessary number of Collision calculations per time step from N^2 to 2N. This means that memory(N) = O(N^2). For small numbers of particles, I think this is the best solution. For a large number of particles, one must instead recalculate many collisions for each time step, leading to an O(N^2) scaling of necessary computations.

TODO
* Fix bugs in collision detection
* Add N-body gravitation
* Implement collision detection with convex surface
* Implement collision detection with concave surface

# OpenGL-interactive-particle-control-system
Interactive system that uses force based input to move a particle through a series of obstacles. The scope is to maintain motion, by applying force to compensate for inelastic collisions and drag.

---

## SCOPE
The goal for the user should be to maintain motion for as long as possible, failure happens when kinetic energy becomes insufficient for too long (the user is not able to generate energy anymore): the project showcases loss of momentum caused by drag and inelastic collisions, the user can try to minimize energy dissipation by applying horizontal forces to move the particle, but with a limited total amount of energy that drains and regenerates at different times.

The scope of this project was to evolve a previous project, a minimal particle collision system, into an interactive system that allowed me to deepen my understanding of real-time physics and applying theoretical knowledge acquired through my bachelor studies.

---

## FEATURES
- environment:
  - gravity: the particle is affected by gravity at all times, but will reach a terminal velocity. Not only does this model real-life physics but it also prevents the particle from tunneling through obstacles at high speed.
  - drag: models viscous air friction
  - obstacles: randomly generated, with variable widths and increasing density as time passes to evolve the system's difficulty.
- collisions:
  - detection: circle-AABB detection, with a special case for when the particle is inside an obstacle to prevent unexpected behaviours. Not perfectly elastic and use different restitution whether it involves boundaries (walls) or obstacles.
  - substepping: added for higher velocity to guarantee stability.
- user input:
  - force: does not directly modify the position of the particle, but applies a force that gets added to the existing forces.
  - control: weakens to avoid infinite energy generation, quadratic scaling was used for more noticeable changes.

- rendering: modern OpenGL pipeline, with minimal changes to the one used in the previous project. It uses transformation matrices to avoid unnecessary data flow from cpu to gpu each time a different size of obstacle gets generated or movement happens.

---

## CODE STRUCTURE
- initializations: include glfw, glad, window, gpu initializations (VBOs, VAOs, and Shader Programs)
- main loop:
  - checks failure conditions
  - creates and destroys obstacles to balance resources used
  - calls the updates
  - moves the camera
- update:
  - applies gravity and input force
  - modifies velocity using acceleration and deltaTime (not frame rate dependent)
  - resolves collisions
- resolve collisions:
  - detects collisions
  - handles the response by calculating normal and relative velocity, with position correction
- obstacle generation:
  - position progresses with the camera and particle, but has a slight random factor going from a minimum step to a maximum step, which decreases with time
  - constant height but variable width
- input handling:
  - detects input and adds the force
  - drains the available energy and scales the force accordingly
  - available energy can regenerate when it is not used but at a lower rate
- rendering:
  - separated to maintain distinction between physics handled by cpu and rendering done by gpu
  - handles drawing the shapes using transformation matrices

---

## LIMITATIONS AND FUTURE WORK
- collision detection is not perfect and discrete, it required limiting velocity and substepping/multiple iterations to prevent tunneling.
- obstacle and collision detection is also not optimal, as the complexity is O(n), where n is the number of obstacles. In this case the number is limited, but there are possible optimizations such as space partitioning

- an AI agent that maximizes momentum preservation could be implemented in the future

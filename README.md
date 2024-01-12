# General-Mechanics-Simulation

Work in progress.. Was used to generate stress tests for the constraint solver to assist in system calibration.

Notes on some dependencies:
- SDL 2+
- Unofficial OpenGL Math library
- Uses OpenGL 4+
  - OpenGL3+ is fine, just changes some declarations of version in SDL Init and shaders


## Updates

### January 12, 2024
**Primary Change**
- N-Body NGon Testing
![Boxes on Boxes](https://github.com/DanRehberg/General-Mechanics-Simulation/blob/main/media/4gon5itrVerlet540p.gif)

**Pending Change**
- CoM Intersection with another body fixes poor line-line overlap
  - Works with Verlet Integration
  - Incorrect with Jacobian (Lagrange) approach
    - Needs a fixed separation axis based on hull of reference NGon
    - Distance constraint resolution nullifies velocity that could separate bodies
- Continuous collision detection implementation
  - Additional demonstration of stability change when using continuous collision detection

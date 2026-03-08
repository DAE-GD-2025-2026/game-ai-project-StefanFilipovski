# Game AI Programming - Stefan Filipovski 2DAE10

## Assignment 1: Flocking & Spatial Partitioning

### Overview
Implementation of flocking behavior using steering behaviors and spatial partitioning in Unreal Engine 5.6.

---

### Week 1 - Steering Behaviors
Implemented the following steering behaviors:
- **Seek** - Move toward target at max speed
- **Flee** - Move away from target
- **Arrive** - Seek with gradual slowdown near target using slow/target radius
- **Face** - Rotate toward target without moving
- **Pursuit** - Seek toward predicted future position of target
- **Evade** - Flee from predicted future position of target
- **Wander** - Random movement using a circle offset in front of the agent

---

### Week 2 - Flocking & Combined Steering
**Combined Steering:**
- **BlendedSteering** - Weighted average of multiple behaviors
- **PrioritySteering** - Uses first valid steering output from an ordered list

**Flocking behaviors (BlendedSteering):**
- **Cohesion** - Steer toward average neighbor position
- **Separation** - Move away from neighbors, inversely proportional to distance
- **Alignment** - Match average neighbor velocity

Combined into a **PrioritySteering** setup: Evade (with radius) Flocking BlendedSteering

---

### Week 3 - Spatial Partitioning
Implemented a flat spatial partitioning grid (CellSpace) to optimize neighbor registration:
- World divided into a uniform grid of cells
- Each cell tracks agents within its bounds
- Neighbors only check cells overlapping the neighborhood radius
- Agents update their cell when crossing boundaries
- Toggle between brute force and spatial partitioning with ImGui
- Performance improvement visible with large agent counts (My PC is very good)




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

---

## Assignment 2: NavMesh Pathfinding

### Overview
Implementation of A* pathfinding on a navigation mesh with path smoothing using the Simple Stupid Funnel Algorithm (SSFA).

---

### Week 5 - A* Pathfinding
- Implemented A* search algorithm with Euclidean heuristic
- Uses open/closed lists with NodeRecords tracking cost-so-far and estimated total cost
- Backtracks through connection history to reconstruct the final path

---

### Week 6 - Navigation Mesh

**NavGraph Construction:**
- Nodes are placed at the midpoint of every edge shared by two triangles
- Connections are created between all nodes that share the same triangle (2 nodes = 1 connection, 3 nodes = 3 connections)
- Connection costs are set to Euclidean distance

**NavMesh Pathfinding:**
- Clones the navigation graph and adds temporary start/end nodes
- Start and end positions are snapped to the closest triangle in the navmesh
- Start/end nodes are connected to all valid edge nodes of their respective triangles
- A* runs on the cloned graph to find the initial path

**Path Smoothing (SSFA):**
- Portals are extracted from the A* path by looking up the edge each node sits on
- Portal endpoints are oriented as right/left relative to the path direction using a 2D cross product
- The funnel algorithm walks through portals, tightening the funnel and adding new apex points when legs cross over

### Known Issues
- Clicking very close to certain wall edges may not generate a path. This happens because some triangles near tight corners connect to graph nodes that are isolated from the main graph. Clicking slightly further from the wall resolves this, as the position snaps to a better-connected triangle.
# 2D Navigation System (Unity C#)

This project contains implementation of the 2D navigation system which can be used for games or other visual 2D simulations. The system includes pathfinding, agents movement along their paths, agents local avoidance and obstacles of any shapes. The navigation system comes in two parts - grid and navigation mesh. The navigation mesh is more advanced and faster. The navigation mesh based system also benefits from fast constrained triangulation calculations when using derived and adapted Delaunator and Constrainautor libraries (see ThirdPartyNotices.md).

There is also a C++ version of this project which uses Godot GDExtension. It can be found in https://github.com/rom8smi/NavigationSystemCpp2D.

There are various example scenes set in the project to cover different kind of scenarios. Here are some examples of these.

<summary>Spiral - two streams of agents move around obstacles as these are alligned in a spiral pattern.</summary>
<br>
<p align="center">
  <img src="Gifs/Spiral.gif" alt="Spiral" width="100%">
</p>

<summary>RandomDynamicObstacles - generates dynamic obstacles, rebuilds navigation mesh and moves agents all at once.</summary>
<br>
<p align="center">
  <img src="Gifs/RandomDynamicObstacles.gif" alt="RandomDynamicObstacles" width="100%">
</p>

## More demos

<details>
  <summary>BlockedDestination - two flows of agents blocked by a large obstacle in a middle.</summary>
  <br>
  <p align="center">
    <img src="Gifs/BlockedDestination.gif" alt="BlockedDestination" width="100%">
  </p>
</details>

<details>
  <summary>BlockedDestinationWithBridge - similar to BlockedDestination but there is a bridge in a middle.</summary>
  <br>
  <p align="center">
    <img src="Gifs/BlockedDestinationWithBridge.gif" alt="BlockedDestinationWithBridge" width="100%">
  </p>
</details>

<details>
  <summary>CentralPassage - agents distributed randomly in a circle tries to move to the corresponding position in the opposite side of the circle.</summary>
  <br>
  <p align="center">
    <img src="Gifs/CentralPassage.gif" alt="CentralPassage" width="100%">
  </p>
</details>

<details>
  <summary>Circles - 3 layers of navmesh where each ring blocks navmesh areas completely.</summary>
  <br>
  <p align="center">
    <img src="Gifs/Circles.gif" alt="Circles" width="100%">
  </p>
</details>

<details>
  <summary>RectangleSpawnersWithObstacles - two rectangular obstacles and agents moving around them.</summary>
  <br>
  <p align="center">
    <img src="Gifs/RectangleSpawnersWithObstacles.gif" alt="RectangleSpawnersWithObstacles" width="100%">
  </p>
</details>

<details>
  <summary>RandomObstacles - similar to RandomDynamicObstacles but obstacles are static through simulation lifetime.</summary>
  <br>
  <p align="center">
    <img src="Gifs/RandomObstacles.gif" alt="RandomObstacles" width="100%">
  </p>
</details>

<details>
  <summary>RandomRotatedObstacles - similar to RandomObstacles but obstacles randomly rotated.</summary>
  <br>
  <p align="center">
    <img src="Gifs/RandomRotatedObstacles.gif" alt="RandomRotatedObstacles" width="100%">
  </p>
</details>

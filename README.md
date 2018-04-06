# Fluid Simulation Technical Demo
A hybrid real-time GPU-based fluid simulator and renderer written in base C++ and Direct3D 11.

<p align="center">
  <img src="https://github.com/Nbickford/FluidSimulation/raw/master/Markdown/fluidsimHeader.gif">
</p>

Some notes about this program:
- Implements hybrid PIC/FLIP simulation using a value of α directly driven by the kinematic viscosity of water

- Runs on base Direct3D 11, so also useful as a relatively minimal reference for setting up such a visualization

- Does nearly all fluid simulation completely in parallel on the GPU using DirectCompute

- Simulates 900,000 particles at thirty frames a second, on at least one laptop.

[Try it out!](https://github.com/Nbickford/FluidSimulation/releases)

Running the Demo
----------------
You can find the latest release of the demo for Direct3D 11-compatible Windows-based x64 systems [here](https://github.com/Nbickford/FluidSimulation/releases). Extract /FluidSimulationDemo to any directory and run FluidSimulationDemo.exe to run the demo.

Make sure to select a discrete graphics card (on most machines, right-click FluidSimulationDemo.exe and select a suitable graphics processor under "Run with graphics processor >", or choose which graphics processor to run the demo with in your NVIDIA or ATI Control Panel).

Controls
--------
Left mouse button: Rotate camera.
Right mouse button: Zoom in/out.
+/-: Speed up/slow down time.
r: Reset simulation.

Neat things about this implementation
-------------------------------------
Our reference text was this simulation was Bridson's *Fluid Simulation for Computer Graphics*, 2nd ed. (2015), in addition to many other sources (see [the comments](https://github.com/Nbickford/FluidSimulation/blob/master/FluidSimulationDemo/Simulation3D.cpp#L260) for more details). We did a few neat things to make the fluid simulation in the book parallelize well:

**Projection**
- Because Bridson's preferred MIC(0) preconditioner is nontrivial to implement in a highly parallel way, we instead use successive over-relaxation using a checkerboard update pattern (see [Erik Arnebäck's article](https://erkaman.github.io/posts/gauss_seidel_graph_coloring.html) on using this same technique for more general topologies) to solve the linear system.
- We constructed a model for the optimal value of the SOR parameter ω to use by sampling the convergence rate of SOR for thousands of values of ω across different grid sizes ([Simulation3D.cpp#936](https://github.com/Nbickford/FluidSimulation/blob/master/FluidSimulationDemo/Simulation3D.cpp#L936)). With this value of ω, we get an asymptotic convergence rate of about 1/0.85≈1.17. 

**Rendering**
- We use exactly one triangle to render the entire final frame (reflections, refraction, sky, and all) by implementing a [Shadertoy-style](https://shadertoy.com) raytracer inside a pixel-shader for a full-screen triangle using both distance fields and traditional raytracing techniques.
- In particular, this means we don't have to implement Marching Cubes. (Inigo Quilez' [Rendering Worlds With Two Triangles](http://www.iquilezles.org/www/material/nvscene2008/nvscene2008.htm) is a pretty good source for showing how this is possible; we reduce "two" to "one" using Timothy Lottes' full-screen triangle trick from [FXAA](https://developer.download.nvidia.com/assets/gamedev/files/sdk/11/FXAA_WhitePaper.pdf))

**Particles and Grids**
- We use a fast sweeping method instead of a fast marching method to compute our level set and our closest-particle information when transferring particles to the grid; this is easier to parallelize (we avoid another prefix sum) and even runs more quickly in some cases. In practice, most of our time seems to actually be spent binning particles and computing Phi for cells containing particles).

**Techniques**
- We checked the GPU version of the 3D code against the CPU version of the code to make sure the results were identical (which happened to uncover a number of subtle bugs and differences along the way - turns out interpolation weights on most GPUs are actually computed using lower-precision fixed-point arithmetic!)

Technical Details
-----------------
**Advection:** RK3.

**Particle Velocity Transfer:** Trilinear hat kernel using fast sweeping.

**Projection:** SOR with ghost fluids.

**Rendering:** 2-bounce water reflection and refraction, plus four bounces of internal glass refraction, using [iq's smoothstep trick](https://www.shadertoy.com/view/XsfGDn) to mildly smooth normals at the expense of blockiness.

<div style="text-align:center"><img src ="https://github.com/Nbickford/FluidSimulation/raw/master/Markdown/fluidsimLinebreak.gif" /></div>

Things that Were Not Implemented
--------------------------------
- I intentionally kept this fluid simulator extremely simple to minimize implementation complexity; as such, this fluid simulator only works for cuboidal domains with no fluid-solid coupling.

- I had problems getting Improved Blobbies to work without also computing and tracking particle radii (in particular, I had problems avoiding significant numbers of visible creases in the level surface with constant particle radii); in the future, a reconstruction method such as Yu and Turk's anisotropic kernel method might be worth considering.

- Caustics were originally [in the plan for rendering](https://twitter.com/neilbickford/status/912404685465649152); unfortunately, these were not implemented due to time constraints, although Guardado and Sánchez-Crespo's method from GPU Gems 1 might work as a way to emulate caustics.

Check out the source code!
--------------------------
In addition to the GPU-based high-performance fluid simulator, the repo also has identical 2D and 3D CPU-based versions and (particle) renderers contained in {Simulation2D.cpp, FluidSimDemoOld.cpp} and {Simulation3D.cpp, FluidSimDemoOld3D.cpp}, respectively, which were used to step up to the GPU-based version; these versions are shorter, and are also well-commented.

Simulation2D.cpp: 845 lines of code, 425 semicolons

Simulation3D.cpp: 1162 lines of code, 635 semicolons

Simulation.cpp: 919 lines of code (and 53 kB of HLSL code across 32 files!)

Contributing
------------
Pull requests are welcome; if you have any problems running the executable or working with the code, please email me or open an issue!

Special thanks to Rishi for lending me a laptop to develop this project on when my old laptop broke during Finals.
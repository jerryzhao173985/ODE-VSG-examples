# ODE-VSG-examples

Examples for a robotic framework based on the Open Dynamics Engine (ODE) and the VulkanSceneGraph (VSG).

## Building

This project uses **CMake**. ODE is required for all builds while VSG is optional.
When `USE_VSG=ON` (default) the example will open a graphical window. When disabled
only the physics simulation is run in a headless mode.

```bash
# install dependencies (Ubuntu)
sudo apt-get install build-essential cmake libode-dev
# VSG is optional and can be built from source
```

```bash
mkdir build && cd build
cmake .. -DUSE_VSG=OFF  # set ON if VSG is available
make
ctest
```

The headless test runs a short simulation and verifies that the robot body
remains above the ground.  A second test checks that the rendering
transforms produced from the physics state are properly synchronised.

To run the graphical demo (with VSG installed):

```bash
./wheeled_simulation
```

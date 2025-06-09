#include "../include/ODEPhysicsModule.h"
#include <cassert>
#include <iostream>

int main() {
    ODEPhysicsModule physics;
    physics.createRobot();
    for (int i = 0; i < 10; ++i) physics.step();
    const dReal* pos = physics.getBodyPosition();
    assert(pos[2] > 0.5); // body should stay above ground
    std::cout << "z=" << pos[2] << std::endl;
    return 0;
}

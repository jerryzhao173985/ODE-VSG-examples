#include "../include/ODEPhysicsModule.h"
#include <cassert>
#include <cmath>

struct DummyTransform {
    double matrix[16];
};

struct DummyRenderingModule {
    DummyTransform bodyTransform;
    DummyTransform wheelTransforms[4];

    void update(const ODEPhysicsModule& physics) {
        // chassis
        const dReal* p = physics.getBodyPosition();
        const dReal* R = physics.getBodyRotation();
        setMatrix(bodyTransform.matrix, p, R);

        // wheels
        for (int i = 0; i < 4; ++i) {
            p = physics.getWheelPosition(i);
            R = physics.getWheelRotation(i);
            setMatrix(wheelTransforms[i].matrix, p, R);
        }
    }

    static void setMatrix(double m[16], const dReal* p, const dReal* R) {
        m[0] = R[0];  m[1] = R[1];  m[2] = R[2];  m[3] = 0.0;
        m[4] = R[4];  m[5] = R[5];  m[6] = R[6];  m[7] = 0.0;
        m[8] = R[8];  m[9] = R[9];  m[10]= R[10]; m[11]= 0.0;
        m[12]= p[0]; m[13]= p[1]; m[14]= p[2]; m[15]= 1.0;
    }
};

int main() {
    ODEPhysicsModule physics;
    physics.createRobot();
    for (int i = 0; i < 5; ++i) physics.step();

    DummyRenderingModule renderer;
    renderer.update(physics);

    const dReal* p = physics.getBodyPosition();
    const dReal* R = physics.getBodyRotation();

    // Verify translation components
    assert(std::abs(renderer.bodyTransform.matrix[12] - p[0]) < 1e-6);
    assert(std::abs(renderer.bodyTransform.matrix[13] - p[1]) < 1e-6);
    assert(std::abs(renderer.bodyTransform.matrix[14] - p[2]) < 1e-6);

    // Verify a few rotation components
    assert(std::abs(renderer.bodyTransform.matrix[0] - R[0]) < 1e-6);
    assert(std::abs(renderer.bodyTransform.matrix[5] - R[5]) < 1e-6);
    assert(std::abs(renderer.bodyTransform.matrix[10] - R[10]) < 1e-6);

    // Wheel[0] translation should also match
    p = physics.getWheelPosition(0);
    assert(std::abs(renderer.wheelTransforms[0].matrix[12] - p[0]) < 1e-6);
    assert(std::abs(renderer.wheelTransforms[0].matrix[13] - p[1]) < 1e-6);
    assert(std::abs(renderer.wheelTransforms[0].matrix[14] - p[2]) < 1e-6);

    return 0;
}

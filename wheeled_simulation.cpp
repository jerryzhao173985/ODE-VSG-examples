#include <ode/ode.h>
#include <vsg/all.h>
#include <vsg/viewer/Trackball.h>

//////////////////////////////////////////////////////////////////
/// Physics module: wraps all ODE setup, stepping, and cleanup ///
//////////////////////////////////////////////////////////////////
class ODEPhysicsModule {
public:
    ODEPhysicsModule()
    {
        dInitODE();
        world        = dWorldCreate();
        space        = dHashSpaceCreate(0);
        contactgroup = dJointGroupCreate(0);
        dWorldSetGravity(world, 0, 0, -9.81);

        // Ground plane at z=0
        ground = dCreatePlane(space, 0, 0, 1, 0);
    }

    ~ODEPhysicsModule()
    {
        dJointGroupDestroy(contactgroup);
        dSpaceDestroy(space);
        dWorldDestroy(world);
        dCloseODE();
    }

    /// Build the robot: body + 4 wheels + hinge‐motors + geoms
    void createRobot()
    {
        // 1) chassis body
        body = dBodyCreate(world);
        dMass m;
        dMassSetBoxTotal(&m, 1.0, 1.0, 1.0, 1.0);
        dBodySetMass(body, &m);
        dBodySetPosition(body, 0, 0, 1.0);

        // geom for collisions
        bodyGeom = dCreateBox(space, 1.0, 1.0, 1.0);
        dGeomSetBody(bodyGeom, body);

        // 2) wheels
        for (int i = 0; i < 4; ++i) {
            // physics body + mass
            wheels[i] = dBodyCreate(world);
            dMass mw;
            dMassSetSphereTotal(&mw, 0.1, 0.2);
            dBodySetMass(wheels[i], &mw);

            // position them at the four corners
            double x = (i % 2 == 0) ?  0.5 : -0.5;
            double y = (i <  2) ?  0.5 : -0.5;
            dBodySetPosition(wheels[i], x, y, 0.5);

            // collision geom
            wheelGeoms[i] = dCreateSphere(space, 0.2);
            dGeomSetBody(wheelGeoms[i], wheels[i]);

            // hinge joint: **axis = Y** for rolling forward/backward  ← FIX
            joints[i] = dJointCreateHinge(world, 0);
            dJointAttach(joints[i], body, wheels[i]);
            dJointSetHingeAnchor(joints[i], x, y, 0.5);
            dJointSetHingeAxis(joints[i], 0, 1, 0);   // ← FIX: horizontal axis

            // give it a motor: target vel = 1 rad/s, max torque = 10 Nm
            dJointSetHingeParam(joints[i], dParamVel, 1.0);
            dJointSetHingeParam(joints[i], dParamFMax, 10.0);
        }
    }

    /// Advance the physics by one fixed time step (10 ms)
    void step()
    {
        // collide
        dSpaceCollide(space, this, &ODEPhysicsModule::nearCallback);
        // integrate
        dWorldStep(world, 0.01);
        // remove all temporary contact joints
        dJointGroupEmpty(contactgroup);
    }

    // Getters for rendering sync:
    const dReal* getBodyPosition()   const { return dBodyGetPosition(body);   }
    const dReal* getBodyRotation()   const { return dBodyGetRotation(body);   }
    const dReal* getWheelPosition(int i) const { return dBodyGetPosition(wheels[i]); }
    const dReal* getWheelRotation(int i) const { return dBodyGetRotation(wheels[i]); }

private:
    /// ODE collision callback: create contact joints with friction/ERP/CFM
    static void nearCallback(void* data, dGeomID o1, dGeomID o2)
    {
        ODEPhysicsModule* self = static_cast<ODEPhysicsModule*>(data);
        dBodyID b1 = dGeomGetBody(o1);
        dBodyID b2 = dGeomGetBody(o2);
        if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact))
            return;

        const int MAXC = 10;
        dContact contact[MAXC];
        int n = dCollide(o1, o2, MAXC, &contact[0].geom, sizeof(dContact));
        if (n > 0) {
            for (int i = 0; i < n; ++i) {
                contact[i].surface.mode     = dContactSlip1     |
                                               dContactSlip2     |
                                               dContactSoftERP   |
                                               dContactSoftCFM   |
                                               dContactApprox1;
                contact[i].surface.mu       = 0.5;
                contact[i].surface.slip1    = 0.02;
                contact[i].surface.slip2    = 0.02;
                contact[i].surface.soft_erp = 0.1;
                contact[i].surface.soft_cfm = 0.01;

                // create the contact joint
                dJointID c = dJointCreateContact(self->world,
                                                  self->contactgroup,
                                                  &contact[i]);
                dJointAttach(c, b1, b2);
            }
        }
    }

    dWorldID       world;
    dSpaceID       space;
    dJointGroupID  contactgroup;
    dGeomID        ground;

    // chassis
    dBodyID        body;
    dGeomID        bodyGeom;

    // wheels
    dBodyID        wheels[4];
    dGeomID        wheelGeoms[4];
    dJointID       joints[4];
};

/////////////////////////////////////////////////////////////////////
/// Rendering module: wraps VSG, scene graph, and camera control ///
/////////////////////////////////////////////////////////////////////
class VSGRenderingModule {
public:
    vsg::ref_ptr<vsg::Viewer>   viewer;
    vsg::ref_ptr<vsg::Group>    scene;
    vsg::ref_ptr<vsg::MatrixTransform> bodyTransform;
    vsg::ref_ptr<vsg::Geode>    bodyGeode;
    vsg::ref_ptr<vsg::MatrixTransform> wheelTransforms[4];
    vsg::ref_ptr<vsg::Geode>    wheelGeodes[4];

    VSGRenderingModule()
    {
        // 1) viewer + window
        viewer = vsg::Viewer::create();
        auto windowTraits = vsg::WindowTraits::create();
        windowTraits->windowTitle = "小轮车机器人仿真系统";
        auto window = vsg::Window::create(windowTraits);
        viewer->addWindow(window);

        // 2) basic 3D camera control
        auto trackball = vsg::Trackball::create(window->getCamera());
        viewer->addEventHandler(trackball);
    }

    /// Build the VSG scene graph for body + wheels
    void createRobot()
    {
        // chassis: Box → Drawable → Geode → Transform
        auto bodyShape    = vsg::Box::create({0,0,0}, 1.0f,1.0f,1.0f);
        auto bodyDrawable = vsg::ShapeDrawable::create(bodyShape);
        bodyGeode         = vsg::Geode::create();
        bodyGeode->addDrawable(bodyDrawable);
        bodyTransform    = vsg::MatrixTransform::create();
        bodyTransform->addChild(bodyGeode);

        // wheels: Sphere → Drawable → Geode → Transform
        for (int i = 0; i < 4; ++i) {
            auto wheelShape    = vsg::Sphere::create({0,0,0}, 0.2f);
            auto wheelDrawable = vsg::ShapeDrawable::create(wheelShape);
            wheelGeodes[i]     = vsg::Geode::create();
            wheelGeodes[i]->addDrawable(wheelDrawable);
            wheelTransforms[i] = vsg::MatrixTransform::create();
            wheelTransforms[i]->addChild(wheelGeodes[i]);
        }

        // assemble into scene root
        scene = vsg::Group::create();
        scene->addChild(bodyTransform);
        for (int i = 0; i < 4; ++i) {
            scene->addChild(wheelTransforms[i]);
        }
        viewer->getWindows()[0]->getOrCreateView()->scene = scene;
    }

    /// Every frame: pull data from physics and write into Transform matrices
    void update(const ODEPhysicsModule& physics)
    {
        // chassis
        {
            const dReal* p = physics.getBodyPosition();
            const dReal* R = physics.getBodyRotation();
            vsg::mat4 M;
            M.set(
                vsg::vec4(R[0], R[1], R[2], 0.0f),
                vsg::vec4(R[4], R[5], R[6], 0.0f),
                vsg::vec4(R[8], R[9], R[10],0.0f),
                vsg::vec4(p[0], p[1], p[2],1.0f)
            );
            bodyTransform->matrix = M;
        }
        // wheels
        for (int i = 0; i < 4; ++i) {
            const dReal* p = physics.getWheelPosition(i);
            const dReal* R = physics.getWheelRotation(i);
            vsg::mat4 M;
            M.set(
                vsg::vec4(R[0], R[1], R[2], 0.0f),
                vsg::vec4(R[4], R[5], R[6], 0.0f),
                vsg::vec4(R[8], R[9], R[10],0.0f),
                vsg::vec4(p[0], p[1], p[2],1.0f)
            );
            wheelTransforms[i]->matrix = M;
        }
    }
};

//////////////////////////////////////////////////////////////
/// main: glue physics + rendering into a single loop     ///
//////////////////////////////////////////////////////////////
int main(int /*argc*/, char** /*argv*/)
{
    ODEPhysicsModule   physics;
    VSGRenderingModule renderer;

    physics.createRobot();
    renderer.createRobot();

    // single loop drives both physics and graphics
    while (renderer.viewer->running()) {
        physics.step();
        renderer.update(physics);
        renderer.viewer->handleEvents();
        renderer.viewer->update();
        renderer.viewer->recordAndSubmit();
        renderer.viewer->present();
    }

    return 0;
}

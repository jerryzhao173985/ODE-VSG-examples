#include "ODEPhysicsModule.h"
#include <iostream>

#ifdef USE_VSG
#include <vsg/all.h>
#include <vsg/viewer/Trackball.h>
#endif

/////////////////////////////////////////////////////////////////////
/// Rendering module: wraps VSG, scene graph, and camera control ///
/////////////////////////////////////////////////////////////////////
#ifdef USE_VSG
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
        auto window = viewer->getWindows()[0];
        window->getOrCreateView()->scene = scene;

        // Create command graph and compile resources
        auto camera = window->getOrCreateCamera();
        auto commandGraph = vsg::createCommandGraphForView(window, camera, scene);
        viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});
        viewer->compile();
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
#endif // USE_VSG

//////////////////////////////////////////////////////////////
/// main: glue physics + rendering into a single loop     ///
//////////////////////////////////////////////////////////////
#ifdef USE_VSG
int main(int /*argc*/, char** /*argv*/)
{
    ODEPhysicsModule   physics;
    VSGRenderingModule renderer;

    physics.createRobot();
    renderer.createRobot();

    // single loop drives both physics and graphics
    while (renderer.viewer->advanceToNextFrame()) {
        physics.step();
        renderer.update(physics);
        renderer.viewer->handleEvents();
        renderer.viewer->update();
        renderer.viewer->recordAndSubmit();
        renderer.viewer->present();
    }

    return 0;
}
#else
int main()
{
    ODEPhysicsModule physics;
    physics.createRobot();
    for (int i = 0; i < 100; ++i) {
        physics.step();
    }
    const dReal* p = physics.getBodyPosition();
    std::cout << "Final position: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
    return 0;
}
#endif

#include "Assignment6.h"

#include "WorldList.h" //This is where we place all of our WOs
#include "ManagerOpenGLState.h" //We can change OpenGL State attributes with this
#include "Axes.h" //We can set Axes to on/off with this
#include "PhysicsEngineODE.h"

//Different WO used by this module
#include "WO.h"
#include "WOStatic.h"
#include "WONVPhysX.h"
#include "WOStaticPlane.h"
#include "WOStaticTrimesh.h"
#include "WOTrimesh.h"
#include "WOHumanCyborg.h"
#include "WOHumanCal3DPaladin.h"
#include "WOWayPointSpherical.h"
#include "WOLight.h"
#include "WOSkyBox.h"
#include "WOCar1970sBeater.h"
#include "Camera.h"
#include "CameraStandard.h"
#include "CameraChaseActorSmooth.h"
#include "CameraChaseActorAbsNormal.h"
#include "CameraChaseActorRelNormal.h"
#include "Model.h"
#include "ModelDataShared.h"
#include "ModelMesh.h"
#include "ModelMeshDataShared.h"
#include "ModelMeshSkin.h"
#include "WONVStaticPlane.h"
#include "WONVPhysX.h"
#include "WONVDynSphere.h"
#include "WOImGui.h" //GUI Demos also need to #include "AftrImGuiIncludes.h"
#include "AftrImGuiIncludes.h"
#include "AftrGLRendererBase.h"
#include "Mat4.h"
#include "WOCar1970sBeater.h"
#include "WONVPhysX.h"
#include <WONVPhysX.h>
#include <irrKlang.h>
#include <iostream>
//#include "imgui.h"

#include "NetMsgBlock.h"

#include <iostream>
#include <cstring>
#include "winsock2.h"     // Winsock header for Windows
#include "ws2tcpip.h"     // Additional header for IP address functions

#pragma comment(lib, "Ws2_32.lib")  // Link against Winsock library

using namespace Aftr;

GLViewNewModule* GLViewNewModule::New(const std::vector<std::string>& args)
{
    GLViewNewModule* glv = new GLViewNewModule();
    glv->init(Aftr::GRAVITY, Vector(0, 0, -1.0f), "aftr.conf", PHYSICS_ENGINE_TYPE::petODE);
    glv->onCreate();
    return glv;
}

GLViewNewModule::GLViewNewModule() : GLView(args)
{
}

bool GLViewNewModule::hasReachedEndpoint()
{
    // Basic collision detection: check if the distance between the player box and end point is within a threshold
    return (playerBox->getPosition() - endPos).magnitude() < 0.5;
}

void GLViewNewModule::onCreate()
{
    // Create a new world object list and initialize the physics engine
    std::string shinyRedPlasticCube(ManagerEnvironmentConfiguration::getSMM() + "/models/cube4x4x4redShinyPlastic_pp.wrl");
    std::string sphereModel(ManagerEnvironmentConfiguration::getSMM() + "/models/sphereR5Earth.wrl");

    startPoint = WO::New(shinyRedPlasticCube, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
    startPoint->setPosition(startPos);
    startPoint->setLabel("Start Point");
    worldLst->push_back(startPoint);

    endPoint = WO::New(shinyRedPlasticCube, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
    endPoint->setPosition(endPos);
    endPoint->setLabel("End Point");
    worldLst->push_back(endPoint);

    playerBox = WO::New(shinyRedPlasticCube, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
    playerBox->setPosition(startPos);
    playerBox->setLabel("Player Box");
    worldLst->push_back(playerBox);

    sphere = WO::New(sphereModel, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
    sphere->setPosition(Vector(5, 5, 5));  // Position the sphere
    sphere->setLabel("Sphere");
    worldLst->push_back(sphere);

    // Add collision geometry to objects
    addCollisionGeometry(playerBox, PE_COLLISION_TYPE::CT_BOX);
    addCollisionGeometry(sphere, PE_COLLISION_TYPE::CT_SPHERE);

    // Add objects to the collision list
    worldObjects.push_back(playerBox);
    worldObjects.push_back(sphere);

    createNewModuleWayPoints();
}


GLViewNewModule::~GLViewNewModule()
{
}

void GLViewNewModule::updateWorld()
{
    GLView::updateWorld();

    // Apply gravity (downward force)
    Vector gravityForce(0, 0, -9.8f);
    applyForce(playerBox, gravityForce);

    // Apply torque to the player box (spin)
    Vector torque(0.1f, 0.1f, 0);  // Small torque for spinning
    applyTorque(playerBox, torque);

    // Check collisions
    checkCollisions(worldObjects, [](WO* obj1, WO* obj2) {
        std::cout << obj1->getLabel() << " collided with " << obj2->getLabel() << std::endl;
        });
}

void GLViewNewModule::onCollisionDetected(WO* obj1, WO* obj2)
{
    // Simple collision response
    std::cout << obj1->getLabel() << " collided with " << obj2->getLabel() << std::endl;
}

void GLViewNewModule::onKeyDown(const SDL_KeyboardEvent& key)
{
    Vector force;

    switch (key.keysym.sym)
    {
    case SDLK_w:    // Move forward
        force = Vector(10.0f, 0, 0);
        applyForce(playerBox, force);
        break;
    case SDLK_s:    // Move backward
        force = Vector(-10.0f, 0, 0);
        applyForce(playerBox, force);
        break;
    case SDLK_a:    // Move left
        force = Vector(0, -10.0f, 0);
        applyForce(playerBox, force);
        break;
    case SDLK_d:    // Move right
        force = Vector(0, 10.0f, 0);
        applyForce(playerBox, force);
        break;
    case SDLK_SPACE:  // Jump
        force = Vector(0, 0, 20.0f);
        applyForce(playerBox, force);
        break;
    default:
        break;
    }
}
void GLViewNewModule::applyForce(WO* wo, const Vector& force)
{
    std::cout << "Applying force to object: " << wo->getLabel() << std::endl;
    Vector currentPosition = wo->getPosition();
    Vector newPosition = currentPosition + (force * 0.1f);  // Adjust the force magnitude
    wo->setPosition(newPosition);  // Update the position based on the applied force
}

void GLViewNewModule::applyTorque(WO* wo, const Vector& torque)
{
    std::cout << "Applying torque to object: " << wo->getLabel() << std::endl;
    Vector normalizedTorque = torque.normalizeMe();
    Mat4 rotationMatrix = Mat4::rotateIdentityMat(normalizedTorque, 0.1f);  // Create a small rotation matrix based on torque
    Mat4 currentTransform = wo->getModel()->getDisplayMatrix();
    Mat4 newTransform = rotationMatrix * currentTransform;  // Apply rotation to the object's current transformation
    wo->getModel()->setDisplayMatrix(newTransform);  // Set the new transformation
}

void GLViewNewModule::addCollisionGeometry(WO* wo, PE_COLLISION_TYPE collisionType)
{
    switch (collisionType)
    {
    case PE_COLLISION_TYPE::CT_BOX:
        std::cout << "Adding box collision geometry for object: " << wo->getLabel() << std::endl;
        // Define and apply a box bounding volume here
        break;
    case PE_COLLISION_TYPE::CT_SPHERE:
        std::cout << "Adding sphere collision geometry for object: " << wo->getLabel() << std::endl;
        // Define and apply a sphere bounding volume here
        break;
    case PE_COLLISION_TYPE::CT_PLANE:
        std::cout << "Adding plane collision geometry for object: " << wo->getLabel() << std::endl;
        // Define and apply a plane bounding volume here
        break;
    default:
        std::cerr << "Unknown collision type!" << std::endl;
    }
}

void GLViewNewModule::checkCollisions(std::vector<WO*>& worldObjects, void (*collisionCallback)(WO* obj1, WO* obj2))
{
    std::cout << "Checking collisions..." << std::endl;

    // Iterate through the list of world objects and check for collisions
    for (size_t i = 0; i < worldObjects.size(); ++i)
    {
        for (size_t j = i + 1; j < worldObjects.size(); ++j)
        {
            WO* obj1 = worldObjects[i];
            WO* obj2 = worldObjects[j];

            // Calculate the distance between the two objects
            Vector pos1 = obj1->getPosition();
            Vector pos2 = obj2->getPosition();
            float distance = (pos1 - pos2).magnitude();

            // If the distance is smaller than a threshold (e.g., 1.0f), we assume a collision has occurred
            if (distance < 1.0f)
            {
                // Trigger the collision callback function
                collisionCallback(obj1, obj2);
            }
        }
    }
}

void GLViewNewModule::createNewModuleWayPoints()
{
    WayPointParametersBase params(this);
    params.frequency = 5000;
    params.useCamera = true;
    params.visible = true;
    WOWayPointSpherical* wayPt = WOWayPointSpherical::New(params, 3);
    wayPt->setPosition(Vector(50, 0, 3));
    worldLst->push_back(wayPt);
}


void GLViewNewModule::onResizeWindow(GLsizei width, GLsizei height)
{
    GLView::onResizeWindow(width, height);
}

void GLViewNewModule::onMouseDown(const SDL_MouseButtonEvent& e)
{
    GLView::onMouseDown(e);
}

void GLViewNewModule::onMouseUp(const SDL_MouseButtonEvent& e)
{
    GLView::onMouseUp(e);
}

void GLViewNewModule::onMouseMove(const SDL_MouseMotionEvent& e)
{
    GLView::onMouseMove(e);
}

void GLViewNewModule::onKeyUp(const SDL_KeyboardEvent& key)
{
    GLView::onKeyUp(key);
}

void GLViewNewModule::loadMap()
{
    this->worldLst = new WorldList();
    this->actorLst = new WorldList();
    this->netLst = new WorldList();

    ManagerOpenGLState::GL_CLIPPING_PLANE = 1000.0;
    ManagerOpenGLState::GL_NEAR_PLANE = 0.1f;
    ManagerOpenGLState::enableFrustumCulling = false;
    Axes::isVisible = true;
    this->glRenderer->isUsingShadowMapping(false);
    this->cam->setPosition(15, 15, 10);

    std::string shinyRedPlasticCube(ManagerEnvironmentConfiguration::getSMM() + "/models/cube4x4x4redShinyPlastic_pp.wrl");
    std::string grass(ManagerEnvironmentConfiguration::getSMM() + "/models/grassFloor400x400_pp.wrl");

    std::vector<std::string> skyBoxImageNames;
    skyBoxImageNames.push_back(ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_mountains+6.jpg");

    // Add skybox and light
    WO* wo = WOSkyBox::New(skyBoxImageNames.at(0), this->getCameraPtrPtr());
    wo->setPosition(Vector(0, 0, 0));
    worldLst->push_back(wo);

    WOLight* light = WOLight::New();
    light->isDirectionalLight(true);
    light->setPosition(Vector(0, 0, 100));
    worldLst->push_back(light);

    // Add ground plane
    WO* ground = WO::New(grass, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
    ground->setPosition(Vector(0, 0, 0));
    worldLst->push_back(ground);

    createNewModuleWayPoints();
}


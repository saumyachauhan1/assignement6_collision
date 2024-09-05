#pragma once

#include "GLView.h"
#include <irrKlang.h>
#include <string>
#include <vector>
#include "Vector.h"
#include "Mat4.h"

using namespace irrklang;

namespace Aftr
{
    class Camera;

    enum class PE_COLLISION_TYPE { CT_BOX, CT_SPHERE, CT_PLANE };

    class GLViewNewModule : public GLView
    {
    public:
        static GLViewNewModule* New(const std::vector<std::string>& outArgs);
        virtual ~GLViewNewModule();
        virtual void updateWorld(); ///< Called once per frame
        virtual void loadMap(); ///< Called once at startup to build this module's scene
        virtual void createNewModuleWayPoints();
        virtual void onResizeWindow(GLsizei width, GLsizei height);
        virtual void onMouseDown(const SDL_MouseButtonEvent& e);
        virtual void onMouseUp(const SDL_MouseButtonEvent& e);
        virtual void onMouseMove(const SDL_MouseMotionEvent& e);
        virtual void onKeyDown(const SDL_KeyboardEvent& key);
        virtual void onKeyUp(const SDL_KeyboardEvent& key);
        virtual void onCollisionDetected(WO* obj1, WO* obj2);  ///< New method for handling collisions
        virtual bool hasReachedEndpoint();

        // New methods for physics simulation
        void applyForce(WO* wo, const Vector& force);
        void applyTorque(WO* wo, const Vector& torque);
        void addCollisionGeometry(WO* wo, PE_COLLISION_TYPE collisionType);
        void checkCollisions(std::vector<WO*>& worldObjects, void (*collisionCallback)(WO* obj1, WO* obj2));
    protected:
        GLViewNewModule();
        virtual void onCreate();

        WO* startPoint;  // Point A
        WO* endPoint;    // Point B
        WO* playerBox;   // The box the player controls
        WO* sphere;      // A sphere added to the scene

        Vector startPos; // Starting position of the player box
        Vector endPos;   // End position (point B)
        std::vector<WO*> worldObjects;  // List of world objects for collision detection
    };

} // namespace Aftr

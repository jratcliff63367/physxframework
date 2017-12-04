#ifndef NV_PHYSX_FRAMEWORK_H
#define NV_PHYSX_FRAMEWORK_H

// The PhysX framework is a DLL which provides a very simple interface to perform
// physics simulations using the PhysX SDK.  It also contains bindings to do debug
// visualizations of the current simulation using the NvRenderDebug remote visualization
// API and/or PVD (the PhysX Visual Debugger)
// This is not intended to be a full function physics API but rather a minimal simple
// system for getting basic demos to work.  The intial version was written specifically 
// to improve and test the V-HACD (voxelized hierarchical convex decomposition library) results.
// @see: https://github.com/kmammou/v-hacd

#include <stdint.h>

namespace RENDER_DEBUG
{
	class RenderDebug;
}

namespace PHYSICS_DOM
{
	class PhysicsDOM;
	class NodeState;
}

namespace NV_PHYSX_FRAMEWORK
{

#define PHYSX_FRAMEWORK_VERSION_NUMBER 110

class PhysicsDOMContainer
{
public:
	virtual const PHYSICS_DOM::PhysicsDOM *getPhysicsDOM(void) = 0;
	virtual void release(void) = 0;
};

// Instantiate the PhysX SDK, create a scene, and a ground plane
class PhysXFramework
{
public:

	class CommandCallback
	{
	public:
		/**
		*\brief Optional callback to the application to process an arbitrary console command.

		This allows the application to process an incoming command from the server.  If the application consumes the command, then it will not be passed on
		to the rest of the default processing.  Return true to indicate that you consumed the command, false if you did not.

		\return Return true if your application consumed the command, return false if it did not.
		*/
		virtual bool processDebugCommand(uint32_t argc, const char **argv) = 0;
	};

	// Returns delta time since last simulation step
	virtual float simulate(bool showPhysics) = 0;

	virtual void setCommandCallback(CommandCallback *cc) = 0;

	// Return the render debug interface if available
	virtual RENDER_DEBUG::RenderDebug *getRenderDebug(void) = 0;

	virtual void setDragForce(float force) = 0;

	virtual void setPauseState(bool state) = 0;
	virtual bool getPauseState(void) const = 0;

	// serialize the current state to an XML file
	virtual void serializeXML(const char *fname) = 0;

	// Debug feature; create some default box stacks
	virtual void createDefaultStacks(void) = 0;

	// Create some of everything so we can serialize the scenes and get a detailed
	// XML output for analysis
	virtual void createSomeOfEverything(void) = 0;

	// Load this physics DOM
	virtual bool loadPhysicsDOM(const PHYSICS_DOM::PhysicsDOM &physicsDOM) = 0;

	virtual void releasePhysicsDOM(void) = 0;

	// Parses a PhysX RepX XML file and loads the contents into a PhysicsDOM container
	virtual PhysicsDOMContainer *importPhysXDOM(const char *fname) = 0;

	// Does a named lookup of a node with this ID and, if found, returns the 'NodeState'
	// interface which can be used to query current state of this object
	virtual PHYSICS_DOM::NodeState *getNodeState(const char *nodeId) = 0;

	// Release the PhysXFramework interface
	virtual void release(void) = 0;

protected:
	virtual ~PhysXFramework(void)
	{
	}
};

PhysXFramework *createPhysXFramework(uint32_t versionNumber,const char *dllName);


};

#endif

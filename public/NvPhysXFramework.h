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

namespace NV_PHYSX_FRAMEWORK
{

// Instantiate the PhysX SDK, create a scene, and a ground plane
class PhysXFramework
{
public:
	virtual void release(void) = 0;

protected:
	virtual ~PhysXFramework(void)
	{
	}
};

PhysXFramework *createPhysXFramework(void);


};

#endif

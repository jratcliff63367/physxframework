#ifndef NV_PHYSX_FRAMEWORK_H
#define NV_PHYSX_FRAMEWORK_H

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

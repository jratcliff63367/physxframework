#ifndef PHYSICS_DOM_PHYSX_H
#define PHYSICS_DOM_PHYSX_H

namespace PHYSICS_DOM
{
	class PhysicsDOM;
}

namespace physx
{
	class PxPhysics;
	class PxCooking;
	class PxScene;
}

namespace PHYSICS_DOM_PHYSX
{

	class PhysicsDOMPhysX
	{
	public:
		static PhysicsDOMPhysX *create(physx::PxPhysics *p, physx::PxCooking *c);
		// Instantiate the contents of this PhysicsDOM into the PhysX SDK
		// If no PxScene is provided, then one will be created.
		// If a PxScene is provided, then objects will be instantiated into it.
		virtual bool loadPhysicsDOM(const PHYSICS_DOM::PhysicsDOM &p,
									physx::PxScene *scene) = 0;

		virtual void reset(void) = 0;
		virtual void release(void) = 0;
	protected:
		virtual ~PhysicsDOMPhysX(void)
		{

		}
	};

}

#endif

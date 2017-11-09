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
}

namespace PHYSICS_DOM_PHYSX
{

	class PhysicsDOMPhysX
	{
	public:
		static PhysicsDOMPhysX *create(physx::PxPhysics *p, physx::PxCooking *c);
		virtual bool loadPhysicsDOM(const PHYSICS_DOM::PhysicsDOM &p) = 0;
		virtual void reset(void) = 0;
		virtual void release(void) = 0;
	protected:
		virtual ~PhysicsDOMPhysX(void)
		{

		}
	};

}

#endif

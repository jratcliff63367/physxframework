#ifndef SOME_OF_EVERYTHING_H
#define SOME_OF_EVERYTHING_H

namespace physx
{
	class PxPhysics;
	class PxCooking;
	class PxScene;
}

namespace SOME_OF_EVERYTHING
{

class SomeOfEverything
{
public:
	static SomeOfEverything *create(physx::PxPhysics *physics, physx::PxCooking *cooking,physx::PxScene *scene);
	virtual void release(void) = 0;
protected:
	virtual ~SomeOfEverything(void)
	{

	}
};

}

#endif

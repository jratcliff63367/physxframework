#include "NvPhysXFramework.h"
#include <ctype.h>

#include "Nv.h"
#include "PxPhysicsAPI.h"
#include "NvRenderDebugTyped.h"

using namespace physx;

namespace NV_PHYSX_FRAMEWORK
{


	PxDefaultAllocator		gAllocator;
	PxDefaultErrorCallback	gErrorCallback;

	PxFoundation*			gFoundation = NULL;
	PxPhysics*				gPhysics = NULL;

	PxDefaultCpuDispatcher*	gDispatcher = NULL;
	PxScene*				gScene = NULL;

	PxMaterial*				gMaterial = NULL;

	PxPvd*                  gPvd = NULL;

	class PhysXFrameworkImpl : public PhysXFramework
	{
	public:
		PhysXFrameworkImpl(void)
		{

		}
		virtual ~PhysXFrameworkImpl(void)
		{

		}

		virtual void release(void)
		{
			delete this;
		}

	};

PhysXFramework *createPhysXFramework(uint32_t versionNumber, const char *dllName)
{
	if (versionNumber != PHYSX_FRAMEWORK_VERSION_NUMBER)
	{
		return nullptr;
	}
	NV_UNUSED(dllName);
	PhysXFrameworkImpl *ret = new PhysXFrameworkImpl;
	return static_cast<PhysXFramework *>(ret);
}

} // end of namespace
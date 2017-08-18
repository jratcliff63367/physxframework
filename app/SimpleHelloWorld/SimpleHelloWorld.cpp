#include <stdio.h>
#include "NvRenderDebug.h"
#include "Nv.h"
#include "NvPhysXFramework.h"

#ifdef NV_DEBUG
#define USE_DEBUG 1
#endif

int main(int argc,const char **argv)
{
	const char *dllName = nullptr;
#if NV_X64
#if USE_DEBUG
	dllName = "PhysXFramework64DEBUG.dll";
#else
	dllName = "PhysXFramework64.dll";
#endif
#else
#if USE_DEBUG
	dllName = "PhysXFramework32DEBUG.dll";
#else
	dllName = "PhysXFramework32.dll";
#endif
#endif

	NV_PHYSX_FRAMEWORK::PhysXFramework *pf = NV_PHYSX_FRAMEWORK::createPhysXFramework(PHYSX_FRAMEWORK_VERSION_NUMBER, dllName);
	if (pf)
	{
		pf->release();
	}

	NV_UNUSED(argc);
	NV_UNUSED(argv);
	return 0;
}

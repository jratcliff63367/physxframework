#include <stdio.h>
#include "NvRenderDebug.h"
#include "Nv.h"
#include "NvPhysXFramework.h"

#ifdef NV_DEBUG
#define USE_DEBUG 1
#endif

int main(int _argc,const char **_argv)
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

		RENDER_DEBUG::RenderDebug *renderDebug = pf->getRenderDebug();
		if (pf)
		{
			for (;;)
			{
				pf->simulate();
				float pos[3] = { 0,0,0 };
				renderDebug->debugSphere(pos, 1);
				renderDebug->render(1.0f / 60.0f, nullptr);

				uint32_t argc;
				const char **argv = renderDebug->getRemoteCommand(argc);
				if (argv)
				{
					const char *cmd = argv[0];
					if (strcmp(cmd, "client_stop") == 0)
					{
						break;
					}
				}
			}
		}


		pf->release();
	}

	NV_UNUSED(_argc);
	NV_UNUSED(_argv);
	return 0;
}

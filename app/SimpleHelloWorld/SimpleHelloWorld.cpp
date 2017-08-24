#include <stdio.h>
#include <string.h>
#include "NvRenderDebug.h"
#include "NvPhysXFramework.h"

#define USE_DEBUG 0

#define TEST_BOX 1
#define BOX_SIZE 20

class SimpleHelloWorld : public NV_PHYSX_FRAMEWORK::PhysXFramework::CommandCallback
{
public:
	SimpleHelloWorld(void)
	{
		const char *dllName = nullptr;
#if _M_X64
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
		mPhysXFramework = NV_PHYSX_FRAMEWORK::createPhysXFramework(PHYSX_FRAMEWORK_VERSION_NUMBER, dllName);
		if (mPhysXFramework)
		{
			mPhysXFramework->setCommandCallback(this);
#if TEST_BOX
			float boxSize[3];
			boxSize[0] = BOX_SIZE;
			boxSize[1] = BOX_SIZE;
			boxSize[2] = BOX_SIZE;
			float boxPosition[3];
			boxPosition[0] = 0;
			boxPosition[1] = BOX_SIZE*0.5f;
			boxPosition[2] = 0;
			mPhysXFramework->createBox(boxSize, boxPosition);
#else
			mPhysXFramework->createDefaultStacks();
#endif
		}
	}

	virtual ~SimpleHelloWorld(void)
	{
		if (mPhysXFramework)
		{
			mPhysXFramework->release();
		}
	}

	bool process(void)
	{
		if (mPhysXFramework)
		{
			mPhysXFramework->simulate(true);
		}
		return !mExit;
	}

	virtual bool processDebugCommand(uint32_t argc, const char **argv)
	{
		bool ret = false;

		if (argc)
		{
			const char *cmd = argv[0];
			if (strcmp(cmd, "client_stop") == 0)
			{
				mExit = true;
				ret = true;
			}
		}
		return ret;
	}

	bool								mExit{ false };
	NV_PHYSX_FRAMEWORK::PhysXFramework *mPhysXFramework{ nullptr };
};

int main(int _argc,const char **_argv)
{
	(_argv);
	(_argc);
	SimpleHelloWorld shw;
	while (shw.process());


	return 0;
}

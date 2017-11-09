#include <stdio.h>
#include <string.h>
#include "NvRenderDebug.h"
#include "NvPhysXFramework.h"
#include "PhysicsDOM.h"

#define USE_DEBUG 0

#define TEST_IMPORT_XML 0
#define TEST_PHYSICS_DOM 0
#define TEST_SOME_OF_EVERYTHING 0
#define TEST_BOX 1
#define BOX_SIZE 1

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
#if TEST_SOME_OF_EVERYTHING
			mPhysXFramework->createSomeOfEverything();
#elif TEST_BOX
			float boxSize[3];
			boxSize[0] = BOX_SIZE;
			boxSize[1] = BOX_SIZE;
			boxSize[2] = BOX_SIZE;
			float boxPosition[3];
			boxPosition[0] = 0;
			boxPosition[1] = BOX_SIZE*0.5f;
			boxPosition[2] = 0;
			mPhysXFramework->createBox(boxSize, boxPosition);
#elif TEST_PHYSICS_DOM
			testPhysicsDOM();
#elif TEST_IMPORT_XML
			testImportXML();
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
			static uint32_t gCount = 0;
			gCount++;
			if (gCount == 16 )
			{
				mPhysXFramework->serializeXML("SimpleHelloWorld.xml");
			}
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

	void testPhysicsDOM(void)
	{

		PHYSICS_DOM::PhysicsDOM dom;
		PHYSICS_DOM::Collection *c = new PHYSICS_DOM::Collection;
		c->id = "0";
		PHYSICS_DOM::PhysicsMaterial *pm = new PHYSICS_DOM::PhysicsMaterial;
		pm->id = "1";
		c->nodes.push_back(pm);
		PHYSICS_DOM::BoxGeometry *box = new PHYSICS_DOM::BoxGeometry;
		PHYSICS_DOM::GeometryInstance *box_instance = new PHYSICS_DOM::GeometryInstance;
		box_instance->geometry = box;
		box_instance->materials.push_back("1");
		PHYSICS_DOM::RigidDynamic *rd = new PHYSICS_DOM::RigidDynamic;
		rd->id = "2";
		rd->geometryInstances.push_back(box_instance);
		c->nodes.push_back(rd);
		PHYSICS_DOM::Scene *s = new PHYSICS_DOM::Scene;
		dom.collections.push_back(c);
		dom.scenes.push_back(s);

		PHYSICS_DOM::InstanceCollection *ic = new PHYSICS_DOM::InstanceCollection;
		ic->id = "3";			// Node '3'
		ic->collection = "0";	// Instance node '0' 
		s->nodes.push_back(ic);

		mPhysXFramework->loadPhysicsDOM(dom);
	}

	void testImportXML(void)
	{
		PHYSICS_DOM::PhysicsDOM dom;
		mPhysXFramework->importPhysXDOM("f:\\github\\InspectXML\\SimpleHelloWorld.xml", dom);
		mPhysXFramework->loadPhysicsDOM(dom);
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

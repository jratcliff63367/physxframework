#include <stdio.h>
#include <string.h>
#include "NvRenderDebug.h"
#include "NvPhysXFramework.h"
#include "PhysicsDOMDef.h"

#define USE_DEBUG 1

#define TEST_IMPORT_XML 1
#define TEST_PHYSICS_DOM 0
#define TEST_SOME_OF_EVERYTHING 0
#define TEST_BOX 0
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

		PHYSICS_DOM::PhysicsDOMDef dom;
		PHYSICS_DOM::CollectionDef *c = new PHYSICS_DOM::CollectionDef;
		c->mId = "0";
		PHYSICS_DOM::PhysicsMaterialDef *pm = new PHYSICS_DOM::PhysicsMaterialDef;
		pm->mId = "1";
		c->mNodes.push_back(pm);
		PHYSICS_DOM::BoxGeometryDef *box = new PHYSICS_DOM::BoxGeometryDef;
		PHYSICS_DOM::GeometryInstanceDef *box_instance = new PHYSICS_DOM::GeometryInstanceDef;
		box_instance->mGeometry = box;
		box_instance->mMaterials.push_back("1");
		PHYSICS_DOM::RigidDynamicDef *rd = new PHYSICS_DOM::RigidDynamicDef;
		rd->mId = "2";
		rd->mGeometryInstances.push_back(box_instance);
		c->mNodes.push_back(rd);
		PHYSICS_DOM::SceneDef *s = new PHYSICS_DOM::SceneDef;
		dom.mCollections.push_back(c);
		dom.mScenes.push_back(s);

		PHYSICS_DOM::InstanceCollectionDef *ic = new PHYSICS_DOM::InstanceCollectionDef;
		ic->mId = "3";			// Node '3'
		ic->mCollection = "0";	// Instance node '0' 
		s->mNodes.push_back(ic);


		dom.initDOM();
		PHYSICS_DOM::PhysicsDOM *pdom = dom.getPhysicsDOM();

		//
		if (pdom->scenesCount)
		{
			PHYSICS_DOM::Scene *ss = pdom->scenes[0];
			if (ss->nodesCount)
			{
				PHYSICS_DOM::Node *n = ss->nodes[0];
				if (n->type == PHYSICS_DOM::NT_INSTANCE_COLLECTION)
				{
					PHYSICS_DOM::InstanceCollection *icd = static_cast<PHYSICS_DOM::InstanceCollection *>(n);
					if (icd)
					{
						printf("%s", icd->collection);
					}
				}
			}
		}
		//

		mPhysXFramework->loadPhysicsDOM(*pdom);
	}

	void testImportXML(void)
	{
		NV_PHYSX_FRAMEWORK::PhysicsDOMContainer *pcontain = mPhysXFramework->importPhysXDOM("ConvexDecomposition1.xml");
		if (pcontain)
		{
			mPhysXFramework->loadPhysicsDOM(*pcontain->getPhysicsDOM());
			mPhysXFramework->serializeXML("f:\\github\\physxframework\\dom.xml");
			pcontain->release();
		}
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

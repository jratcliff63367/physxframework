#include "NvPhysXFramework.h"
#include "RenderDebugPhysX.h"
#include <ctype.h>

#include "Nv.h"
#include "PxPhysicsAPI.h"
#include "NvRenderDebugTyped.h"

using namespace physx;

#define HOST_NAME "localhost"
#define PVD_HOST "localhost"

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
			const char *dllName = NULL;
#ifdef _WIN64
			dllName = "NvRenderDebug_x64.dll";
#else
			dllName = "NvRenderDebug_x86.dll";
#endif
			printf("Loading RenderDebug DLL\r\n");
			RENDER_DEBUG::RenderDebug::Desc desc;
			desc.dllName = dllName;

			desc.hostName = HOST_NAME;

			desc.applicationName = "ConvexDecomposition";
			desc.runMode = RENDER_DEBUG::RenderDebug::RM_CLIENT;
			mRenderDebug = RENDER_DEBUG::createRenderDebug(desc);
			if (mRenderDebug)
			{
				mRenderDebugTyped = mRenderDebug->getRenderDebugTyped();
			}
			initPhysics(true);
			if (gScene && mRenderDebug )
			{
				mRenderDebugPhysX = createRenderDebugPhysX(gScene, mRenderDebug, true);
			}
		}

		virtual ~PhysXFrameworkImpl(void)
		{
			if (mRenderDebug)
			{
				mRenderDebug->release();
			}
			if (mRenderDebugPhysX)
			{
				mRenderDebugPhysX->release();
			}
			cleanupPhysics(true);
		}

		// Return the render debug interface if available
		virtual RENDER_DEBUG::RenderDebug *getRenderDebug(void) final
		{
			return mRenderDebug;
		}

		virtual void release(void) final
		{
			delete this;
		}

		PxReal stackZ = 10.0f;

		PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity = PxVec3(0))
		{
			PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
			dynamic->setAngularDamping(0.5f);
			dynamic->setLinearVelocity(velocity);
			gScene->addActor(*dynamic);
			return dynamic;
		}

		void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
		{
			PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);
			for (PxU32 i = 0; i < size; i++)
			{
				for (PxU32 j = 0; j < size - i; j++)
				{
					PxTransform localTm(PxVec3(PxReal(j * 2) - PxReal(size - i), PxReal(i * 2 + 1), 0) * halfExtent);
					PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
					body->attachShape(*shape);
					PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
					gScene->addActor(*body);
				}
			}
			shape->release();
		}

		void initPhysics(bool interactive)
		{
			gFoundation = PxCreateFoundation(PX_FOUNDATION_VERSION, gAllocator, gErrorCallback);

			gPvd = PxCreatePvd(*gFoundation);
			PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
			gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

			gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

			PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
			sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
			gDispatcher = PxDefaultCpuDispatcherCreate(2);
			sceneDesc.cpuDispatcher = gDispatcher;
			sceneDesc.filterShader = PxDefaultSimulationFilterShader;
			gScene = gPhysics->createScene(sceneDesc);

			PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
			if (pvdClient)
			{
				pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
				pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
				pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
			}
			gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

			PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
			gScene->addActor(*groundPlane);

			for (PxU32 i = 0; i < 5; i++)
				createStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f);

			if (!interactive)
				createDynamic(PxTransform(PxVec3(0, 40, 100)), PxSphereGeometry(10), PxVec3(0, -50, -100));
		}

		void stepPhysics(bool interactive)
		{
			PX_UNUSED(interactive);
			gScene->simulate(1.0f / 60.0f);
			gScene->fetchResults(true);
			if (mRenderDebugPhysX)
			{
//				mRenderDebugPhysX->render(1.0f / 60.0f, true, true, true, false);
			}
		}

		void cleanupPhysics(bool interactive)
		{
			PX_UNUSED(interactive);
			gScene->release();
			gDispatcher->release();
			gPhysics->release();
			PxPvdTransport* transport = gPvd->getTransport();
			gPvd->release();
			transport->release();

			gFoundation->release();
		}

		virtual void simulate(void) final
		{
			stepPhysics(true);
		}

		RENDER_DEBUG::RenderDebug		*mRenderDebug{ nullptr };
		RENDER_DEBUG::RenderDebugTyped	*mRenderDebugTyped{ nullptr };
		RenderDebugPhysX				*mRenderDebugPhysX{ nullptr };
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
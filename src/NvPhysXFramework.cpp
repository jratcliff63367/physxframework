#include "NvPhysXFramework.h"
#include "RenderDebugPhysX.h"
#include <ctype.h>

#include "Nv.h"
#include "PxPhysicsAPI.h"
#include "NvRenderDebugTyped.h"
#include "PsTime.h"

using namespace physx;

#define HOST_NAME "localhost"
#define PVD_HOST "localhost"
#define USE_DEBUG 0

namespace NV_PHYSX_FRAMEWORK
{




	class PhysXFrameworkImpl : public PhysXFramework, public RenderDebugPhysX::Interface
	{
	public:
		PhysXFrameworkImpl(void)
		{
			const char *dllName = NULL;
#ifdef _WIN64
			dllName = "NvRenderDebug_x64.dll";
#else
#if USE_DEBUG
			dllName = "NvRenderDebugDEBUG_x86.dll";
#else
			dllName = "NvRenderDebug_x86.dll";
#endif
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
			initPhysics();
			if (mScene && mRenderDebug )
			{
				mRenderDebugPhysX = createRenderDebugPhysX(mScene, mRenderDebug, true);
				if (mRenderDebugPhysX)
				{
					mRenderDebugPhysX->setInterface(this);
				}
			}
			mTime.getElapsedSeconds();
		}

		virtual ~PhysXFrameworkImpl(void)
		{
			if (mRenderDebugPhysX)
			{
				mRenderDebugPhysX->release();
			}
			if (mRenderDebug)
			{
				mRenderDebug->release();
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
			PxRigidDynamic* dynamic = PxCreateDynamic(*mPhysics, t, geometry, *mMaterial, 10.0f);
			dynamic->setAngularDamping(0.5f);
			dynamic->setLinearVelocity(velocity);
			mScene->addActor(*dynamic);
			return dynamic;
		}

		void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
		{
			PxShape* shape = mPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *mMaterial);
			for (PxU32 i = 0; i < size; i++)
			{
				for (PxU32 j = 0; j < size - i; j++)
				{
					PxTransform localTm(PxVec3(PxReal(j * 2) - PxReal(size - i), PxReal(i * 2 + 1), 0) * halfExtent);
					PxRigidDynamic* body = mPhysics->createRigidDynamic(t.transform(localTm));
					body->attachShape(*shape);
					PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
					mScene->addActor(*body);
				}
			}
			shape->release();
		}

		void initPhysics(void)
		{
			mFoundation = PxCreateFoundation(PX_FOUNDATION_VERSION, mAllocator, mErrorCallback);

			mPvd = PxCreatePvd(*mFoundation);
			PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
			mPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

			mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale(), true, mPvd);

			PxSceneDesc sceneDesc(mPhysics->getTolerancesScale());
			sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
			mDispatcher = PxDefaultCpuDispatcherCreate(2);
			sceneDesc.cpuDispatcher = mDispatcher;
			sceneDesc.filterShader = PxDefaultSimulationFilterShader;
			mScene = mPhysics->createScene(sceneDesc);

			PxPvdSceneClient* pvdClient = mScene->getScenePvdClient();
			if (pvdClient)
			{
				pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
				pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
				pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
			}
			mMaterial = mPhysics->createMaterial(0.5f, 0.5f, 0.6f);

			PxRigidStatic* groundPlane = PxCreatePlane(*mPhysics, PxPlane(0, 1, 0, 0), *mMaterial);
			mScene->addActor(*groundPlane);


		}

		float stepPhysics(bool interactive)
		{
			PX_UNUSED(interactive);
			float dtime = float(mTime.getElapsedSeconds());
			mScene->simulate(dtime);
			mScene->fetchResults(true);
			if (mRenderDebugPhysX)
			{
				mRenderDebugPhysX->render(dtime, true, true, true, false);
			}
			return dtime;
		}

		void cleanupPhysics(bool interactive)
		{
			PX_UNUSED(interactive);
			mScene->release();
			mDispatcher->release();
			mPhysics->release();
			PxPvdTransport* transport = mPvd->getTransport();
			mPvd->release();
			transport->release();

			mFoundation->release();
		}

		virtual void simulate(void) final
		{
			float dtime = stepPhysics(true);
			if (mRenderDebug )
			{
				mRenderDebug->render(dtime, nullptr);
			}
		}

		virtual void setCommandCallback(CommandCallback *cc)
		{
			mCommandCallback = cc;
		}

		virtual bool processDebugCommand(uint32_t argc, const char **argv)
		{
			bool ret = false;

			if (mCommandCallback)
			{
				ret = mCommandCallback->processDebugCommand(argc, argv);
			}
			return ret;
		}

		// Create a default series of stacked boxes for testing purposes
		virtual void createDefaultStacks(void)
		{
			for (PxU32 i = 0; i < 5; i++)
			{
				createStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f);
			}
		}

		CommandCallback					*mCommandCallback{ nullptr };
		RENDER_DEBUG::RenderDebug		*mRenderDebug{ nullptr };
		RENDER_DEBUG::RenderDebugTyped	*mRenderDebugTyped{ nullptr };
		RenderDebugPhysX				*mRenderDebugPhysX{ nullptr };
		PxDefaultAllocator				mAllocator;
		PxDefaultErrorCallback			mErrorCallback;
		PxFoundation					*mFoundation{ nullptr };
		PxPhysics						*mPhysics{ nullptr };
		PxDefaultCpuDispatcher			*mDispatcher{ nullptr };
		PxScene							*mScene{ nullptr };
		PxMaterial						*mMaterial{ nullptr };
		PxPvd							*mPvd{ nullptr };
		physx::shdfnd::Time				mTime;
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
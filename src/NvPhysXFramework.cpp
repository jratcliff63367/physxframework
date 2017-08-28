#include "NvPhysXFramework.h"
#include "RenderDebugPhysX.h"
#include <ctype.h>

#include "Nv.h"
#include "PxPhysicsAPI.h"
#include "NvBounds3.h"
#include "NvRenderDebugTyped.h"
#include "PsTime.h"

using namespace physx;

#define HOST_NAME "localhost"
#define PVD_HOST "localhost"
#define USE_DEBUG 0
#define USE_BOXES 0	// if true, create boxes instead of convex hulls for simulation behavior testing

#pragma warning(disable:4100)

namespace NV_PHYSX_FRAMEWORK
{

	class ConvexMeshImp : public PhysXFramework::ConvexMesh
	{
	public:
		ConvexMeshImp(uint32_t vcount,
			const float *vertices,
			uint32_t tcount,
			const uint32_t *indices,
			PxCooking *cooking,
			PxPhysics *physics)
		{

			PxConvexMeshDesc desc;
			desc.points.data = vertices;
			desc.points.count = vcount;
			desc.points.stride = sizeof(float) * 3;
			desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

			mConvexMesh = cooking->createConvexMesh(desc, physics->getPhysicsInsertionCallback());

			mBounds.setEmpty();
			for (uint32_t i = 0; i < vcount; i++)
			{
				const float *p = &vertices[i * 3];
				PxVec3 v(p[0], p[1], p[2]);
				mBounds.include(v);
			}


		}
		virtual ~ConvexMeshImp(void)
		{
			if (mConvexMesh)
			{
				mConvexMesh->release();
			}
		}

		virtual void release(void) final
		{
			delete this;
		};

		PxBounds3				mBounds;
		PxConvexMesh	*mConvexMesh{ nullptr };
	}; 

	class CompoundActorImpl : public PhysXFramework::CompoundActor
	{
	public:
		CompoundActorImpl(PxPhysics *p,PxScene *scene,PxMaterial *defaultMaterial) : mPhysics(p), mScene(scene), mDefaultMaterial(defaultMaterial)
		{
			mActor = p->createRigidDynamic(PxTransform(PxIdentity));
		}

		virtual ~CompoundActorImpl(void)
		{
			if (mActor)
			{
				mActor->release();
			}
		}

		virtual void addConvexMesh(PhysXFramework::ConvexMesh *cmesh,
			float meshPosition[3],
			float meshScale[3]) final
		{
			if (!mActor) return;
			ConvexMeshImp *cm = static_cast<ConvexMeshImp *>(cmesh);
#if USE_BOXES
			PxBoxGeometry	box;
			PxVec3 dimensions = cm->mBounds.getDimensions();
			box.halfExtents.x = dimensions.x*0.5f*meshScale[0];
			box.halfExtents.y = dimensions.x*0.5f*meshScale[1];
			box.halfExtents.z = dimensions.x*0.5f*meshScale[2];
			PxTransform localPose(PxIdentity);
			localPose.p = PxVec3(meshPosition[0], meshPosition[1], meshPosition[2]);
			PxShape *shape = mPhysics->createShape(box, *mDefaultMaterial, true);
			if (shape)
			{
				shape->setLocalPose(localPose);
				mActor->attachShape(*shape);
			}
#else
			PxConvexMeshGeometry	convex;
			convex.convexMesh = cm->mConvexMesh;
			convex.scale = PxVec3(meshScale[0], meshScale[1], meshScale[2]);
			PxTransform localPose(PxIdentity);
			localPose.p = PxVec3(meshPosition[0], meshPosition[1], meshPosition[2]);
			PxShape *shape = mPhysics->createShape(convex, *mDefaultMaterial, true);
			if (shape)
			{
				shape->setLocalPose(localPose);
				mActor->attachShape(*shape);
			}
#endif
		}

		// Create a simulated actor based on the collection of convex meshes
		virtual void createActor(const float centerOfMass[3],float mass) final
		{
			if (mActor)
			{
				PxVec3 com(centerOfMass[0], centerOfMass[1], centerOfMass[2]);
				PxTransform p(com);
				mActor->setCMassLocalPose(p);
				mActor->setMass(mass);
				PxRigidBodyExt::setMassAndUpdateInertia(*mActor, mActor->getMass());
				mScene->addActor(*mActor);
			}
		}

		virtual void getXform(float xform[16])
		{
			if (mActor)
			{
				PxTransform p = mActor->getGlobalPose();
				PxMat44 m(p);
				memcpy(xform, m.front(), sizeof(float)*16);
			}
		}

		virtual void release(void) final
		{
			delete this;
		}

		PxPhysics			*mPhysics{ nullptr };
		PxScene				*mScene{ nullptr };
		PxMaterial			*mDefaultMaterial{ nullptr };
		PxRigidDynamic		*mActor{ nullptr };
	};

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
			cleanupPhysics();
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
			PxRigidBodyExt::setMassAndUpdateInertia(*dynamic, dynamic->getMass());
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

		PxTolerancesScale getTolerancesScale(void)
		{
			PxTolerancesScale s;
			return s;
		}

		void initPhysics(void)
		{
			mFoundation = PxCreateFoundation(PX_FOUNDATION_VERSION, mAllocator, mErrorCallback);

			mPvd = PxCreatePvd(*mFoundation);
			PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
			mPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

			mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, getTolerancesScale(), true, mPvd);
			mCooking = PxCreateCooking(PX_PHYSICS_VERSION, *mFoundation, PxCookingParams(getTolerancesScale()));

			PxSceneDesc sceneDesc(mPhysics->getTolerancesScale());
			sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
			mDispatcher = PxDefaultCpuDispatcherCreate(2);
			sceneDesc.cpuDispatcher = mDispatcher;
//			sceneDesc.flags &= ~PxSceneFlag::eENABLE_PCM;
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
			float STEP_TIME = 1.0f / 60.0f;
			float dtime = float(mTime.getElapsedSeconds());
			if (dtime > (1.0f / 10.0f))
			{
				dtime = 1.0f / 10.0f;
			}
			mElapsedTime += dtime;
			while (mElapsedTime > STEP_TIME)
			{
				mScene->simulate(STEP_TIME);
				mScene->fetchResults(true);
				mElapsedTime -= STEP_TIME;
			}
			return dtime;
		}

		void cleanupPhysics(void)
		{
			mScene->release();
			mDispatcher->release();
			mPhysics->release();
			mCooking->release();
			PxPvdTransport* transport = mPvd->getTransport();
			mPvd->release();
			transport->release();
			mFoundation->release();
		}

		virtual float simulate(bool showPhysics) final
		{
			float dtime = stepPhysics(true);

			if (mRenderDebugPhysX )
			{
				mRenderDebugPhysX->render(dtime, showPhysics, showPhysics, showPhysics, false);
			}
			if (mRenderDebug )
			{
				mRenderDebug->render(dtime, nullptr);
			}
			return dtime;
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

		// Create a convex mesh using the provided raw triangles describing a convex hull.
		virtual ConvexMesh *createConvexMesh(uint32_t vcount,
			const float *vertices,
			uint32_t tcount,
			const uint32_t *indices) final
		{
			ConvexMeshImp *c = new ConvexMeshImp(vcount, vertices, tcount, indices,mCooking,mPhysics);
			return static_cast<ConvexMesh *>(c);
		}

		// Create a physically simulated compound actor comprised of a collection of convex meshes
		virtual CompoundActor *createCompoundActor(void) final
		{
			CompoundActorImpl *c = new CompoundActorImpl(mPhysics, mScene, mMaterial);
			return static_cast<CompoundActor *>(c);
		}

		// create a box in the simulated scene
		virtual void createBox(const float boxSize[3], const float boxPosition[3])
		{
			PxVec3 pos(boxPosition[0], boxPosition[1], boxPosition[2]);
			PxRigidDynamic *actor = mPhysics->createRigidDynamic(PxTransform(pos));
			PxBoxGeometry	box;
			box.halfExtents.x = boxSize[0] * 0.5f;
			box.halfExtents.y = boxSize[1] * 0.5f;
			box.halfExtents.z = boxSize[2] * 0.5f;
			PxShape *shape = mPhysics->createShape(box, *mMaterial, true);
			if (shape)
			{
				actor->attachShape(*shape);
				PxRigidBodyExt::setMassAndUpdateInertia(*actor, actor->getMass());
				mScene->addActor(*actor);
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
		PxCooking						*mCooking{ nullptr };
		PxDefaultCpuDispatcher			*mDispatcher{ nullptr };
		PxScene							*mScene{ nullptr };
		PxMaterial						*mMaterial{ nullptr };
		PxPvd							*mPvd{ nullptr };
		physx::shdfnd::Time				mTime;
		float							mElapsedTime{ 0 };
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
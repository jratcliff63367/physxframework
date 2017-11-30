#include "NvPhysXFramework.h"
#include "RenderDebugPhysX.h"
#include "SomeOfEverything.h"
#include "PhysicsDOMPhysX.h"
#include "ImportPhysXDOM.h"
#include "PhysicsDOMDef.h"
#include <ctype.h>

#include "Nv.h"
#include "PxPhysicsAPI.h"
#include "NvBounds3.h"
#include "NvRenderDebugTyped.h"
#include "PsTime.h"
#include <vector>
#include <unordered_set>

using namespace physx;

#define HOST_NAME "localhost"
#define PVD_HOST "localhost"
#define USE_DEBUG 0

#pragma warning(disable:4100)

namespace NV_PHYSX_FRAMEWORK
{

	class PhysicsDOMContainerImpl : public PhysicsDOMContainer
	{
	public:
		PhysicsDOMContainerImpl(void)
		{
		}
		virtual ~PhysicsDOMContainerImpl(void)
		{
		}

		virtual const PHYSICS_DOM::PhysicsDOM *getPhysicsDOM(void)
		{
			mDOM.initDOM();
			return mDOM.getPhysicsDOM();
		}

		virtual void release(void)
		{
			delete this;
		}

		PHYSICS_DOM::PhysicsDOMDef	mDOM;
	};

typedef std::unordered_set< uint64_t > CollisionFilterSet;

CollisionFilterSet gCollisionFilters;

PxFilterFlags contactReportFilterShader(PxFilterObjectAttributes attributes0, PxFilterData filterData0,
	PxFilterObjectAttributes attributes1, PxFilterData filterData1,
	PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	PX_UNUSED(attributes0);
	PX_UNUSED(attributes1);
	PX_UNUSED(filterData0);
	PX_UNUSED(filterData1);
	PX_UNUSED(constantBlockSize);
	PX_UNUSED(constantBlock);

	if (filterData0.word0 && filterData1.word0)
	{
		uint64_t c1 = filterData0.word0;
		uint64_t c2 = filterData1.word0;
		uint64_t c = (c1 << 32) | c2;
		const CollisionFilterSet::iterator found = gCollisionFilters.find(c);
		if (found != gCollisionFilters.end())
		{
			return PxFilterFlag::eKILL;
		}
	}

	// all initial and persisting reports for everything, with per-point data
	pairFlags = PxPairFlag::eSOLVE_CONTACT | PxPairFlag::eDETECT_DISCRETE_CONTACT
		| PxPairFlag::eNOTIFY_TOUCH_FOUND
		| PxPairFlag::eNOTIFY_TOUCH_PERSISTS
		| PxPairFlag::eNOTIFY_CONTACT_POINTS;
	return PxFilterFlag::eDEFAULT;
}


typedef	std::vector< PxRigidDynamic * > PxActorVector;
typedef std::vector< PxShape * > PxShapeVector;
typedef std::vector< PxJoint * > PxJointVector;

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
		CompoundActorImpl(PxPhysics *p,
						PxScene *scene, 
						PxMaterial *defaultMaterial,
						RenderDebugPhysX *renderDebugPhysX) : mPhysics(p), mScene(scene), mDefaultMaterial(defaultMaterial), mRenderDebugPhysX(renderDebugPhysX)
		{
		}

		virtual ~CompoundActorImpl(void)
		{
			for (size_t i = 0; i < mJoints.size(); i++)
			{
				mJoints[i]->release();
			}
			for (size_t i = 0; i < mActors.size(); i++)
			{
				mActors[i]->release();
			}
		}

		virtual void addConvexMesh(PhysXFramework::ConvexMesh *cmesh,
			float meshPosition[3],
			float meshScale[3]) final
		{
			ConvexMeshImp *cm = static_cast<ConvexMeshImp *>(cmesh);
			PxConvexMeshGeometry	convex;
			convex.convexMesh = cm->mConvexMesh;
			convex.scale = PxVec3(meshScale[0], meshScale[1], meshScale[2]);
			PxTransform localPose(PxIdentity);
			localPose.p = PxVec3(meshPosition[0], meshPosition[1], meshPosition[2]);
			PxShape *shape = mPhysics->createShape(convex, *mDefaultMaterial, true);
			if (shape)
			{
				shape->setLocalPose(localPose);
				mShapes.push_back(shape);
			}
		}

		// Create a simulated actor based on the collection of convex meshes
		virtual void createActor(const float centerOfMass[3], float mass, bool asRagdoll) final
		{
			if (asRagdoll)
			{
				mass = mass / float(mShapes.size());
				for (size_t i = 0; i < mShapes.size(); i++)
				{
					PxRigidDynamic *actor = mPhysics->createRigidDynamic(PxTransform(PxIdentity));
					mActors.push_back(actor);
					PxShape *s = mShapes[i];
					PxFilterData filterData;
					filterData.word0 = uint32_t(i + 1);
					s->setSimulationFilterData(filterData);
					PxTransform actorPose = s->getLocalPose();
					PxTransform identityPose(PxIdentity);
					s->setLocalPose(identityPose);
					actor->setGlobalPose(actorPose);
					actor->attachShape(*s);
					PxRigidBodyExt::setMassAndUpdateInertia(*actor, mass);
					mScene->addActor(*actor);
				}
			}
			else
			{
				PxRigidDynamic *actor = mPhysics->createRigidDynamic(PxTransform(PxIdentity));
				mActors.push_back(actor);
				for (size_t i = 0; i < mShapes.size(); i++)
				{
					PxShape *s = mShapes[i];
					actor->attachShape(*s);
				}
				PxVec3 com(centerOfMass[0], centerOfMass[1], centerOfMass[2]);
				PxTransform p(com);
				actor->setCMassLocalPose(p);
				actor->setMass(mass);
				PxRigidBodyExt::setMassAndUpdateInertia(*actor, actor->getMass());
				mScene->addActor(*actor);
			}
		}

		// Sets the collision filter pairs.
		virtual void setCollisionFilterPairs(uint32_t pairCount, const uint32_t *collisionPairs)
		{
			gCollisionFilters.clear();
#if 0
			uint32_t bodyCount = uint32_t(mActors.size());
			for (uint32_t i = 0; i < bodyCount; i++)
			{
				for (uint32_t j = i + 1; j < bodyCount; j++)
				{
					uint64_t p1 = i + 1;
					uint64_t p2 = j + 1;
					uint64_t c1 = (p1 << 32) | p2;
					uint64_t c2 = (p2 << 32) | p1;
					gCollisionFilters.insert(c1);
					gCollisionFilters.insert(c2);
				}
			}
#else
			// Add all collision pair filter permutations to the collion filter set
			for (uint32_t i = 0; i < pairCount; i++)
			{
				uint64_t p1 = collisionPairs[i * 2 + 0]+1;
				uint64_t p2 = collisionPairs[i * 2 + 1]+1;
				uint64_t c1 = (p1 << 32) | p2;
				uint64_t c2 = (p2 << 32) | p1;
				gCollisionFilters.insert(c1);
				gCollisionFilters.insert(c2);
			}
#endif
		}

		// If we are mouse dragging and the currently selected object is an actor in this compound
		// system, then return true and assign 'bodyIndex' to the index number of the body selected.
		virtual bool getSelectedBody(uint32_t &bodyIndex) final
		{
			bool ret = false;

			bodyIndex = 0;
			PxRigidActor *actor = mRenderDebugPhysX->getSelectedActor();
			if (actor)
			{
				for (size_t i = 0; i < mActors.size(); ++i)
				{
					if (actor == mActors[i])
					{
						bodyIndex = uint32_t(i);
						ret = true;
						break;
					}
				}
			}

			return ret;
		}


		virtual bool getConstraintXform(float xform[16], uint32_t constraint)
		{
			bool ret = false;

			if (constraint < uint32_t(mJoints.size()))
			{
				PxJoint *j = mJoints[constraint];
				PxTransform p = j->getLocalPose(PxJointActorIndex::eACTOR0);
				PxRigidActor *a1;
				PxRigidActor *a2;
				j->getActors(a1, a2);
				if (a1)
				{
					p = a1->getGlobalPose()*p;
				}
				PxMat44 m(p);
				memcpy(xform, m.front(), sizeof(float) * 16);
				ret = true;
			}

			return ret;
		}

		virtual bool getXform(float xform[16],uint32_t index)
		{
			bool ret = false;
			if (index < uint32_t(mActors.size()))
			{
				PxRigidDynamic *a = mActors[index];
				PxTransform p = a->getGlobalPose();
				PxMat44 m(p);
				memcpy(xform, m.front(), sizeof(float)*16);
				ret = true;
			}
			return ret;
		}

		// fixed joint
		PxJoint* createFixedJoint(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1)
		{
			PxFixedJoint* j = PxFixedJointCreate(*mPhysics, a0, t0, a1, t1);
			j->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true); // enable visualization!!
			j->setConstraintFlag(PxConstraintFlag::eDISABLE_PREPROCESSING, true);
			return j;
		}

		// spherical joint limited to an angle of at most pi/4 radians (45 degrees)
		PxJoint* createLimitedSpherical(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1,uint32_t swing1Limit,uint32_t swing2Limit)
		{
			PxSphericalJoint* j = PxSphericalJointCreate(*mPhysics, a0, t0, a1, t1);

			float lrange1 = (PxPi * 2) * (float(swing1Limit) / 360.0f);
			float lrange2 = (PxPi * 2) * (float(swing2Limit) / 360.0f);

			j->setLimitCone(PxJointLimitCone(lrange1, lrange2));
			j->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, true);
			j->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true); // enable visualization!!
			j->setConstraintFlag(PxConstraintFlag::eDISABLE_PREPROCESSING, true);

			return j;
		}

		// D6 joint with a spring maintaining its position
		PxJoint* createHingeJoint(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1, uint32_t limitRangeDegrees)
		{
			PxRevoluteJoint* j = PxRevoluteJointCreate(*mPhysics, a0, t0, a1, t1);
			j->setConstraintFlag(PxConstraintFlag::eDISABLE_PREPROCESSING, true);

			float lrange = (PxPi * 2) * (float(limitRangeDegrees) / 360.0f);
			PxJointAngularLimitPair limit(-lrange, lrange);
			j->setLimit(limit);
			j->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);

			j->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true); // enable visualization!!
			return j;
		}

		// Creates a fixed constraint between these two bodies
		virtual bool createConstraint(uint32_t bodyA, uint32_t bodyB, const float worldPos[3],					// World position of the constraint location
			const float worldOrientation[4],
			ConstraintType type,			// Type of constraint to use
			float	distanceLimit,
			uint32_t twistLimit,			// Twist limit in degrees (if used)
			uint32_t swing1Limit,			// Swing 1 limit in degrees (if used)
			uint32_t swing2Limit) final		// Swing 2 limit in degrees (if used)
		{
			bool ret = false;
			uint32_t actorCount = uint32_t(mActors.size());
			if (bodyA < actorCount && bodyB < actorCount)
			{
				PxRigidDynamic *a1 = mActors[bodyA];
				PxRigidDynamic *a2 = mActors[bodyB];

				PxTransform constraintWorld;

				constraintWorld.p.x = worldPos[0];
				constraintWorld.p.y = worldPos[1];
				constraintWorld.p.z = worldPos[2];

				constraintWorld.q.x = worldOrientation[0];
				constraintWorld.q.y = worldOrientation[1];
				constraintWorld.q.z = worldOrientation[2];
				constraintWorld.q.w = worldOrientation[3];

				PxTransform inverse1 = a1->getGlobalPose().getInverse();
				PxTransform other1 = inverse1 * constraintWorld;


				PxTransform inverse2 = a2->getGlobalPose().getInverse();
				PxTransform other2 = inverse2 * constraintWorld;

				PxJoint *joint = nullptr;

				switch (type)
				{
					case CT_FIXED:
						joint = createFixedJoint(a1, other1, a2, other2);
						break;
					case CT_HINGE:
						joint = createHingeJoint(a1, other1, a2, other2, twistLimit);
						break;
					case CT_SPHERICAL:
						joint = createLimitedSpherical(a1, other1, a2, other2, swing1Limit, swing2Limit);
						break;
					case CT_BALL_AND_SOCKET:
						break;
					case CT_REVOLUTE:
						break;
				}
				if (joint)
				{
					mJoints.push_back(joint);
				}
			}
			return ret;
		}

		virtual void release(void) final
		{
			delete this;
		}

		PxPhysics			*mPhysics{ nullptr };
		PxScene				*mScene{ nullptr };
		PxMaterial			*mDefaultMaterial{ nullptr };
		RenderDebugPhysX	*mRenderDebugPhysX{ nullptr };

		PxActorVector		mActors;
		PxShapeVector		mShapes;
		PxJointVector		mJoints;
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
			if (mPhysicsDOMPhysX)
			{
				mPhysicsDOMPhysX->release();
			}
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
			PxInitExtensions(*mPhysics, mPvd);
			PxSceneDesc sceneDesc(mPhysics->getTolerancesScale());
			sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
			mDispatcher = PxDefaultCpuDispatcherCreate(2);
			sceneDesc.cpuDispatcher = mDispatcher;
			sceneDesc.filterShader = contactReportFilterShader;
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
			if (mPaused)
			{
				return 0;
			}
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
				if (mScene)
				{
					mScene->simulate(STEP_TIME);
					mScene->fetchResults(true);
				}
				mElapsedTime -= STEP_TIME;
			}
			return dtime;
		}

		void cleanupPhysics(void)
		{
			if (mSomeOfEverything)
			{
				mSomeOfEverything->release();
			}
			if (mScene)
			{
				mScene->release();
			}
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
			CompoundActorImpl *c = new CompoundActorImpl(mPhysics, mScene, mMaterial,mRenderDebugPhysX);
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

		virtual void setDragForce(float force)
		{
			if (mRenderDebugPhysX)
			{
				mRenderDebugPhysX->setDragForce(force);
			}
		}

		/**
		Creates two example collections:
		- collection with actors and joints that can be instantiated multiple times in the scene
		- collection with shared objects
		*/
		void createCollections(PxPhysics *physics,
								PxCollection* &sharedCollection, 
								PxCollection* &actorCollection, 
								PxSerializationRegistry& sr)
		{

			sharedCollection = PxCreateCollection();		// collection for all the shared objects
			actorCollection = PxCreateCollection();			// collection for all the nonshared objects
			// Serialize materials
			{
				uint32_t count = physics->getNbMaterials();
				if (count)
				{
					physx::PxMaterial **objects = new physx::PxMaterial*[count];
					physics->getMaterials(objects, count, 0);
					for (uint32_t i = 0; i < count; i++)
					{
						sharedCollection->add(*objects[i]);
					}
					delete[]objects;
				}
			}


			// Serialize shapes
			{
				uint32_t count = physics->getNbShapes();
				if (count)
				{
					physx::PxShape **objects = new physx::PxShape*[count];
					physics->getShapes(objects, count, 0);
					for (uint32_t i = 0; i < count; i++)
					{
						sharedCollection->add(*objects[i]);
					}
					delete[]objects;
				}
			}


			// Serialize triangle meshes
			{
				uint32_t count = physics->getNbTriangleMeshes();
				if (count)
				{
					physx::PxTriangleMesh **objects = new physx::PxTriangleMesh*[count];
					physics->getTriangleMeshes(objects, count, 0);
					for (uint32_t i = 0; i < count; i++)
					{
						sharedCollection->add(*objects[i]);
					}
					delete[]objects;
				}
			}
			// Serialize convex hulls
			{
				uint32_t count = physics->getNbConvexMeshes();
				if (count)
				{
					physx::PxConvexMesh **objects = new physx::PxConvexMesh*[count];
					physics->getConvexMeshes(objects, count, 0);
					for (uint32_t i = 0; i < count; i++)
					{
						sharedCollection->add(*objects[i]);
					}
					delete[]objects;
				}
			}
			// Serialize heightfields
			{
				uint32_t count = physics->getNbHeightFields();
				if (count)
				{
					physx::PxHeightField **objects = new physx::PxHeightField*[count];
					physics->getHeightFields(objects, count, 0);
					for (uint32_t i = 0; i < count; i++)
					{
						sharedCollection->add(*objects[i]);
					}
					delete[]objects;
				}
			}
			// Serialize cloth fabrics
			{
				uint32_t count = physics->getNbClothFabrics();
				if (count)
				{
					physx::PxClothFabric **objects = new physx::PxClothFabric*[count];
					physics->getClothFabrics(objects, count);
					for (uint32_t i = 0; i < count; i++)
					{
						sharedCollection->add(*objects[i]);
					}
					delete[]objects;
				}
			}
			// Seralize the contents of each scene
			{
				uint32_t count = physics->getNbScenes();
				if (count)
				{
					physx::PxScene **objects = new physx::PxScene*[count];
					physics->getScenes(objects, count, 0);
					for (uint32_t i = 0; i < count; i++)
					{
						PxScene *scene = objects[i];
						auto addActors = [this](PxCollection *actorCollection,PxScene *scene, PxActorTypeFlags type)
						{
							PxU32 acount = scene->getNbActors(type);
							PxActor **actors = new PxActor*[acount];
							scene->getActors(type, actors, acount, 0);
							for (uint32_t j = 0; j < acount; j++)
							{
								actorCollection->add(*actors[j]);
							}
							delete[]actors;
						};
						addActors(actorCollection, scene, PxActorTypeFlag::eRIGID_STATIC);
						addActors(actorCollection, scene, PxActorTypeFlag::eRIGID_DYNAMIC);
						addActors(actorCollection, scene, PxActorTypeFlag::eCLOTH);
						addActors(actorCollection, scene, PxActorTypeFlag::ePARTICLE_FLUID);
						addActors(actorCollection, scene, PxActorTypeFlag::ePARTICLE_SYSTEM);
						// Add all of the constraints
						{
							uint32_t jcount = scene->getNbConstraints();
							if (jcount)
							{
								PxConstraint **constraints = new PxConstraint *[jcount];
								scene->getConstraints(constraints, jcount, 0);
								for (uint32_t j = 0; j < jcount; j++)
								{
									actorCollection->add(*constraints[j]);
								}
								delete[]constraints;
							}
						}
						// Add all of the articulations
						{
							uint32_t jcount = scene->getNbArticulations();
							if (jcount)
							{
								PxArticulation **articulations = new PxArticulation *[jcount];
								scene->getArticulations(articulations, jcount, 0);
								for (uint32_t j = 0; j < jcount; j++)
								{
									actorCollection->add(*articulations[j]);
								}
								delete[]articulations;
							}
						}
						// Add all of the aggregates
						{
							uint32_t jcount = scene->getNbAggregates();
							if (jcount)
							{
								PxAggregate **aggregates = new PxAggregate *[jcount];
								scene->getAggregates(aggregates, jcount, 0);
								for (uint32_t j = 0; j < jcount; j++)
								{
									actorCollection->add(*aggregates[j]);
								}
								delete[]aggregates;
							}
						}
					}
				}
			}
			PxSerialization::complete(*sharedCollection, sr);
			PxSerialization::createSerialObjectIds(*sharedCollection, PxSerialObjectId(1));	// arbitrary choice of base for references to shared objects
			PxSerialization::complete(*actorCollection, sr, sharedCollection, true);
		}


		/**
		Create objects, add them to collections and serialize the collections to the steams gSharedStream and gActorStream
		This function doesn't setup the gPhysics global as the corresponding physics object is only used locally
		*/
		void serializeObjects(PxPhysics *physics,PxOutputStream& sharedStream, PxOutputStream& actorStream)
		{
			PxSerializationRegistry* sr = PxSerialization::createSerializationRegistry(*physics);

			PxCollection* sharedCollection = NULL;
			PxCollection* actorCollection = NULL;
			createCollections(physics,sharedCollection, actorCollection, *sr);

			PxSerialization::serializeCollectionToXml(sharedStream, *sharedCollection, *sr);
			PxSerialization::serializeCollectionToXml(actorStream, *actorCollection, *sr, NULL, sharedCollection);

			actorCollection->release();
			sharedCollection->release();

			sr->release();
		}


		// serialize the current state to an XML file
		virtual void serializeXML(const char *fname, PxPhysics *physics)
		{
			// Alternatively PxDefaultFileOutputStream could be used 
			PxDefaultMemoryOutputStream sharedOutputStream;
			PxDefaultMemoryOutputStream actorOutputStream;
			serializeObjects(physics, sharedOutputStream, actorOutputStream);
			{
				FILE *fph = fopen(fname, "wb");
				if (fph)
				{
					fwrite(sharedOutputStream.getData(), sharedOutputStream.getSize(), 1, fph);
					fwrite(actorOutputStream.getData(), actorOutputStream.getSize(), 1, fph);
					fclose(fph);
				}
			}
		}


		// serialize the current state to an XML file
		virtual void serializeXML(const char *fname) final
		{
			serializeXML(fname, mPhysics);
		}



		virtual void setPauseState(bool state) final
		{
			mPaused = state;
		}

		virtual bool getPauseState(void) const final
		{
			return mPaused;
		}

		// Create some of everything so we can serialize the scenes and get a detailed
		// XML output for analysis
		virtual void createSomeOfEverything(void)
		{
			if (mSomeOfEverything)
			{
				mSomeOfEverything->release();
			}
			mSomeOfEverything = SOME_OF_EVERYTHING::SomeOfEverything::create(mPhysics, mCooking, mScene);
		}

		// Load this physics DOM
		virtual bool loadPhysicsDOM(const PHYSICS_DOM::PhysicsDOM &physicsDOM)
		{
			bool ret = false;

			if (mPhysicsDOMPhysX)
			{
				mPhysicsDOMPhysX->release();
			}
			mPhysicsDOMPhysX = PHYSICS_DOM_PHYSX::PhysicsDOMPhysX::create(mPhysics, mCooking);
			mPhysicsDOMPhysX->loadPhysicsDOM(physicsDOM,mScene);

			return ret;
		}

		virtual PhysicsDOMContainer *importPhysXDOM(const char *fname) final
		{
			PhysicsDOMContainer *ret = nullptr;

			IMPORT_PHYSX_DOM::ImportPhysXDOM *imp = IMPORT_PHYSX_DOM::ImportPhysXDOM::create();

			PhysicsDOMContainerImpl *pcontain = new PhysicsDOMContainerImpl;

			bool ok = imp->importPhysXDOM(fname, pcontain->mDOM);
			if (ok)
			{
				ret = static_cast<PhysicsDOMContainer *>(pcontain);
			}
			else
			{
				pcontain->release();
			}

			imp->release();

			return ret;
		}


		bool							mPaused{ false };
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
		SOME_OF_EVERYTHING::SomeOfEverything				*mSomeOfEverything{ nullptr };
		PHYSICS_DOM_PHYSX::PhysicsDOMPhysX *mPhysicsDOMPhysX{ nullptr };
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
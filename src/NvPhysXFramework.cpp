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

		// Does a named lookup of a node with this ID and, if found, returns the 'NodeState'
		// interface which can be used to query current state of this object
		virtual PHYSICS_DOM::NodeState *getNodeState(const char *nodeId) final
		{
			PHYSICS_DOM::NodeState *ret = nullptr;

			if (mPhysicsDOMPhysX)
			{
				ret = mPhysicsDOMPhysX->getNodeState(nodeId);
			}

			return ret;
		}

		virtual void releasePhysicsDOM(void) final
		{
			if (mPhysicsDOMPhysX)
			{
				mPhysicsDOMPhysX->release();
				mPhysicsDOMPhysX = nullptr;
			}
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
#include "SomeOfEverything.h"
#include "PxPhysicsAPI.h"
#include "NvBounds3.h"
#include <vector>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#pragma warning(disable:4100)

#define NUMBER_OF_MATERIALS 2
#define NUMBER_OF_SHAPES 2

typedef std::vector< physx::PxMaterial *>		MaterialVector;
typedef std::vector< physx::PxShape *>			ShapeVector;
typedef std::vector< physx::PxConvexMesh *>		ConvexMeshVector;
typedef std::vector< physx::PxTriangleMesh * >	TriangleMeshVector;
typedef std::vector< physx::PxRigidStatic *>	RigidStaticVector;
typedef std::vector< physx::PxRigidDynamic *>	RigidDynamicVector;
typedef std::vector< physx::PxHeightField *>	HeightFieldVector;
typedef std::vector< physx::PxJoint *>			JointVector;
typedef std::vector< physx::PxArticulation *>	ArticulationVector;
typedef std::vector< physx::PxArticulationLink * > ArticulationLinkVector;

using namespace physx;

namespace SOME_OF_EVERYTHING
{

	// All of the different joint types
	enum JointType
	{
		JT_FIXED,
		JT_REVOLUTE,
		JT_PRISMATIC,
		JT_SPHERICAL,
		JT_D6,
		JT_DISTANCE,
		JT_ARTICULATION,
		JT_LAST
	};

#define HEIGHT_FIELD_SIZE 16

	// Mesh: 32 vertices 64 triangles
#define VERTEX_COUNT 32
#define TRIANGLE_COUNT 64
	float gVertices[VERTEX_COUNT * 3] = {
		-0.200678006,-6.840312958,6.840312958,
		-0.200678006,-4.724764824,6.840312958,
		-0.200678006,-4.724764824,4.439278126,
		-0.200678006,-0.649251997,2.597712994,
		-0.200678006,-0.649251997,-2.021457911,
		-0.200678006,5.880214214,-2.021457911,
		-0.200678006,6.840312958,-6.840312958,
		-0.200678006,1.902750969,-6.840312958,
		-0.200678006,1.902750969,-3.969496965,
		-0.200678006,-4.626715183,-3.969496965,
		-0.200678006,-4.626715183,-6.840312958,
		-0.200678006,-6.840312958,-6.840312958,
		-0.200678006,6.840312958,6.840312958,
		-0.200678006,5.880214214,2.597712994,
		-0.200678006,1.804700971,4.439278126,
		-0.200678006,1.804700971,6.840312958,
		0.200678006,-6.840312958,-6.840312958,
		0.200678006,-6.840312958,6.840312958,
		0.200678006,-4.724764824,6.840312958,
		0.200678006,1.804700971,6.840312958,
		0.200678006,6.840312958,6.840312958,
		0.200678006,6.840312958,-6.840312958,
		0.200678006,1.902750969,-6.840312958,
		0.200678006,-4.626715183,-6.840312958,
		0.200678006,1.804700971,4.439278126,
		0.200678006,-4.724764824,4.439278126,
		0.200678006,-0.649251997,2.597712994,
		0.200678006,5.880214214,2.597712994,
		0.200678006,5.880214214,-2.021457911,
		0.200678006,-4.626715183,-3.969496965,
		0.200678006,1.902750969,-3.969496965,
		0.200678006,-0.649251997,-2.021457911,
	};
	uint32_t gIndices[TRIANGLE_COUNT * 3] = {
		4,8,9,
		12,13,14,
		16,0,11,
		18,0,17,
		20,15,19,
		6,20,21,
		6,22,7,
		10,16,11,
		20,24,27,
		16,29,17,
		29,8,30,
		7,30,8,
		23,9,29,
		2,24,14,
		19,14,24,
		1,25,2,
		4,28,5,
		27,5,28,
		3,31,4,
		26,13,27,
		11,0,9,
		2,3,4,
		0,1,2,
		9,10,11,
		6,7,8,
		9,0,2,
		5,6,8,
		9,2,4,
		4,5,8,
		12,6,5,
		13,3,14,
		12,5,13,
		14,15,12,
		3,2,14,
		16,17,0,
		18,1,0,
		20,12,15,
		6,12,20,
		6,21,22,
		10,23,16,
		20,19,24,
		24,25,26,
		27,28,20,
		24,26,27,
		28,21,20,
		18,17,25,
		16,23,29,
		30,22,21,
		30,21,28,
		31,26,25,
		30,28,31,
		29,30,31,
		25,17,29,
		29,31,25,
		29,9,8,
		7,22,30,
		23,10,9,
		2,25,24,
		19,15,14,
		1,18,25,
		4,31,28,
		27,13,5,
		3,26,31,
		26,3,13,
	};


static float ranf(void)
{
	uint32_t v = rand();
	return float(v) / float(RAND_MAX);
}

class SomeOfEverythingImpl : public SomeOfEverything
{
public:
	SomeOfEverythingImpl(physx::PxPhysics *physics, physx::PxCooking *cooking,physx::PxScene *scene) : mScene(scene), mPhysics(physics),mCooking(cooking)
	{
		for (uint32_t y = 0; y < HEIGHT_FIELD_SIZE; y++)
		{
			for (uint32_t x = 0; x < HEIGHT_FIELD_SIZE; x++)
			{
				mHeightFieldData[y*HEIGHT_FIELD_SIZE + x] = x;
			}
		}

		for (uint32_t i = 0; i < NUMBER_OF_MATERIALS; i++)
		{
			float staticFriction = ranf();
			float dynamicFriction = ranf();
			float restitution = ranf();
			physx::PxMaterial *m = physics->createMaterial(staticFriction, dynamicFriction, restitution);
			mMaterials.push_back(m);
		}
		for (uint32_t i = 0; i < NUMBER_OF_SHAPES; i++)
		{
			physx::PxMaterial *m = getRandomMaterial();
			// Create a box shape
			{
				physx::PxVec3 dimensions;
				dimensions.x = ranf();
				dimensions.y = ranf();
				dimensions.z = ranf();
				physx::PxBoxGeometry box(dimensions);
				physx::PxShape *s = physics->createShape(box, *m);
				s->setName("@boxShape");
				mShapes.push_back(s);
			}
			// Create a sphere shape
			{
				physx::PxSphereGeometry sphere(ranf());
				physx::PxShape *s = physics->createShape(sphere, *m);
				s->setName("@sphereShape");
				mShapes.push_back(s);
			}
			// Create a capsule shape
			{
				physx::PxCapsuleGeometry capsule(ranf(), ranf());
				physx::PxShape *s = physics->createShape(capsule, *m);
				s->setName("@capsuleShape");
				mShapes.push_back(s);
			}
			// Create convex hull and convex hull shape
			{
				#define HULL_POINTS 16
				physx::PxVec3 points[HULL_POINTS];
				// Create a random point cloud for the convex hull
				for (auto &j : points)
				{
					j.x = ranf();
					j.y = ranf();
					j.z = ranf();
				}
				physx::PxConvexMeshDesc desc;
				desc.points.data = points;
				desc.points.count = HULL_POINTS;
				desc.points.stride = sizeof(float) * 3;
				desc.flags = physx::PxConvexFlag::eCOMPUTE_CONVEX;
				physx::PxConvexMesh *c = mCooking->createConvexMesh(desc, mPhysics->getPhysicsInsertionCallback());
				mConvexMeshes.push_back(c);
				float scale = ranf() + 1.0f;
				physx::PxVec3 sc(scale, scale, scale);
				physx::PxConvexMeshGeometry convex(c,sc);
				physx::PxShape *s = physics->createShape(convex, *m);
				s->setName("@convexShape");
				mShapes.push_back(s);
			}
			// Create triangle mesh and triangle mesh shape
			{
				physx::PxTriangleMeshDesc desc;
				desc.points.data = gVertices;
				desc.points.count = VERTEX_COUNT;
				desc.points.stride = sizeof(float) * 3;
				desc.triangles.data = gIndices;
				desc.triangles.count = TRIANGLE_COUNT;
				desc.triangles.stride = sizeof(uint32_t) * 3;

				if (mMaterialIndices == nullptr)
				{
					mMaterialIndices = new uint16_t[TRIANGLE_COUNT];
					for (uint32_t j = 0; j < TRIANGLE_COUNT; j++)
					{
						mMaterialIndices[j] = j & 3;
					}
				}
				desc.materialIndices.data = mMaterialIndices;
				desc.materialIndices.stride = sizeof(uint16_t);

				physx::PxTriangleMesh *mesh = mCooking->createTriangleMesh(desc, mPhysics->getPhysicsInsertionCallback());
				mTriangleMeshes.push_back(mesh);
				float scale = ranf() + 1.0f;
				physx::PxVec3 sc(scale, scale, scale);
				physx::PxTriangleMeshGeometry tmesh(mesh, physx::PxMeshScale(sc));
				physx::PxShape *s = physics->createShape(tmesh, *m);
				s->setName("@triangleShape");
				mShapes.push_back(s);
			}
		}
		{
			physx::PxMaterial *m = getRandomMaterial();
			physx::PxHeightFieldDesc desc;
			desc.samples.data = mHeightFieldData;
			desc.samples.stride = sizeof(uint32_t);
			desc.nbRows = HEIGHT_FIELD_SIZE;
			desc.nbColumns = HEIGHT_FIELD_SIZE;
			physx::PxHeightField *hf = mCooking->createHeightField(desc, mPhysics->getPhysicsInsertionCallback());
			mHeightFields.push_back(hf);
			float scale = ranf() + 1.0f;
			physx::PxHeightFieldGeometry hfg(hf, physx::PxMeshGeometryFlag::eDOUBLE_SIDED, scale, scale, scale);
			physx::PxShape *s = physics->createShape(hfg, *m);
			s->setName("@heightFieldShape");
			mShapes.push_back(s);

			
		}
		for (auto &i : mShapes)
		{
			physx::PxTransform t(physx::PxIdentity);
			t.p.x = ranf() * 10 - 5;
			t.p.z = ranf() * 10 - 5;
			physx::PxRigidStatic *rd = physx::PxCreateStatic(*mPhysics, t, *i);
			if (rd)
			{
				rd->setName("@rigidStatic");
				mScene->addActor(*rd);
				mRigidStatics.push_back(rd);
			}
		}

		for (auto &i : mShapes)
		{
			bool isValid = false;
			switch (i->getGeometryType())
			{
			case physx::PxGeometryType::eBOX:
			case physx::PxGeometryType::eSPHERE:
			case physx::PxGeometryType::eCONVEXMESH:
			case physx::PxGeometryType::eCAPSULE:
				isValid = true;
				break;
			}
			if (isValid)
			{
				physx::PxTransform t(physx::PxIdentity);
				t.p.x = ranf() * 10 - 5;
				t.p.z = ranf() * 10 - 5;
				t.p.y = ranf() * 4 + 1.0f;
				physx::PxRigidDynamic *rd = physx::PxCreateDynamic(*mPhysics, t, *i, 1.0f);
				if (rd)
				{
					rd->setName("@rigidDynamic");
					if ((rand() & 3) == 0)
					{
						uint32_t index = rand() % mShapes.size();
						physx::PxShape *s = mShapes[index];
						switch (s->getGeometryType())
						{
						case physx::PxGeometryType::eBOX:
						case physx::PxGeometryType::eSPHERE:
						case physx::PxGeometryType::eCONVEXMESH:
						case physx::PxGeometryType::eCAPSULE:
							rd->attachShape(*s);
							break;
						}
					}
					mScene->addActor(*rd);
					mRigidDynamics.push_back(rd);
				}
			}
		}

		mAggregate = mPhysics->createAggregate(32, true);
		for (auto &i : mShapes)
		{
			bool isValid = false;
			switch (i->getGeometryType())
			{
			case physx::PxGeometryType::eBOX:
			case physx::PxGeometryType::eSPHERE:
			case physx::PxGeometryType::eCONVEXMESH:
			case physx::PxGeometryType::eCAPSULE:
				isValid = true;
				break;
			}
			if (isValid)
			{
				physx::PxTransform t(physx::PxIdentity);
				t.p.x = ranf() * 10 - 5;
				t.p.z = ranf() * 10 - 5;
				t.p.y = ranf() * 4 + 1.0f;
				physx::PxRigidDynamic *rd = physx::PxCreateDynamic(*mPhysics, t, *i, 1.0f);
				if (rd)
				{
					rd->setName("@rigidDynamic");
					if ((rand() & 3) == 0)
					{
						uint32_t index = rand() % mShapes.size();
						physx::PxShape *s = mShapes[index];
						switch (s->getGeometryType())
						{
						case physx::PxGeometryType::eBOX:
						case physx::PxGeometryType::eSPHERE:
						case physx::PxGeometryType::eCONVEXMESH:
						case physx::PxGeometryType::eCAPSULE:
							rd->attachShape(*s);
							break;
						}
					}
					mAggregate->addActor(*rd);
					mRigidDynamics.push_back(rd);
				}
			}
		}
		{
			PxArticulation *a = mPhysics->createArticulation();
			physx::PxTransform linkPose(physx::PxIdentity);
			PxArticulationLink* link = a->createLink(nullptr, linkPose);
			physx::PxMaterial *m = getRandomMaterial();
			PxSphereGeometry sphere(1.0f);
			PxRigidActorExt::createExclusiveShape(*link, sphere, *m);
			PxRigidBodyExt::updateMassAndInertia(*link, 1.0f);
			{
				PxArticulationLink* childLink = a->createLink(link, linkPose);
				PxRigidActorExt::createExclusiveShape(*childLink, sphere, *m);
				PxRigidBodyExt::updateMassAndInertia(*childLink, 1.0f);
			}

			mAggregate->addArticulation(*a);
			mArticulations.push_back(a);
		}
		mScene->addAggregate(*mAggregate);
		{
			PxArticulation *a = mPhysics->createArticulation();
			mScene->addArticulation(*a);
			mArticulations.push_back(a);
		}

		createJoint(JT_FIXED);
		createJoint(JT_D6);
		createJoint(JT_SPHERICAL);
		createJoint(JT_DISTANCE);
		createJoint(JT_REVOLUTE);
		createJoint(JT_PRISMATIC);
	}


	physx::PxMaterial *getRandomMaterial(void) const 
	{
		uint32_t index = rand() % mMaterials.size();
		return mMaterials[index];
	}

	virtual ~SomeOfEverythingImpl(void)
	{
		for (auto &i : mArticulationLinks)
		{
			i->release();
		}
		for (auto &i : mArticulations)
		{
			i->release();
		}
		for (auto &i : mJoints)
		{
			i->release();
		}
		for (auto &i : mRigidStatics)
		{
			i->release();
		}
		for (auto &i : mRigidDynamics)
		{
			i->release();
		}
		if (mAggregate)
		{
			mAggregate->release();
		}
		for (auto &i : mMaterials)
		{
			i->release();
		}
		for (auto &i : mShapes)
		{
			i->release();
		}
		for (auto &i : mConvexMeshes)
		{
			i->release();
		}
		for (auto &i : mTriangleMeshes)
		{
			i->release();
		}
		for (auto &i : mHeightFields)
		{
			i->release();
		}
		delete[]mMaterialIndices;
	}

	virtual void release(void) final
	{
		delete this;
	}

	PxRigidDynamic *findActor(PxRigidDynamic *exclude)
	{
		PxRigidDynamic *ret = nullptr;

		for (;;)
		{
			uint32_t index = rand() % mRigidDynamics.size();
			ret = mRigidDynamics[index];
			if (ret != exclude)
			{
				break;
			}
		}
		return ret;
	}

	void createJoint(JointType type)
	{
		PxRigidDynamic *a1 = findActor(nullptr);
		PxRigidDynamic *a2 = findActor(a1);

		physx::PxTransform t = a1->getGlobalPose();

		float worldPos[3];
		float worldOrientation[4];
		worldPos[0] = t.p.x;
		worldPos[1] = t.p.y;
		worldPos[2] = t.p.z;
		worldOrientation[0] = t.q.x;
		worldOrientation[1] = t.q.y;
		worldOrientation[2] = t.q.z;
		worldOrientation[3] = t.q.w;

		createConstraint(a1, a2, worldPos, worldOrientation, type, ranf(), 5, 15, 15);
	}

	// Prismatic Joint
	PxJoint* createPrismaticJoint(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1)
	{
		PxPrismaticJoint* j = PxPrismaticJointCreate(*mPhysics, a0, t0, a1, t1);
		j->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true); // enable visualization!!
		j->setConstraintFlag(PxConstraintFlag::eDISABLE_PREPROCESSING, true);
		j->setName("@prismaticJoint");
		return j;
	}

	// Distance Joint
	PxJoint* createDistanceJoint(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1)
	{
		PxDistanceJoint* j = PxDistanceJointCreate(*mPhysics, a0, t0, a1, t1);
		j->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true); // enable visualization!!
		j->setConstraintFlag(PxConstraintFlag::eDISABLE_PREPROCESSING, true);
		j->setName("@prismaticJoint");
		return j;
	}

	// Revolute Joint
	PxJoint* createRevoluteJoint(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1)
	{
		PxRevoluteJoint* j = PxRevoluteJointCreate(*mPhysics, a0, t0, a1, t1);
		j->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true); // enable visualization!!
		j->setConstraintFlag(PxConstraintFlag::eDISABLE_PREPROCESSING, true);
		j->setName("@prismaticJoint");
		return j;
	}

	// fixed joint
	PxJoint* createFixedJoint(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1)
	{
		PxFixedJoint* j = PxFixedJointCreate(*mPhysics, a0, t0, a1, t1);
		j->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true); // enable visualization!!
		j->setConstraintFlag(PxConstraintFlag::eDISABLE_PREPROCESSING, true);
		j->setName("@fixedJoint");
		return j;
	}

	// spherical joint limited to an angle of at most pi/4 radians (45 degrees)
	PxJoint* createLimitedSpherical(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1, uint32_t swing1Limit, uint32_t swing2Limit)
	{
		PxSphericalJoint* j = PxSphericalJointCreate(*mPhysics, a0, t0, a1, t1);

		float lrange1 = (PxPi * 2) * (float(swing1Limit) / 360.0f);
		float lrange2 = (PxPi * 2) * (float(swing2Limit) / 360.0f);

		j->setLimitCone(PxJointLimitCone(lrange1, lrange2));
		j->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, true);
		j->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true); // enable visualization!!
		j->setConstraintFlag(PxConstraintFlag::eDISABLE_PREPROCESSING, true);
		j->setName("@sphericalJoint");
		return j;
	}

	// D6 joint with a spring maintaining its position
	PxJoint* createHingeJoint(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1, uint32_t limitRangeDegrees)
	{
		PxD6Joint* j = PxD6JointCreate(*mPhysics, a0, t0, a1, t1);
		j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLOCKED);
		j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLOCKED);
		j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
		j->setConstraintFlag(PxConstraintFlag::eDISABLE_PREPROCESSING, true);

		float lrange = (PxPi * 2) * (float(limitRangeDegrees) / 360.0f);

		PxJointAngularLimitPair limit(-lrange, lrange);
		j->setTwistLimit(limit);
		j->setDrive(PxD6Drive::eSLERP, PxD6JointDrive(0, 1000, FLT_MAX, true));
		j->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true); // enable visualization!!
		j->setName("@hingJoint");
		return j;
	}

	// Creates a fixed constraint between these two bodies
	virtual void createConstraint(
		PxRigidDynamic *a1,PxRigidDynamic *a2, const float worldPos[3],					// World position of the constraint location
		const float worldOrientation[4],
		JointType type,			// Type of constraint to use
		float	distanceLimit,
		uint32_t twistLimit,			// Twist limit in degrees (if used)
		uint32_t swing1Limit,			// Swing 1 limit in degrees (if used)
		uint32_t swing2Limit) final		// Swing 2 limit in degrees (if used)
	{
		{
	
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
			case JT_FIXED:
				joint = createFixedJoint(a1, other1, a2, other2);
				break;
			case JT_D6:
				joint = createHingeJoint(a1, other1, a2, other2, twistLimit);
				break;
			case JT_SPHERICAL:
				joint = createLimitedSpherical(a1, other1, a2, other2, swing1Limit, swing2Limit);
				break;
			case JT_DISTANCE:
				joint = createDistanceJoint(a1, other1, a2, other2);
				break;
			case JT_REVOLUTE:
				joint = createRevoluteJoint(a1, other1, a2, other2);
				break;
			case JT_PRISMATIC:
				joint = createPrismaticJoint(a1, other1, a2, other2);
				break;
			}
			if (joint)
			{
				mJoints.push_back(joint);
			}
		}
	}


	physx::PxScene		*mScene{ nullptr };
	physx::PxPhysics	*mPhysics{ nullptr };
	physx::PxCooking	*mCooking{ nullptr };
	MaterialVector		mMaterials;
	ShapeVector			mShapes;
	ConvexMeshVector	mConvexMeshes;
	TriangleMeshVector	mTriangleMeshes;
	RigidStaticVector	mRigidStatics;
	RigidDynamicVector	mRigidDynamics;
	HeightFieldVector	mHeightFields;
	JointVector			mJoints;
	uint16_t			*mMaterialIndices{ nullptr };
	uint32_t			mHeightFieldData[HEIGHT_FIELD_SIZE*HEIGHT_FIELD_SIZE];
	PxAggregate			*mAggregate{ nullptr };
	ArticulationVector	mArticulations;
	ArticulationLinkVector	mArticulationLinks;
};

SomeOfEverything *SomeOfEverything::create(physx::PxPhysics *physics, physx::PxCooking *cooking,physx::PxScene *scene)
{
	SomeOfEverythingImpl *ret = new SomeOfEverythingImpl(physics, cooking,scene);
	return static_cast<SomeOfEverything *>(ret);
}


}
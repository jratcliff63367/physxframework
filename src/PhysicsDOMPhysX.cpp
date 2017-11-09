#include "PhysicsDOMPhysX.h"
#include "PhysicsDOM.h"
#include "PxPhysicsAPI.h"
#include "NvBounds3.h"
#include <vector>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#pragma warning(disable:4100)

namespace PHYSICS_DOM_PHYSX
{

	typedef std::vector< physx::PxScene *> SceneVector;

	class PhysicsDOMPhysXImpl : public PhysicsDOMPhysX
	{
	public:
		PhysicsDOMPhysXImpl(physx::PxPhysics *p, physx::PxCooking *c) : mPhysics(p), mCooking(c)
		{

		}

		virtual bool loadPhysicsDOM(const PHYSICS_DOM::PhysicsDOM &p) final
		{
			bool ret = false;

			for (auto &i : p.scenes)
			{
				const PHYSICS_DOM::Scene &s = *i;
				physx::PxTolerancesScale ts;
				ts.length = s.physx_tolerancesScale.physx_length;
				ts.speed = s.physx_tolerancesScale.physx_speed;
				physx::PxSceneDesc desc(ts);
				desc.gravity = physx::PxVec3(s.gravity.x, s.gravity.y, s.gravity.z);
				desc.simulationEventCallback = nullptr; // TODO
				desc.contactModifyCallback = nullptr; // TODO
				desc.ccdContactModifyCallback = nullptr; // TODO
				desc.filterShaderData = nullptr; // TODO
				desc.filterShaderDataSize = 0; // TODO
				desc.filterShader = nullptr; // TODO
				desc.filterCallback = nullptr; // TODO
				switch ( s.physx_broadPhaseType )
				{
					case PHYSICS_DOM::BPT_SAP:
						desc.broadPhaseType = physx::PxBroadPhaseType::eSAP;
						break;
					case PHYSICS_DOM::BPT_MBP:
						desc.broadPhaseType = physx::PxBroadPhaseType::eMBP;
						break;
					case PHYSICS_DOM::BPT_GPU:
						desc.broadPhaseType = physx::PxBroadPhaseType::eGPU;
						break;
				}

				desc.broadPhaseCallback = nullptr; // TODO

				desc.limits.maxNbActors = s.physx_sceneLimits.physx_maxNbActors;				//!< Expected maximum number of actors
				desc.limits.maxNbBodies = s.physx_sceneLimits.physx_maxNbBodies;				//!< Expected maximum number of dynamic rigid bodies
				desc.limits.maxNbStaticShapes = s.physx_sceneLimits.physx_maxNbStaticShapes;			//!< Expected maximum number of static shapes
				desc.limits.maxNbDynamicShapes = s.physx_sceneLimits.physx_maxNbDynamicShapes;			//!< Expected maximum number of dynamic shapes
				desc.limits.maxNbAggregates = s.physx_sceneLimits.physx_maxNbAggregates;			//!< Expected maximum number of aggregates
				desc.limits.maxNbConstraints = s.physx_sceneLimits.physx_maxNbConstraints;			//!< Expected maximum number of constraint shaders
				desc.limits.maxNbRegions = s.physx_sceneLimits.physx_maxNbRegions;				//!< Expected maximum number of broad-phase regions
				desc.limits.maxNbBroadPhaseOverlaps = s.physx_sceneLimits.physx_maxNbBroadPhaseOverlaps;	//!< Expected maximum number of broad-phase overlaps

				switch (s.physx_frictionType)
				{
					case PHYSICS_DOM::FT_PATCH:
						desc.frictionType = physx::PxFrictionType::ePATCH;
						break;
					case PHYSICS_DOM::FT_ONE_DIRECTIONAL:
						desc.frictionType = physx::PxFrictionType::eONE_DIRECTIONAL;
						break;
					case PHYSICS_DOM::FT_TWO_DIRECTIONAL:
						desc.frictionType = physx::PxFrictionType::eTWO_DIRECTIONAL;
						break;
				}

				desc.bounceThresholdVelocity = s.physx_bounceThresholdVelocity;
				desc.ccdMaxSeparation = s.physx_ccdMaxSeparation;
				desc.solverOffsetSlop = s.physx_solverOffsetSlop;
				desc.flags = (physx::PxSceneFlags)(0);
				if (s.physx_enableActiveActors)
				{
					desc.flags.set(physx::PxSceneFlag::eENABLE_ACTIVE_ACTORS);
				}
				if (s.physx_enableActiveTransforms)
				{
					desc.flags.set(physx::PxSceneFlag::eENABLE_ACTIVETRANSFORMS);
				}
				physx::PxScene *scene = mPhysics->createScene(desc);
				mScenes.push_back(scene);

			}

			return ret;
		}

		virtual void reset(void) final
		{

		}

		virtual void release(void) final
		{
			delete this;
		}

		SceneVector			mScenes;
		physx::PxPhysics	*mPhysics{ nullptr };
		physx::PxCooking	*mCooking{ nullptr };
	};

	PhysicsDOMPhysX *PhysicsDOMPhysX::create(physx::PxPhysics *p, physx::PxCooking *c)
	{
		PhysicsDOMPhysXImpl *ret = new PhysicsDOMPhysXImpl(p, c);
		return static_cast<PhysicsDOMPhysX *>(ret);
	}
}

#include "PhysicsDOMPhysX.h"
#include "PhysicsDOM.h"
#include "PxPhysicsAPI.h"
#include "NvBounds3.h"
#include "StringHelper.h"
#include <unordered_map>
#include <string>
#include <string.h>
#include <vector>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#pragma warning(disable:4100)

namespace PHYSICS_DOM_PHYSX
{

	typedef std::unordered_map< std::string,const PHYSICS_DOM::Node * > NodeMap;
	typedef std::vector< physx::PxScene *> SceneVector;

	// Map node id to physx materials
	typedef std::unordered_map< std::string, physx::PxMaterial *> MaterialMap;

	class PhysicsDOMPhysXImpl : public PhysicsDOMPhysX
	{
	public:
		PhysicsDOMPhysXImpl(physx::PxPhysics *p, physx::PxCooking *c) : mPhysics(p), mCooking(c)
		{

		}

		virtual ~PhysicsDOMPhysXImpl(void)
		{
			for (auto &i : mScenes)
			{
				i->release();
			}
			if (mDefaultMaterial)
			{
				mDefaultMaterial->release();
			}
			for (auto &i : mMaterials)
			{
				i.second->release();
			}
		}

		void buildNodeMap(const PHYSICS_DOM::Node *node)
		{
			mNodeMap[node->id] = node;
			switch (node->type)
			{
				case PHYSICS_DOM::NT_COLLECTION:
				case PHYSICS_DOM::NT_SCENE:
					// Recursively add all nodes from this collection node
					{
						auto c = static_cast<const PHYSICS_DOM::Collection *>(node);
						for (auto &i : c->nodes)
						{
							buildNodeMap(i);
						}
					}
					break;
			}
		}

		virtual bool loadPhysicsDOM(const PHYSICS_DOM::PhysicsDOM &p,physx::PxScene *scene) final
		{
			bool ret = false;

			if (mDefaultMaterial == nullptr)
			{
				mDefaultMaterial = mPhysics->createMaterial(0.5f, 0.5f, 0.0f);
			}
			// Build the NodeMap
			{
				// Add all of the collections recursively
				for (const auto &i : p.collections)
				{
					buildNodeMap(i);
				}
				// Add all of the scenes recursively
				for (const auto &i : p.scenes)
				{
					buildNodeMap(i);
				}
			}

			for (auto &i : p.scenes)
			{
				const PHYSICS_DOM::Scene &s = *i;

				if (scene == nullptr)
				{
					physx::PxTolerancesScale ts;
					ts.length = s.physx_sceneDesc.tolerancesScale.length;
					ts.speed = s.physx_sceneDesc.tolerancesScale.speed;
					physx::PxSceneDesc desc(ts);
					desc.gravity = physx::PxVec3(s.gravity.x, s.gravity.y, s.gravity.z);
					desc.simulationEventCallback = nullptr; // TODO
					desc.contactModifyCallback = nullptr; // TODO
					desc.ccdContactModifyCallback = nullptr; // TODO
					desc.filterShaderData = nullptr; // TODO
					desc.filterShaderDataSize = 0; // TODO
					desc.filterShader = nullptr; // TODO
					desc.filterCallback = nullptr; // TODO
					switch (s.physx_sceneDesc.broadPhaseType)
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

					desc.limits.maxNbActors = s.physx_sceneDesc.sceneLimits.maxNbActors;				//!< Expected maximum number of actors
					desc.limits.maxNbBodies = s.physx_sceneDesc.sceneLimits.maxNbBodies;				//!< Expected maximum number of dynamic rigid bodies
					desc.limits.maxNbStaticShapes = s.physx_sceneDesc.sceneLimits.maxNbStaticShapes;			//!< Expected maximum number of static shapes
					desc.limits.maxNbDynamicShapes = s.physx_sceneDesc.sceneLimits.maxNbDynamicShapes;			//!< Expected maximum number of dynamic shapes
					desc.limits.maxNbAggregates = s.physx_sceneDesc.sceneLimits.maxNbAggregates;			//!< Expected maximum number of aggregates
					desc.limits.maxNbConstraints = s.physx_sceneDesc.sceneLimits.maxNbConstraints;			//!< Expected maximum number of constraint shaders
					desc.limits.maxNbRegions = s.physx_sceneDesc.sceneLimits.maxNbRegions;				//!< Expected maximum number of broad-phase regions
					desc.limits.maxNbBroadPhaseOverlaps = s.physx_sceneDesc.sceneLimits.maxNbBroadPhaseOverlaps;	//!< Expected maximum number of broad-phase overlaps

					switch (s.physx_sceneDesc.frictionType)
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

					desc.bounceThresholdVelocity = s.physx_sceneDesc.bounceThresholdVelocity;
					desc.ccdMaxSeparation = s.physx_sceneDesc.ccdMaxSeparation;
					desc.solverOffsetSlop = s.physx_sceneDesc.solverOffsetSlop;
					desc.flags = (physx::PxSceneFlags)(0);
					if (s.physx_sceneDesc.enableActiveActors)
					{
						desc.flags.set(physx::PxSceneFlag::eENABLE_ACTIVE_ACTORS);
					}
					if (s.physx_sceneDesc.enableActiveTransforms)
					{
						desc.flags.set(physx::PxSceneFlag::eENABLE_ACTIVETRANSFORMS);
					}
					scene = mPhysics->createScene(desc);
					mScenes.push_back(scene);
				}
				mActiveScene = scene;
				PHYSICS_DOM::Pose pose;
				PHYSICS_DOM::Vec3 scale(1, 1, 1);
				for (auto &j : s.nodes)
				{
					processNode(j, pose, scale);
				}

			}

			return ret;
		}

		void reportWarning(const char *fmt,...)
		{
			va_list         args;
			char            buffer[4096];
			va_start(args, fmt);
			STRING_HELPER::stringFormatV(buffer, sizeof(buffer), fmt, args);
			va_end(args);
			printf("[PHYSICS_DOM::Warning]%s\r\n", buffer);
		}

		void processNode(const PHYSICS_DOM::Node *n,		// Node to process
						 const PHYSICS_DOM::Pose &pose,		// Parent relative pose
						const PHYSICS_DOM::Vec3 &scale)		// Parent relative scale
		{
			if (n == nullptr)
			{
				reportWarning("Encountered a null node pointer!!");
				return;
			}
			switch (n->type)
			{
				case PHYSICS_DOM::NT_PHYSICS_MATERIAL: 				// A physics material
					{
						auto m = static_cast<const PHYSICS_DOM::PhysicsMaterial *>(n);
						physx::PxMaterial *pmat = mPhysics->createMaterial(m->staticFriction, m->dynamicFriction, m->restitution);
						mMaterials[n->id] = pmat;
					}
					break;
				case PHYSICS_DOM::NT_GEOMETRY_INSTANCE:  				// Defines an instance of a geometry
					reportWarning("Node type not yet implemented");
					break;
				case PHYSICS_DOM::NT_TRIANGLEMESH: 					// Defines the contents of a triangle mesh
					reportWarning("Node type not yet implemented");
					break;
				case PHYSICS_DOM::NT_CONVEXHULL:						// Defines the contents of a convex hull
					reportWarning("Node type not yet implemented");
					break;
				case PHYSICS_DOM::NT_HEIGHTFIELD: 					// Defines the contents of a heightfield
					reportWarning("Node type not yet implemented");
					break;
				case PHYSICS_DOM::NT_RIGID_BODY:   					// Common properties of both static and dynamic rigid bodies
					reportWarning("Node type not yet implemented");
					break;
				case PHYSICS_DOM::NT_RIGID_DYNAMIC:  					// A dynamic rigid body
				case PHYSICS_DOM::NT_RIGID_STATIC: 					// A static rigid body
					{
						auto rb = static_cast<const PHYSICS_DOM::RigidBody *>(n);
						physx::PxRigidStatic *pstatic = nullptr;
						physx::PxRigidDynamic *pdynamic = nullptr;
						physx::PxRigidActor *ractor = nullptr;
						// TODO;; concatenate parent pose!
						physx::PxTransform rpose = getTransform(rb->globalPose);
						if (n->type == PHYSICS_DOM::NT_RIGID_DYNAMIC)
						{
							pdynamic = mPhysics->createRigidDynamic(rpose);
							ractor = static_cast<physx::PxRigidActor *>(pdynamic);
						}
						else
						{
							pstatic = mPhysics->createRigidStatic(rpose);
							ractor = static_cast<physx::PxRigidActor *>(pstatic);
						}
						// let's create the shapes and add them...
						for (auto &j : rb->geometryInstances)
						{
							std::vector< physx::PxMaterial *> materials;
							if (j->materials.empty())
							{
								materials.push_back(mDefaultMaterial);
							}
							else
							{
								for (auto &k : j->materials)
								{
									materials.push_back(locateMaterial(k));
								}
							}
							physx::PxBoxGeometry box;
							physx::PxGeometry *geometry = nullptr;
							switch (j->geometry->type )
							{
								case PHYSICS_DOM::GT_BOX_GEOMETRY:
									{
										auto boxGeom = static_cast<const PHYSICS_DOM::BoxGeometry *>(j->geometry);
										geometry = static_cast<physx::PxGeometry *>(&box);
										box.halfExtents = physx::PxVec3(boxGeom->dimensions.x*0.5f, boxGeom->dimensions.y*0.5f, boxGeom->dimensions.z*0.5f);
									}
								break;
								default:
									reportWarning("Not yet implemented!");
									break;
							}
							if (geometry)
							{
								physx::PxShape *shape = ractor->createShape(*geometry, &materials[0], uint16_t(materials.size()));
								if (shape)
								{
									shape->setLocalPose(getTransform(j->localPose));
								}
							}
						}
						mActiveScene->addActor(*ractor);
					}
					break;
				case PHYSICS_DOM::NT_BODY_PAIR_FILTERS:  				// A node representing a collection of body pair filters
					reportWarning("Node type not yet implemented");
					break;
				case PHYSICS_DOM::NT_JOINT:  							// Base class for a joint
					reportWarning("Node type not yet implemented");
					break;
				case PHYSICS_DOM::NT_FIXED_JOINT:						// A fixed joint
					reportWarning("Node type not yet implemented");
					break;
				case PHYSICS_DOM::NT_SPHERICAL_JOINT:				// A spherical joint
					reportWarning("Node type not yet implemented");
					break;
				case PHYSICS_DOM::NT_REVOLUTE_JOINT:   				// A revolute joint
					reportWarning("Node type not yet implemented");
					break;
				case PHYSICS_DOM::NT_PRISMATIC_JOINT:					// A prismatic joint
					reportWarning("Node type not yet implemented");
					break;
				case PHYSICS_DOM::NT_DISTANCE_JOINT:   				// A distance joint
					reportWarning("Node type not yet implemented");
					break;
				case PHYSICS_DOM::NT_D6_JOINT: 						// A six degree of freedom joint
					reportWarning("Node type not yet implemented");
					break;
				case PHYSICS_DOM::NT_INSTANCE_COLLECTION:				// Instantiates a collection of nodes
					{
						auto i = static_cast<const PHYSICS_DOM::InstanceCollection *>(n);
						auto j = mNodeMap.find(i->collection);
						if (j == mNodeMap.end())
						{
							reportWarning("Failed to locate Node(%s) to instance.", i->collection.c_str());
						}
						else
						{
							const PHYSICS_DOM::Node *fnode = (*j).second;
							if (fnode->type == PHYSICS_DOM::NT_COLLECTION ||
								fnode->type == PHYSICS_DOM::NT_INSTANCE_COLLECTION )
							{
								// TODO!! Combine parent relative pose!!
								processNode(fnode, i->pose, i->scale);
							}
							else
							{
								reportWarning("Node(%s) is not of type Collection or InstanceCollection.", i->collection.c_str());
							}
						}
					}
					break;
				case PHYSICS_DOM::NT_COLLECTION:   					// Defines a collection of nodes
				case PHYSICS_DOM::NT_SCENE:
					{
						auto i = static_cast<const PHYSICS_DOM::Collection *>(n);
						for (auto &j : i->nodes)
						{
							processNode(j,pose,scale);
						}
					}
					break;
			}
		}


		virtual void reset(void) final
		{

		}

		virtual void release(void) final
		{
			delete this;
		}

		physx::PxMaterial *locateMaterial(const std::string &id)
		{
			physx::PxMaterial *ret = nullptr;
			auto found = mMaterials.find(id);
			if (found == mMaterials.end())
			{
				ret = mDefaultMaterial;
			}
			else
			{
				ret = found->second;
			}
			return ret;
		}

		physx::PxTransform getTransform(const PHYSICS_DOM::Pose &pose)
		{
			physx::PxTransform rpose;
			rpose.p = physx::PxVec3(pose.p.x, pose.p.x, pose.p.z);
			rpose.q = physx::PxQuat(pose.q.x, pose.q.y, pose.q.z, pose.q.w);
			return rpose;
		}

		physx::PxScene		*mActiveScene{ nullptr };

		SceneVector			mScenes;
		physx::PxPhysics	*mPhysics{ nullptr };
		physx::PxCooking	*mCooking{ nullptr };
		NodeMap				mNodeMap;	// map of Id's to nodes
		physx::PxMaterial	*mDefaultMaterial{ nullptr };
		MaterialMap			mMaterials;	// Materials created
	};

	PhysicsDOMPhysX *PhysicsDOMPhysX::create(physx::PxPhysics *p, physx::PxCooking *c)
	{
		PhysicsDOMPhysXImpl *ret = new PhysicsDOMPhysXImpl(p, c);
		return static_cast<PhysicsDOMPhysX *>(ret);
	}
}
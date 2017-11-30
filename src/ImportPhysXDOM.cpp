#include "ImportPhysXDOM.h"
#include "PhysicsDOMDef.h"
#include "FastXml.h"
#include "StringHelper.h"
#include <unordered_map>
#include <string>
#include <vector>

#pragma warning(disable:4100)

#define USE_PHYSX_SETTINGS 0

namespace IMPORT_PHYSX_DOM
{
	// 170 unique element types
	enum ElementType {
		ET_ActorFlags,
		ET_Actors,
		ET_AngularDamping,
		ET_AngularVelocity,
		ET_BounceThreshold,
		ET_BreakForce,
		ET_CMassLocalPose,
		ET_ChildPose,
		ET_ColumnScale,
		ET_ConstraintFlags,
		ET_ContactDistance,
		ET_ContactOffset,
		ET_ContactReportThreshold,
		ET_ConvexEdgeThreshold,
		ET_ConvexMesh,
		ET_Damping,
		ET_DistanceJointFlags,
		ET_DominanceGroup,
		ET_Drive,
		ET_DriveForceLimit,
		ET_DriveGearRatio,
		ET_DrivePosition,
		ET_DriveType,
		ET_DriveVelocity,
		ET_DynamicFriction,
		ET_ExternalCompliance,
		ET_ExternalDriveIterations,
		ET_Flags,
		ET_ForceLimit,
		ET_Format,
		ET_FrictionCombineMode,
		ET_Geometry,
		ET_GlobalPose,
		ET_HalfExtents,
		ET_HalfHeight,
		ET_HeightField,
		ET_HeightFieldFlags,
		ET_HeightScale,
		ET_Id,
		ET_InternalCompliance,
		ET_InternalDriveIterations,
		ET_InvInertiaScale0,
		ET_InvInertiaScale1,
		ET_InvMassScale0,
		ET_InvMassScale1,
		ET_Joint,
		ET_Length,
		ET_Limit,
		ET_LimitCone,
		ET_LinearDamping,
		ET_LinearLimit,
		ET_LinearVelocity,
		ET_Links,
		ET_LocalPose,
		ET_Lower,
		ET_Mass,
		ET_MassSpaceInertiaTensor,
		ET_Materials,
		ET_MaxAngularVelocity,
		ET_MaxContactImpulse,
		ET_MaxDepenetrationVelocity,
		ET_MaxDistance,
		ET_MaxMargin,
		ET_MaxNbActors,
		ET_MaxProjectionIterations,
		ET_MinCCDAdvanceCoefficient,
		ET_MinDistance,
		ET_Motion,
		ET_Name,
		ET_NbColumns,
		ET_NbRows,
		ET_NumActors,
		ET_OwnerClient,
		ET_Parent,
		ET_ParentPose,
		ET_PhysX30Collection,
		ET_Points,
		ET_ProjectionAngularTolerance,
		ET_ProjectionLinearTolerance,
		ET_PxActorRef,
		ET_PxAggregate,
		ET_PxArticulation,
		ET_PxArticulationLink,
		ET_PxArticulationRef,
		ET_PxBVH33TriangleMesh,
		ET_PxBoxGeometry,
		ET_PxCapsuleGeometry,
		ET_PxConvexMesh,
		ET_PxConvexMeshGeometry,
		ET_PxD6Joint,
		ET_PxDistanceJoint,
		ET_PxFixedJoint,
		ET_PxHeightField,
		ET_PxHeightFieldGeometry,
		ET_PxMaterial,
		ET_PxMaterialRef,
		ET_PxPlaneGeometry,
		ET_PxPrismaticJoint,
		ET_PxRevoluteJoint,
		ET_PxRigidDynamic,
		ET_PxRigidStatic,
		ET_PxShape,
		ET_PxShapeRef,
		ET_PxSphereGeometry,
		ET_PxSphericalJoint,
		ET_PxTriangleMeshGeometry,
		ET_QueryFilterData,
		ET_Radius,
		ET_RestOffset,
		ET_Restitution,
		ET_RestitutionCombineMode,
		ET_Rotation,
		ET_RowScale,
		ET_Scale,
		ET_SelfCollision,
		ET_SeparationTolerance,
		ET_Shapes,
		ET_SimulationFilterData,
		ET_SleepThreshold,
		ET_SolverIterationCounts,
		ET_Speed,
		ET_SphericalJointFlags,
		ET_StabilizationThreshold,
		ET_StaticFriction,
		ET_Stiffness,
		ET_SwingLimit,
		ET_SwingLimitContactDistance,
		ET_SwingLimitEnabled,
		ET_TangentialDamping,
		ET_TangentialStiffness,
		ET_TargetOrientation,
		ET_TargetVelocity,
		ET_Thickness,
		ET_Tolerance,
		ET_TriangleMesh,
		ET_Triangles,
		ET_TwistLimit,
		ET_TwistLimitContactDistance,
		ET_TwistLimitEnabled,
		ET_UpVector,
		ET_Upper,
		ET_Value,
		ET_WakeCounter,
		ET_YAngle,
		ET_ZAngle,
		ET_actor0,
		ET_actor1,
		ET_angular,
		ET_eACTOR0,
		ET_eACTOR1,
		ET_eSLERP,
		ET_eSWING,
		ET_eSWING1,
		ET_eSWING2,
		ET_eTWIST,
		ET_eX,
		ET_eY,
		ET_eZ,
		ET_force,
		ET_linear,
		ET_lower,
		ET_materialIndices,
		ET_minPositionIters,
		ET_minVelocityIters,
		ET_points,
		ET_samples,
		ET_torque,
		ET_upper,
		ET_yLimit,
		ET_zLimit,
		ET_LAST
	};

	// 1 unique attribute types
	enum AttributeType {
		AT_version,
		AT_LAST
	};

	struct ElementStruct
	{
		ElementType 	mType;
		const char		*mName;
	};

	static ElementStruct gElements[ET_LAST] =
	{
		ET_ActorFlags, "ActorFlags",
		ET_Actors, "Actors",
		ET_AngularDamping, "AngularDamping",
		ET_AngularVelocity, "AngularVelocity",
		ET_BounceThreshold, "BounceThreshold",
		ET_BreakForce, "BreakForce",
		ET_CMassLocalPose, "CMassLocalPose",
		ET_ChildPose, "ChildPose",
		ET_ColumnScale, "ColumnScale",
		ET_ConstraintFlags, "ConstraintFlags",
		ET_ContactDistance, "ContactDistance",
		ET_ContactOffset, "ContactOffset",
		ET_ContactReportThreshold, "ContactReportThreshold",
		ET_ConvexEdgeThreshold, "ConvexEdgeThreshold",
		ET_ConvexMesh, "ConvexMesh",
		ET_Damping, "Damping",
		ET_DistanceJointFlags, "DistanceJointFlags",
		ET_DominanceGroup, "DominanceGroup",
		ET_Drive, "Drive",
		ET_DriveForceLimit, "DriveForceLimit",
		ET_DriveGearRatio, "DriveGearRatio",
		ET_DrivePosition, "DrivePosition",
		ET_DriveType, "DriveType",
		ET_DriveVelocity, "DriveVelocity",
		ET_DynamicFriction, "DynamicFriction",
		ET_ExternalCompliance, "ExternalCompliance",
		ET_ExternalDriveIterations, "ExternalDriveIterations",
		ET_Flags, "Flags",
		ET_ForceLimit, "ForceLimit",
		ET_Format, "Format",
		ET_FrictionCombineMode, "FrictionCombineMode",
		ET_Geometry, "Geometry",
		ET_GlobalPose, "GlobalPose",
		ET_HalfExtents, "HalfExtents",
		ET_HalfHeight, "HalfHeight",
		ET_HeightField, "HeightField",
		ET_HeightFieldFlags, "HeightFieldFlags",
		ET_HeightScale, "HeightScale",
		ET_Id, "Id",
		ET_InternalCompliance, "InternalCompliance",
		ET_InternalDriveIterations, "InternalDriveIterations",
		ET_InvInertiaScale0, "InvInertiaScale0",
		ET_InvInertiaScale1, "InvInertiaScale1",
		ET_InvMassScale0, "InvMassScale0",
		ET_InvMassScale1, "InvMassScale1",
		ET_Joint, "Joint",
		ET_Length, "Length",
		ET_Limit, "Limit",
		ET_LimitCone, "LimitCone",
		ET_LinearDamping, "LinearDamping",
		ET_LinearLimit, "LinearLimit",
		ET_LinearVelocity, "LinearVelocity",
		ET_Links, "Links",
		ET_LocalPose, "LocalPose",
		ET_Lower, "Lower",
		ET_Mass, "Mass",
		ET_MassSpaceInertiaTensor, "MassSpaceInertiaTensor",
		ET_Materials, "Materials",
		ET_MaxAngularVelocity, "MaxAngularVelocity",
		ET_MaxContactImpulse, "MaxContactImpulse",
		ET_MaxDepenetrationVelocity, "MaxDepenetrationVelocity",
		ET_MaxDistance, "MaxDistance",
		ET_MaxMargin, "MaxMargin",
		ET_MaxNbActors, "MaxNbActors",
		ET_MaxProjectionIterations, "MaxProjectionIterations",
		ET_MinCCDAdvanceCoefficient, "MinCCDAdvanceCoefficient",
		ET_MinDistance, "MinDistance",
		ET_Motion, "Motion",
		ET_Name, "Name",
		ET_NbColumns, "NbColumns",
		ET_NbRows, "NbRows",
		ET_NumActors, "NumActors",
		ET_OwnerClient, "OwnerClient",
		ET_Parent, "Parent",
		ET_ParentPose, "ParentPose",
		ET_PhysX30Collection, "PhysX30Collection",
		ET_Points, "Points",
		ET_ProjectionAngularTolerance, "ProjectionAngularTolerance",
		ET_ProjectionLinearTolerance, "ProjectionLinearTolerance",
		ET_PxActorRef, "PxActorRef",
		ET_PxAggregate, "PxAggregate",
		ET_PxArticulation, "PxArticulation",
		ET_PxArticulationLink, "PxArticulationLink",
		ET_PxArticulationRef, "PxArticulationRef",
		ET_PxBVH33TriangleMesh, "PxBVH33TriangleMesh",
		ET_PxBoxGeometry, "PxBoxGeometry",
		ET_PxCapsuleGeometry, "PxCapsuleGeometry",
		ET_PxConvexMesh, "PxConvexMesh",
		ET_PxConvexMeshGeometry, "PxConvexMeshGeometry",
		ET_PxD6Joint, "PxD6Joint",
		ET_PxDistanceJoint, "PxDistanceJoint",
		ET_PxFixedJoint, "PxFixedJoint",
		ET_PxHeightField, "PxHeightField",
		ET_PxHeightFieldGeometry, "PxHeightFieldGeometry",
		ET_PxMaterial, "PxMaterial",
		ET_PxMaterialRef, "PxMaterialRef",
		ET_PxPlaneGeometry, "PxPlaneGeometry",
		ET_PxPrismaticJoint, "PxPrismaticJoint",
		ET_PxRevoluteJoint, "PxRevoluteJoint",
		ET_PxRigidDynamic, "PxRigidDynamic",
		ET_PxRigidStatic, "PxRigidStatic",
		ET_PxShape, "PxShape",
		ET_PxShapeRef, "PxShapeRef",
		ET_PxSphereGeometry, "PxSphereGeometry",
		ET_PxSphericalJoint, "PxSphericalJoint",
		ET_PxTriangleMeshGeometry, "PxTriangleMeshGeometry",
		ET_QueryFilterData, "QueryFilterData",
		ET_Radius, "Radius",
		ET_RestOffset, "RestOffset",
		ET_Restitution, "Restitution",
		ET_RestitutionCombineMode, "RestitutionCombineMode",
		ET_Rotation, "Rotation",
		ET_RowScale, "RowScale",
		ET_Scale, "Scale",
		ET_SelfCollision, "SelfCollision",
		ET_SeparationTolerance, "SeparationTolerance",
		ET_Shapes, "Shapes",
		ET_SimulationFilterData, "SimulationFilterData",
		ET_SleepThreshold, "SleepThreshold",
		ET_SolverIterationCounts, "SolverIterationCounts",
		ET_Speed, "Speed",
		ET_SphericalJointFlags, "SphericalJointFlags",
		ET_StabilizationThreshold, "StabilizationThreshold",
		ET_StaticFriction, "StaticFriction",
		ET_Stiffness, "Stiffness",
		ET_SwingLimit, "SwingLimit",
		ET_SwingLimitContactDistance, "SwingLimitContactDistance",
		ET_SwingLimitEnabled, "SwingLimitEnabled",
		ET_TangentialDamping, "TangentialDamping",
		ET_TangentialStiffness, "TangentialStiffness",
		ET_TargetOrientation, "TargetOrientation",
		ET_TargetVelocity, "TargetVelocity",
		ET_Thickness, "Thickness",
		ET_Tolerance, "Tolerance",
		ET_TriangleMesh, "TriangleMesh",
		ET_Triangles, "Triangles",
		ET_TwistLimit, "TwistLimit",
		ET_TwistLimitContactDistance, "TwistLimitContactDistance",
		ET_TwistLimitEnabled, "TwistLimitEnabled",
		ET_UpVector, "UpVector",
		ET_Upper, "Upper",
		ET_Value, "Value",
		ET_WakeCounter, "WakeCounter",
		ET_YAngle, "YAngle",
		ET_ZAngle, "ZAngle",
		ET_actor0, "actor0",
		ET_actor1, "actor1",
		ET_angular, "angular",
		ET_eACTOR0, "eACTOR0",
		ET_eACTOR1, "eACTOR1",
		ET_eSLERP, "eSLERP",
		ET_eSWING, "eSWING",
		ET_eSWING1, "eSWING1",
		ET_eSWING2, "eSWING2",
		ET_eTWIST, "eTWIST",
		ET_eX, "eX",
		ET_eY, "eY",
		ET_eZ, "eZ",
		ET_force, "force",
		ET_linear, "linear",
		ET_lower, "lower",
		ET_materialIndices, "materialIndices",
		ET_minPositionIters, "minPositionIters",
		ET_minVelocityIters, "minVelocityIters",
		ET_points, "points",
		ET_samples, "samples",
		ET_torque, "torque",
		ET_upper, "upper",
		ET_yLimit, "yLimit",
		ET_zLimit, "zLimit",
	};

	struct AttributeStruct
	{
		AttributeType mType;
		const char	  *mName;
	};

	static AttributeStruct gAttributes[AT_LAST] =
	{
		AT_version, "version",
	};

	typedef std::unordered_map< std::string, ElementType > ElementTypeMap;
	typedef std::unordered_map< std::string, AttributeType > AttributeTypeMap;

	ElementTypeMap gElementsMap;
	AttributeTypeMap gAttributesMap;

	static void initMaps(void)
	{
		for (auto &i : gElements)
		{
			gElementsMap[std::string(i.mName)] = i.mType;
		}
		for (auto &i : gAttributes)
		{
			gAttributesMap[std::string(i.mName)] = i.mType;
		}
	}

	static ElementType getElementType(const char *str)
	{
		ElementType ret = ET_LAST;

		ElementTypeMap::iterator found = gElementsMap.find(std::string(str));
		if (found != gElementsMap.end())
		{
			ret = (*found).second;
		}
		return ret;
	}

	static AttributeType getAttributeType(const char *str)
	{
		AttributeType ret = AT_LAST;

		AttributeTypeMap::iterator found = gAttributesMap.find(std::string(str));
		if (found != gAttributesMap.end())
		{
			ret = (*found).second;
		}
		return ret;
	}
	static const char *getElementName(ElementType t)
	{
		const char *ret = "**UNKONWN-ELEMENT-TYPE**";
		if (t < ET_LAST)
		{
			ret = gElements[t].mName;
		}
		return ret;
	}

	static const char *getAttributeName(AttributeType t)
	{
		const char *ret = "**UNKONWN-ATTRIBUTE-TYPE**";
		if (t < AT_LAST)
		{
			ret = gAttributes[t].mName;
		}
		return ret;
	}


#define MAX_STACK 32

	typedef std::unordered_map< uint32_t, PHYSICS_DOM::NodeDef * > NodeIdMap;

	class ImportPhysXDOMImpl : public ImportPhysXDOM, public FAST_XML::FastXml::Callback
	{
	public:
		ImportPhysXDOMImpl(void)
		{
			initMaps(); // initialize the look up tables

		}
		virtual ~ImportPhysXDOMImpl(void)
		{

		}

		void reportError(uint32_t lineno, const char *fmt, ...)
		{
			va_list         args;
			char            buffer[4096];
			va_start(args, fmt);
			STRING_HELPER::stringFormatV(buffer, sizeof(buffer), fmt, args);
			va_end(args);
			printf("[ImportPhysXDOM:ERROR:Lineno:%d]%s\n", lineno, buffer);
		}

		void nestingError(uint32_t lineno, ElementType type, ElementType expected, ElementType parent)
		{
			reportError(lineno, "Nesting error <%s> expected parent <%s> (or others) but found <%s>",
				getElementName(type),
				getElementName(expected),
				getElementName(parent));
		}


		// Imports an XML serialized asset and converts it into a standardized PhysicsDOM
		virtual bool importPhysXDOM(const char *xmlName,		// Name of the PhysX XML file
			PHYSICS_DOM::PhysicsDOMDef &dom) final	// The DOM to load it into
		{
			bool ret = false;

			PHYSICS_DOM::PhysicsDOMDef c;
			dom = c;	// empty it
			mImportDOM = &dom;

			FAST_XML::FastXml *f = FAST_XML::FastXml::create();
			f->processXml(xmlName, this);
			f->release();
			if (!dom.mCollections.empty())
			{
				// Create a scene which instances the collections
				PHYSICS_DOM::SceneDef *scene = new PHYSICS_DOM::SceneDef;
				scene->mId = getID();
				for (auto &i : dom.mCollections)
				{
					PHYSICS_DOM::InstanceCollectionDef *ic = new PHYSICS_DOM::InstanceCollectionDef;
					ic->mId = getID();
					ic->mCollection = i->mId;
					scene->mNodes.push_back(ic);
				}
				dom.mScenes.push_back(scene);
				ret = true;
			}
			mImportDOM = nullptr;

			return ret;
		}

		// return true to continue processing the XML document, false to skip.
		virtual bool processElement(
			const char *elementName,   // name of the element
			uint32_t argc,         // number of attributes pairs
			const char **argv,         // list of attributes.
			const char  *elementData,  // element data, null if none
			uint32_t lineno) final  // line number in the source XML file
		{
			if (argc & 1) // if it's odd
			{
				reportError(lineno, "Attribute key/value pair mismatch");
				argc--;
			}
			uint32_t acount = argc / 2;
			mCurrentType = getElementType(elementName);
			mPreviousPreviousType = mStackLocation >= 2 ? mTypeStack[mStackLocation - 2] : ET_LAST;
			mPreviousType = mStackLocation ? mTypeStack[mStackLocation - 1] : ET_LAST;
			mTypeStack[mStackLocation] = mCurrentType;
			mStackLocation++;
			if (mStackLocation > MAX_STACK)
			{
				mStackLocation = MAX_STACK;
				reportError(lineno, "ElementTypes nested too deeply!");
			}
			for (uint32_t i = 0; i < acount; i++)
			{
				AttributeType atype = getAttributeType(argv[i * 2 + 0]);
				getAttributeName(atype);
			}

			switch (mCurrentType)
			{
				case ET_BounceThreshold:
				case ET_ChildPose:
				case ET_ColumnScale:
				case ET_ContactDistance:
				case ET_ConvexEdgeThreshold:
				case ET_Damping:
				case ET_DistanceJointFlags:
				case ET_Drive:
				case ET_DriveForceLimit:
				case ET_DriveGearRatio:
				case ET_DrivePosition:
				case ET_DriveType:
				case ET_DriveVelocity:
				case ET_ExternalCompliance:
				case ET_ExternalDriveIterations:
				case ET_ForceLimit:
				case ET_Format:
				case ET_HalfExtents:
				case ET_HalfHeight:
				case ET_HeightField:
				case ET_HeightFieldFlags:
				case ET_HeightScale:
				case ET_InternalCompliance:
				case ET_InternalDriveIterations:
				case ET_Joint:
				case ET_Limit:
				case ET_LimitCone:
				case ET_LinearLimit:
				case ET_Links:
				case ET_Lower:
				case ET_MaxDistance:
				case ET_MaxNbActors:
				case ET_MaxProjectionIterations:
				case ET_MinDistance:
				case ET_Motion:
				case ET_NbColumns:
				case ET_NbRows:
				case ET_NumActors:
				case ET_Parent:
				case ET_ParentPose:
				case ET_Points:
				case ET_PxActorRef:
				case ET_PxAggregate:
				case ET_PxArticulation:
				case ET_PxArticulationLink:
				case ET_PxArticulationRef:
				case ET_PxBVH33TriangleMesh:
				case ET_PxBoxGeometry:
				case ET_PxCapsuleGeometry:
				case ET_PxD6Joint:
				case ET_PxDistanceJoint:
				case ET_PxHeightField:
				case ET_PxHeightFieldGeometry:
				case ET_PxPrismaticJoint:
				case ET_PxRevoluteJoint:
				case ET_PxShapeRef:
				case ET_PxSphereGeometry:
				case ET_PxSphericalJoint:
				case ET_PxTriangleMeshGeometry:
				case ET_Radius:
				case ET_RowScale:
				case ET_SelfCollision:
				case ET_SeparationTolerance:
				case ET_SphericalJointFlags:
				case ET_Stiffness:
				case ET_SwingLimit:
				case ET_SwingLimitContactDistance:
				case ET_SwingLimitEnabled:
				case ET_TangentialDamping:
				case ET_TangentialStiffness:
				case ET_TargetOrientation:
				case ET_TargetVelocity:
				case ET_Thickness:
				case ET_Tolerance:
				case ET_TriangleMesh:
				case ET_Triangles:
				case ET_TwistLimit:
				case ET_TwistLimitContactDistance:
				case ET_TwistLimitEnabled:
				case ET_Upper:
				case ET_Value:
				case ET_YAngle:
				case ET_ZAngle:
				case ET_angular:
				case ET_eSLERP:
				case ET_eSWING:
				case ET_eSWING1:
				case ET_eSWING2:
				case ET_eTWIST:
				case ET_eX:
				case ET_eY:
				case ET_eZ:
				case ET_linear:
				case ET_lower:
				case ET_materialIndices:
				case ET_samples:
				case ET_upper:
				case ET_yLimit:
				case ET_zLimit:
					reportError(lineno, "ElementType(%s) not yet implemented", elementName);
					break;
				case ET_UpVector:
				case ET_Length:
				case ET_Mass:
				case ET_Speed:
				case ET_ActorFlags:
				case ET_DominanceGroup:
				case ET_OwnerClient:
				case ET_RestitutionCombineMode:
				case ET_FrictionCombineMode:
				case ET_Shapes:
				case ET_Geometry:
				case ET_SimulationFilterData:
				case ET_QueryFilterData:
				case ET_Materials:
				case ET_ContactOffset:
				case ET_RestOffset:
				case ET_MaxMargin:
				case ET_MinCCDAdvanceCoefficient:
				case ET_MaxDepenetrationVelocity:
				case ET_MaxContactImpulse:
				case ET_MaxAngularVelocity:
				case ET_SleepThreshold:
				case ET_StabilizationThreshold:
				case ET_WakeCounter:
				case ET_SolverIterationCounts:
				case ET_minPositionIters:
				case ET_minVelocityIters:
				case ET_ContactReportThreshold:
				case ET_Actors:
				case ET_BreakForce:
				case ET_force:
				case ET_torque:
				case ET_ConstraintFlags:
				case ET_InvMassScale0:
				case ET_InvMassScale1:
				case ET_InvInertiaScale0:
				case ET_InvInertiaScale1:
				case ET_ProjectionAngularTolerance:
				case ET_ProjectionLinearTolerance:
					// We ignore these elements currently; may need to add support for some of them
					// later as custom properties.  Many are just safely ignored however.
					break;
				case ET_actor0:
				case ET_actor1:
					if (mPreviousType == ET_Actors)
					{
						uint32_t id = uint32_t(atoi(elementData));
						auto found = mNodeIdMap.find(id);
						if (found == mNodeIdMap.end())
						{
							reportError(lineno, "Unable to find node id: %s", elementData);
						}
						else
						{
							PHYSICS_DOM::NodeDef *n = found->second;
							if (mCurrentJoint)
							{
								if (mCurrentType == ET_actor0)
								{
									mCurrentJoint->mBody0 = n->mId;
								}
								else
								{
									mCurrentJoint->mBody1 = n->mId;
								}
							}
						}
					}
					else
					{
						nestingError(lineno, mCurrentType, ET_Actors, mPreviousType);
					}
					break;
				case ET_LinearDamping:
				case ET_AngularDamping:
					if (mCurrentRigidDynamic)
					{
						float v = STRING_HELPER::getFloatValue(elementData, nullptr);
						switch (mCurrentType)
						{
						case ET_LinearDamping:
							mCurrentRigidDynamic->mLinearDamping = v;
							break;
						case ET_AngularDamping:
							mCurrentRigidDynamic->mAngularDamping = v;
							break;
						}
					}
					break;
				case ET_MassSpaceInertiaTensor:
				case ET_LinearVelocity:
				case ET_AngularVelocity:
					if (mCurrentRigidDynamic)
					{
						PHYSICS_DOM::Vec3 v;
						STRING_HELPER::getVec3(elementData, nullptr, v.x, v.y, v.z);
						switch (mCurrentType)
						{
							case ET_MassSpaceInertiaTensor:
								mCurrentRigidDynamic->mMassSpaceInertiaTensor = v;
								break;
							case ET_LinearVelocity:
								mCurrentRigidDynamic->mLinearVelocity = v;
								break;
							case ET_AngularVelocity:
								mCurrentRigidDynamic->mAngularVelocity = v;
								break;
						}
					}
					break;
				case ET_CMassLocalPose:
					if (mCurrentRigidDynamic)
					{
						PHYSICS_DOM::Pose p;
						const char *scan = elementData;
						const char *nextScan = nullptr;
						bool ok = STRING_HELPER::getVec4(scan, &nextScan, p.q.x, p.q.y, p.q.z, p.q.w);
						if (ok)
						{
							scan = nextScan;
							STRING_HELPER::getVec3(scan, nullptr, p.p.x, p.p.y, p.p.z);
						}
						mCurrentRigidDynamic->mCenterOfMassLocalPose = p;
					}
					break;
				case ET_Rotation:
					if (mPreviousType == ET_Scale && mPreviousPreviousType == ET_PxConvexMeshGeometry)
					{
						if (mCurrentConvexHullGeometry)
						{
							PHYSICS_DOM::Quat q;
							STRING_HELPER::getVec4(elementData, nullptr, q.x, q.y, q.z, q.w);
							mCurrentConvexHullGeometry->mScale.rotation = q;
						}
					}
					else
					{
						nestingError(lineno, mCurrentType, ET_PxConvexMeshGeometry, mPreviousType);
					}
					break;
				case ET_Scale:
					if (mPreviousType == ET_PxConvexMeshGeometry || mPreviousPreviousType == ET_PxConvexMeshGeometry )
					{
						if (mCurrentConvexHullGeometry && mPreviousPreviousType == ET_PxConvexMeshGeometry )
						{
							PHYSICS_DOM::Vec3 s;
							STRING_HELPER::getVec3(elementData, nullptr, s.x, s.y, s.z);
							mCurrentConvexHullGeometry->mScale.scale = s;
						}
					}
					else if (mPreviousType == ET_PhysX30Collection )
					{

					}
					else
					{
						nestingError(lineno, mCurrentType, ET_PxConvexMeshGeometry, mPreviousType);
					}
					break;
				case ET_Flags:
					if (mPreviousType == ET_PxShape)
					{

					}
					else
					{
						nestingError(lineno, mCurrentType, ET_PxShape, mPreviousType);
					}
					break;
				case ET_ConvexMesh:
					if (mCurrentConvexHullGeometry)
					{
						uint32_t id = uint32_t(atoi(elementData));
						auto found = mNodeIdMap.find(id);
						if (found == mNodeIdMap.end())
						{
							reportError(lineno, "Unable to find node id: %s", elementData);
						}
						else
						{
							PHYSICS_DOM::NodeDef *n = found->second;
							mCurrentConvexHullGeometry->mConvexMesh = n->mId;
						}
					}
					break;
				case ET_PxMaterialRef:
					if (mCurrentGeometryInstance)
					{
						uint32_t id = uint32_t(atoi(elementData));
						auto found = mNodeIdMap.find(id);
						if (found == mNodeIdMap.end())
						{
							reportError(lineno, "Unable to find node id: %s", elementData);
						}
						else
						{
							PHYSICS_DOM::NodeDef *n = found->second;
							mCurrentGeometryInstance->mMaterials.push_back(n->mId);
						}
					}
					break;
				case ET_PxConvexMeshGeometry:
					if (mCurrentGeometryInstance)
					{
						mCurrentConvexHullGeometry = new PHYSICS_DOM::ConvexHullGeometryDef;
						mCurrentGeometry = static_cast<PHYSICS_DOM::GeometryDef *>(mCurrentConvexHullGeometry);
						mCurrentGeometryInstance->mGeometry = mCurrentGeometry;
					}
					break;
				case ET_PxPlaneGeometry:
					if (mCurrentGeometryInstance)
					{
						PHYSICS_DOM::PlaneGeometryDef *plane = new PHYSICS_DOM::PlaneGeometryDef;
						mCurrentGeometry = static_cast<PHYSICS_DOM::GeometryDef *>(plane);
						mCurrentGeometryInstance->mGeometry = mCurrentGeometry;
					}
					break;
				case ET_PxShape:
					if (mCurrentRigidBody)
					{
						mCurrentGeometryInstance = new PHYSICS_DOM::GeometryInstanceDef;
						mCurrentRigidBody->mGeometryInstances.push_back(mCurrentGeometryInstance);
					}
					break;
				case ET_LocalPose:
				case ET_eACTOR0:
				case ET_eACTOR1:
					{
						PHYSICS_DOM::Pose p;
						if (elementData)
						{
							const char *scan = elementData;
							const char *nextScan = nullptr;
							bool ok = STRING_HELPER::getVec4(scan, &nextScan, p.q.x, p.q.y, p.q.z, p.q.w);
							if (ok)
							{
								scan = nextScan;
								STRING_HELPER::getVec3(scan, nullptr, p.p.x, p.p.y, p.p.z);
							}
						}
						switch (mPreviousType)
						{
							case ET_PxShape:
								if (mCurrentGeometryInstance)
								{
									mCurrentGeometryInstance->mLocalPose = p;
								}
								else
								{
									reportError(lineno, "Missing current geometry instance");
								}
								break;
							case ET_PxFixedJoint:
							case ET_LocalPose:
								if (mCurrentJoint)
								{
									if (mCurrentType == ET_eACTOR0)
									{
										mCurrentJoint->mLocalpose0 = p;
									}
									else if (mCurrentType == ET_eACTOR1)
									{
										mCurrentJoint->mLocalpose1 = p;
									}
								}
								else
								{
									reportError(lineno, "Missing current joint");
								}
								break;
							default:
								nestingError(lineno, mCurrentType, ET_PxShape, mPreviousType);
								break;
						}
					}
					break;
				case ET_GlobalPose:
					if (mCurrentRigidBody)
					{
						PHYSICS_DOM::Pose p;
						const char *scan = elementData;
						const char *nextScan = nullptr;
						bool ok = STRING_HELPER::getVec4(scan, &nextScan, p.q.x, p.q.y, p.q.z, p.q.w);
						if (ok)
						{
							scan = nextScan;
							STRING_HELPER::getVec3(scan, nullptr, p.p.x, p.p.y, p.p.z);
						}
						mCurrentRigidBody->mGlobalPose = p;
					}
					break;
				case ET_Name:
					if (mCurrentNode && elementData )
					{
						mCurrentNode->mName = std::string(elementData);
					}
					break;
				case ET_PxFixedJoint:
					if (mPreviousType == ET_PhysX30Collection)
					{
						mCurrentFixedJoint = new PHYSICS_DOM::FixedJointDef;
						mCurrentNode = static_cast<PHYSICS_DOM::NodeDef *>(mCurrentFixedJoint);
						mCurrentJoint = static_cast<PHYSICS_DOM::JointDef *>(mCurrentFixedJoint);
						mCurrentFixedJoint->mId = getID();
						mCurrentCollection->mNodes.push_back(mCurrentFixedJoint);
					}
					else
					{
						nestingError(lineno, mCurrentType, ET_PhysX30Collection, mPreviousType);
					}
					break;
				case ET_PxRigidDynamic:
					if (mPreviousType == ET_PhysX30Collection)
					{
						mCurrentRigidDynamic = new PHYSICS_DOM::RigidDynamicDef;
						mCurrentNode = static_cast<PHYSICS_DOM::NodeDef *>(mCurrentRigidDynamic);
						mCurrentRigidBody = static_cast<PHYSICS_DOM::RigidBodyDef *>(mCurrentRigidDynamic);
						mCurrentRigidDynamic->mId = getID();
						mCurrentCollection->mNodes.push_back(mCurrentRigidDynamic);
					}
					else
					{
						nestingError(lineno, mCurrentType, ET_PhysX30Collection, mPreviousType);
					}
					break;
				case ET_PxRigidStatic:
					if (mPreviousType == ET_PhysX30Collection)
					{
						mCurrentRigidStatic = new PHYSICS_DOM::RigidStaticDef;
						mCurrentNode = static_cast<PHYSICS_DOM::NodeDef *>(mCurrentRigidStatic);
						mCurrentRigidBody = static_cast<PHYSICS_DOM::RigidBodyDef *>(mCurrentRigidStatic);
						mCurrentRigidStatic->mId = getID();
						mCurrentCollection->mNodes.push_back(mCurrentRigidStatic);
					}
					else
					{
						nestingError(lineno, mCurrentType, ET_PhysX30Collection, mPreviousType);
					}
					break;
				case ET_points:
					if (mPreviousType == ET_PxConvexMesh)
					{
						// ok.. time to build up the array of points...
						PHYSICS_DOM::Vec3 v;
						const char *scan = elementData;
						const char *nextVec = nullptr;
						bool ok = true;
						while (ok)
						{
							ok = STRING_HELPER::getVec3(scan, &nextVec, v.x, v.y, v.z);
							if (ok)
							{
								mCurrentConvexHull->mPoints.push_back(v);
								if (nextVec)
								{
									scan = nextVec;
								}
								else
								{
									ok = false;
								}
							}
						}
					}
					else
					{
						nestingError(lineno, mCurrentType, ET_PxConvexMesh, mPreviousType);
					}
					break;
				case ET_PxConvexMesh:
					if (mPreviousType == ET_PhysX30Collection)
					{
						mCurrentConvexHull = new PHYSICS_DOM::ConvexHullDef;
						mCurrentNode = static_cast<PHYSICS_DOM::NodeDef *>(mCurrentConvexHull);
						mCurrentConvexHull->mId = getID();
						mCurrentCollection->mNodes.push_back(mCurrentConvexHull);
					}
					else
					{
						nestingError(lineno, mCurrentType, ET_PhysX30Collection, mPreviousType);
					}
					break;
				case ET_Restitution:
					if (mPreviousType == ET_PxMaterial)
					{
						if (mCurrentMaterial)
						{
							mCurrentMaterial->mRestitution = STRING_HELPER::getFloatValue(elementData, nullptr);
						}
					}
					else
					{
						nestingError(lineno, mCurrentType, ET_PxMaterial, mPreviousType);
					}
					break;
				case ET_StaticFriction:
					if (mPreviousType == ET_PxMaterial)
					{
						if (mCurrentMaterial)
						{
							mCurrentMaterial->mStaticFriction = STRING_HELPER::getFloatValue(elementData, nullptr);
						}
					}
					else
					{
						nestingError(lineno, mCurrentType, ET_PxMaterial, mPreviousType);
					}
					break;
				case ET_DynamicFriction:
					if (mPreviousType == ET_PxMaterial)
					{
						if (mCurrentMaterial)
						{
							mCurrentMaterial->mDynamicFriction = STRING_HELPER::getFloatValue(elementData, nullptr);
						}
					}
					else
					{
						nestingError(lineno, mCurrentType, ET_PxMaterial, mPreviousType);
					}
					break;
				case ET_Id:
					switch (mPreviousType)
					{
						case ET_PxMaterial:
						case ET_PxConvexMesh:
						case ET_PxRigidStatic:
						case ET_PxRigidDynamic:
						case ET_PxFixedJoint:
							if (mCurrentNode)
							{
								uint32_t id = (uint32_t)atoi(elementData);
								mNodeIdMap[id] = mCurrentNode;
							}
							break;
						default:
							reportError(lineno, "Got <id> with for node type (%s) not yet supported.", getElementName(mPreviousType));
							break;
					}
					break;
				case ET_PxMaterial:
					if ( mPreviousType == ET_PhysX30Collection )
					{
						mCurrentMaterial = new PHYSICS_DOM::PhysicsMaterialDef;
						mCurrentNode = static_cast<PHYSICS_DOM::NodeDef *>(mCurrentMaterial);
						mCurrentMaterial->mId = getID();
						mCurrentCollection->mNodes.push_back(mCurrentMaterial);
					}
					else
					{
						nestingError(lineno, mCurrentType, ET_PhysX30Collection, mPreviousType);
					}
					break;
				case ET_PhysX30Collection:
					{
						mCurrentCollection = new PHYSICS_DOM::CollectionDef;
						mCurrentCollection->mId = getID();
						mImportDOM->mCollections.push_back(mCurrentCollection);
					}
					break;
				default:
					reportError(lineno, "Unknown elementType(%s)", elementName);
					break;
			}
			return true;
		}

		virtual bool processComment(const char *comment)final  // encountered a comment in the XML
		{
			return true;
		}

		void reportCloseError(uint32_t lineno, ElementType type, ElementType expected, ElementType previousType)
		{
			reportError(lineno, "Got element-close <%s> without matching parent <%s> instead found <%s>",
				getElementName(type),
				getElementName(expected),
				getElementName(previousType));
		}

		// 'element' is the name of the element that is being closed.
		// depth is the recursion depth of this element.
		// Return true to continue processing the XML file.
		// Return false to stop processing the XML file; leaves the read pointer of the stream right after this close tag.
		// The bool 'isError' indicates whether processing was stopped due to an error, or intentionally canceled early.
		virtual bool processClose(const char *element, uint32_t depth, bool &isError, uint32_t lineno) final	  // process the 'close' indicator for a previously encountered element
		{
			// We pop the element type stack and revise the current and previous type variables
			if ((depth + 1) != mStackLocation)
			{
				reportError(lineno, "Element Stack is messed up.");
			}
			ElementType type = getElementType(element);
			if (mStackLocation)
			{
				mStackLocation--;
				if (mTypeStack[mStackLocation] != type)
				{
					reportError(lineno, "ElementClose did not match the previous element open! Invalid XML file.");
					mStackLocation++; // don't pop the stack, this was a mismatched close
					return true;
				}
				else
				{
					mPreviousPreviousType = ET_LAST;
					mCurrentType = mTypeStack[mStackLocation];
					if (mStackLocation)
					{
						mPreviousType = mTypeStack[mStackLocation - 1];
						if ((mStackLocation - 1))
						{
							mPreviousPreviousType = mTypeStack[mStackLocation - 2];
						}
					}
					else
					{
						mPreviousType = ET_LAST;
					}
				}
			}
			switch (type)
			{
			case ET_PhysX30Collection:
				mCurrentCollection = nullptr;
				break;
			case ET_PxPlaneGeometry:
				mCurrentGeometry = nullptr;
				break;
			case ET_PxConvexMeshGeometry:
				mCurrentGeometry = nullptr;
				mCurrentConvexHullGeometry = nullptr;
				break;
			case ET_PxShape:
				mCurrentGeometryInstance = nullptr;
				break;
			case ET_PxConvexMesh:
				mCurrentConvexHull = nullptr;
				mCurrentNode = nullptr;
				break;
			case ET_PxRigidStatic:
				mCurrentRigidStatic = nullptr;
				mCurrentNode = nullptr;
				break;
			case ET_PxRigidDynamic:
				mCurrentRigidDynamic = nullptr;
				mCurrentNode = nullptr;
				break;
			case ET_PxFixedJoint:
				mCurrentFixedJoint = nullptr;
				mCurrentNode = nullptr;
				break;
			case ET_PxMaterial:
				mCurrentMaterial = nullptr;
				mCurrentNode = nullptr;
				break;
			}
			return true;
		}


	virtual void release(void) final
	{
		delete this;
	}

	std::string getID(void)
	{
		mId++;
		char scratch[512];
		STRING_HELPER::stringFormat(scratch, 512, "ID%d", mId);
		return std:: string(scratch);
	}

	uint32_t		mId{ 0 };
	// The DOM we are importing
	PHYSICS_DOM::PhysicsDOMDef		*mImportDOM{ nullptr };
	PHYSICS_DOM::PhysicsMaterialDef	*mCurrentMaterial{ nullptr };
	PHYSICS_DOM::CollectionDef		*mCurrentCollection{ nullptr };
	PHYSICS_DOM::ConvexHullDef		*mCurrentConvexHull{ nullptr };
	PHYSICS_DOM::RigidDynamicDef	*mCurrentRigidDynamic{ nullptr };
	PHYSICS_DOM::RigidStaticDef		*mCurrentRigidStatic{ nullptr };
	PHYSICS_DOM::RigidBodyDef		*mCurrentRigidBody{ nullptr };
	PHYSICS_DOM::NodeDef			*mCurrentNode{ nullptr };
	PHYSICS_DOM::GeometryDef		*mCurrentGeometry{ nullptr };
	PHYSICS_DOM::GeometryInstanceDef	*mCurrentGeometryInstance{ nullptr };
	PHYSICS_DOM::ConvexHullGeometryDef	*mCurrentConvexHullGeometry{ nullptr };
	PHYSICS_DOM::JointDef			*mCurrentJoint{ nullptr };
	PHYSICS_DOM::FixedJointDef		*mCurrentFixedJoint{ nullptr };
	uint32_t						mStackLocation{ 0 };
	ElementType						mCurrentType{ ET_LAST };	// The current element type we are processing
	ElementType						mPreviousType{ ET_LAST };	// The previous element type (parent node type)
	ElementType						mPreviousPreviousType{ ET_LAST }; // two up the call stack
	ElementType						mTypeStack[MAX_STACK];
	NodeIdMap						mNodeIdMap; // map XML id's to our Node pointers
};

ImportPhysXDOM *ImportPhysXDOM::create(void)
{
	ImportPhysXDOMImpl *ret = new ImportPhysXDOMImpl;
	return static_cast<ImportPhysXDOM *>(ret);
}


} // End of IMPORT_PHYSX_DOM namespace

#include "RenderDebugPhysX.h"
#include "PxPhysics.h"
#include "PxScene.h"
#include "PxRigidActor.h"
#include "PxRigidDynamic.h"
#include "PxDeletionListener.h"
#include "extensions/PxRigidBodyExt.h"
#include "common/PxRenderBuffer.h"
#include "foundation/PxVec2.h"
#include "PxPhysics.h"
#include "PxScene.h"
#include "PxActor.h"
#include "PxRigidActor.h"
#include "PxShape.h"
#include "cloth/PxCloth.h"
#include "PxDeletionListener.h"

#define ALIAS_PHYSX

#include "NvRenderDebugTyped.h"

#include "geometry/PxHeightField.h"
#include "geometry/PxConvexMesh.h"
#include "geometry/PxConvexMeshGeometry.h"
#include "geometry/PxTriangleMesh.h"
#include "geometry/PxTriangleMeshGeometry.h"
#include "geometry/PxBoxGeometry.h"
#include "geometry/PxSphereGeometry.h"
#include "geometry/PxCapsuleGeometry.h"
#include "geometry/PxHeightFieldGeometry.h"
#include "geometry/PxHeightFieldSample.h"

#include "foundation/PxMat44.h"
#include "foundation/PxTransform.h"
#include "foundation/PxMath.h"

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxAssert.h"
#include "foundation/PxVec3.h"
#include "SimpleCamera.h"

#ifdef _MSC_VER
#if _MSC_VER <= 1500
#define OLD_VC
#pragma warning(disable:4619)
#endif
#endif

#ifdef _MSC_VER
#ifndef OLD_VC
#pragma warning(disable:4986) // temporary something with exceptions needs to be figured out.
#endif
#pragma warning(disable:4530) // temporary, something else with exceptions that needs to be figured out.
#endif

#ifdef PX_APPLE
#include <tr1/unordered_map>
#else
#include <unordered_map>
#endif
#include <vector>

#define SPHERE_VELOCITY 20
#define SPHERE_MASS 50
#define SPHERE_RADIUS 0.2f
#define DRAG_FORCE 0.1f

namespace NV_PHYSX_FRAMEWORK
{

class RenderDebugScenePhysX 
{
public:
	virtual void notifyModifySamples(physx::PxHeightField *hf,uint32_t subColStart,uint32_t subRowStart,uint32_t nbSubCols,uint32_t nbSubRows) = 0;
	virtual physx::PxScene *getScene(void) = 0;
	virtual void render(bool showStatics,bool showDynamics,bool showCloth) = 0;
	virtual void release(void) = 0;
	virtual void releaseActor(physx::PxRigidActor *actor) = 0;
protected:
	virtual ~RenderDebugScenePhysX(void)
	{

	}
};


RenderDebugScenePhysX *createRenderDebugScenePhysX(physx::PxScene *scene,RENDER_DEBUG::RenderDebugTyped *shapeRenderInterface);

#define DEFAULT_MAX_SHAPES 256
#define DEFAULT_MAX_ACTOR_COUNT 4096

namespace RENDER_DEBUG_PHYSX
{

class ModifySamplesPending
{
public:
	physx::PxHeightField	*mHeightField;
	uint32_t				mSubColStart;
	uint32_t				mSubRowStart;
	uint32_t				mNbSubCols;
	uint32_t				mNbSubRows;
};

typedef std::vector< ModifySamplesPending > ModifySamplesPendingVector;

#define SAFE_RELEASE(x) if ( x ) { x->release(); x = NULL; }

#define SAFE_RELEASE_MESH(x) if ( x ) { mRenderDebug->releaseTriangleMesh(x); }

// Modify this section if you want to hook the RenderDebugScenePhysX class into your own memory allocation routines.
// Default behavior is simply malloc/free


	// simple sphere, uses subdivision to enhance...
	static physx::PxVec3 simpleSpherePosition[6] =
	{
		physx::PxVec3( 1.0f, 0.0f, 0.0f),
		physx::PxVec3(-1.0f, 0.0f, 0.0f),
		physx::PxVec3( 0.0f, 1.0f, 0.0f),
		physx::PxVec3( 0.0f,-1.0f, 0.0f),
		physx::PxVec3( 0.0f, 0.0f, 1.0f),
		physx::PxVec3( 0.0f, 0.0f,-1.0f),
	};

	static physx::PxU32 simpleSphereIndices[8*3] =
	{
		0, 2, 4,
		0, 4, 3,
		0, 3, 5,
		0, 5, 2,

		1, 2, 5,
		1, 5, 3,
		1, 3, 4,
		1, 4, 2,
	};



	static physx::PxF32 fm_computePlane(const physx::PxF32 *A,const physx::PxF32 *B,const physx::PxF32 *C,physx::PxF32 *n) // returns D
	{
		physx::PxF32 vx = (B[0] - C[0]);
		physx::PxF32 vy = (B[1] - C[1]);
		physx::PxF32 vz = (B[2] - C[2]);

		physx::PxF32 wx = (A[0] - B[0]);
		physx::PxF32 wy = (A[1] - B[1]);
		physx::PxF32 wz = (A[2] - B[2]);

		physx::PxF32 vw_x = vy * wz - vz * wy;
		physx::PxF32 vw_y = vz * wx - vx * wz;
		physx::PxF32 vw_z = vx * wy - vy * wx;

		physx::PxF32 mag = ::sqrt((vw_x * vw_x) + (vw_y * vw_y) + (vw_z * vw_z));

		if ( mag < 0.000001f )
		{
			mag = 0;
		}
		else
		{
			mag = 1.0f/mag;
		}

		physx::PxF32 x = vw_x * mag;
		physx::PxF32 y = vw_y * mag;
		physx::PxF32 z = vw_z * mag;


		physx::PxF32 D = 0.0f - ((x*A[0])+(y*A[1])+(z*A[2]));

		n[0] = x;
		n[1] = y;
		n[2] = z;

		return D;
	}


#define TSCALE1 (1.0f/4.0f)
#define THSCALE1 (1.0f/16.0f)

	class MeshBuilder
	{
	public:	
		MeshBuilder(physx::PxU32 maxVertices)
		{
			mVertices.reserve(maxVertices);
		}

		void getVertex(const physx::PxF32 *p,const physx::PxF32 *n,physx::PxU32 i1,physx::PxU32 i2)
		{
			RENDER_DEBUG::RenderDebugMeshVertex v;

			v.mPosition[0] = p[0];
			v.mPosition[1] = p[1];
			v.mPosition[2] = p[2];

			v.mNormal[0] = n[0];
			v.mNormal[1] = n[1];
			v.mNormal[2] = n[2];

			v.mTexel[0] = p[i1]*TSCALE1;
			v.mTexel[1] = p[i2]*TSCALE1;

			mVertices.push_back(v);
		}

		void addTriangle(const physx::PxF32 *p1,const physx::PxF32 *p2,const physx::PxF32 *p3)
		{
			physx::PxF32 normal[3];
			fm_computePlane(p3,p2,p1,normal);

			float nx = fabsf(normal[0]);
			float ny = fabsf(normal[1]);
			float nz = fabsf(normal[2]);

			physx::PxU32 i1 = 0;
			physx::PxU32 i2 = 0;

			if ( nx <= ny && nx <= nz ) 
				i1 = 0;
			if ( ny <= nx && ny <= nz ) 
				i1 = 1;
			if ( nz <= nx && nz <= ny ) 
				i1 = 2;

			switch ( i1 )
			{
			case 0:
				if ( ny < nz )
					i2 = 1;
				else
					i2 = 2;
				break;
			case 1:
				if ( nx < nz )
					i2 = 0;
				else
					i2 = 2;
				break;
			case 2:
				if ( nx < ny )
					i2 = 0;
				else
					i2 = 1;
				break;
			}

			getVertex(p1,normal,i1,i2);
			getVertex(p2,normal,i1,i2);
			getVertex(p3,normal,i1,i2);
		}

		std::vector< RENDER_DEBUG::RenderDebugMeshVertex > mVertices;
	};

	class InstancePose
	{
	public:
		physx::PxVec3		mPosition;
		physx::PxVec3		mColumn0;
		physx::PxVec3		mColumn1;
		physx::PxVec3		mColumn2;
	};

	class MyRenderContext 
	{
	public:

		MyRenderContext(void)
		{
			mTimeStamp = 0;
			mContext = 0;
			mFrameCount = 0;
			mNext = NULL;
		}

		MyRenderContext(uint32_t meshId)
		{
			mTimeStamp = 0;
			mContext = meshId;
			mFrameCount = 0;
			mNext = NULL;
		}

		~MyRenderContext(void)
		{

		}

		physx::PxVec3 getVec3(const physx::PxVec4 &p)
		{
			return *( const physx::PxVec3 *)&p.x;
		}

		void addInstance(const physx::PxTransform &xform,const physx::PxVec3 *scale)
		{
			InstancePose pose;
			physx::PxMat44 m;
			physx::PxMat44 *dest = (physx::PxMat44 *)&m;
			*dest = physx::PxMat44(xform);
			if ( scale )
			{
				physx::PxMat44 temp = physx::PxMat44(physx::PxIdentity);
				temp.column0.x = scale->x;
				temp.column1.y = scale->y;
				temp.column2.z = scale->z;
				*dest = *dest * temp;
			}

			pose.mPosition = getVec3(m.column3);
			pose.mColumn0  = getVec3(m.column0);
			pose.mColumn1  = getVec3(m.column1);
			pose.mColumn2  = getVec3(m.column2);

			const RENDER_DEBUG::RenderDebugInstance *inst = (const RENDER_DEBUG::RenderDebugInstance *)&pose;
			mInstances.push_back(*inst);
		}

		bool newFrame(physx::PxU32 frameCount)
		{
			bool ret = false;
			if ( frameCount != mFrameCount )
			{
				mFrameCount = frameCount;
				ret = true;
			}
			return ret;
		}

		uint32_t					mTimeStamp;
		uint32_t					mContext;
		MyRenderContext				*mNext;
		physx::PxU32				mFrameCount;
		std::vector< RENDER_DEBUG::RenderDebugInstance > mInstances;
	};

#ifdef PX_APPLE
	typedef std::tr1::unordered_map< size_t, MyRenderContext * > MyRenderContextMap;
#else
#ifdef OLD_VC
	typedef std::tr1::unordered_map< size_t, MyRenderContext * > MyRenderContextMap;
#else
	typedef std::unordered_map< size_t, MyRenderContext * > MyRenderContextMap;
#endif
#endif


	class CapsuleContext
	{
	public:
		physx::PxF32	mRadius;
		physx::PxF32	mHalfHeight;
		MyRenderContext	*mMyContext;
	};

	class HeightFieldContext
	{
	public:
		void processModifySamples(RENDER_DEBUG::RenderDebug *renderDebug,
								uint32_t subColStart,
								uint32_t subRowStart,
								uint32_t nbSubCols,
								uint32_t nbSubRows)
		{
			mTimestamp = mGeometry.heightField->getTimestamp(); // revise the timestamp...
			physx::PxHeightField *hf = mGeometry.heightField;
			// Create render object
			const physx::PxU32		nbCols = hf->getNbColumns();
			const physx::PxU32		nbRows = hf->getNbRows();
			const physx::PxU32		nbVerts = nbRows * nbCols;	

			// Get the heightfield vertices for only this sub-region
			physx::PxVec3* vertexes = new physx::PxVec3[nbVerts];

			{
				uint32_t rowStart = subRowStart ? subRowStart-1 : 0;
				uint32_t rowEnd = subRowStart+nbSubRows+1;
				if ( rowEnd > nbRows )
				{
					rowEnd = nbRows;
				}
				uint32_t colStart = subColStart ? subColStart-1 : 0;
				uint32_t colEnd = subColStart+nbSubCols+1;
				if ( colEnd > nbCols )
				{
					colEnd = nbCols;
				}
				for(physx::PxU32 i = rowStart; i < rowEnd; i++) 
				{
					for(physx::PxU32 j = colStart; j < colEnd; j++) 
					{
						vertexes[i * nbCols + j] = physx::PxVec3(physx::PxF32(i), hf->getHeight(physx::PxF32(i),physx::PxF32(j)), physx::PxF32(j));
					}
				}
			}

			uint32_t refreshCount = nbSubCols*nbSubRows;
			uint32_t refreshIndex = 0;
			uint32_t *indices = new uint32_t[refreshCount];
			RENDER_DEBUG::RenderDebugMeshVertex *vertices = new RENDER_DEBUG::RenderDebugMeshVertex[refreshCount];
			physx::PxVec3 scale(mGeometry.rowScale,mGeometry.heightScale,mGeometry.columnScale);


			for(physx::PxU32 i = subColStart; i < (subColStart+nbSubCols) && i < (nbCols-1); ++i) 
			{
				for(physx::PxU32 j = subRowStart; j < (subRowStart+nbSubRows) && j < (nbRows-1); ++j) 
				{

					physx::PxU32 i0 = j*nbCols + i;
					physx::PxU32 i1 = j*nbCols + i + 1;
					physx::PxU32 i2 = (j+1) * nbCols + i;

					// i2---i3
					// |    |
					// |    |
					// i0---i1
					// this is really a corner vertex index, not triangle index
					// first triangle
					{
						physx::PxU32 idx1 = i2; // duplicate i0 to make a hole
						physx::PxU32 idx2 = i0;
						physx::PxU32 idx3 = i1;

						physx::PxVec3 v0 = vertexes[idx1];
						physx::PxVec3 v1 = vertexes[idx2];
						physx::PxVec3 v2 = vertexes[idx3];

						v0 = v0.multiply(scale);
						v1 = v1.multiply(scale);
						v2 = v2.multiply(scale);

						physx::PxVec3 fnormal = (v1 - v0).cross(v2 - v0);
						fnormal.normalize();

						RENDER_DEBUG::RenderDebugMeshVertex &v = vertices[refreshIndex];

						v.mPosition[0] = v1.x;
						v.mPosition[1] = v1.y;
						v.mPosition[2] = v1.z;

						v.mTexel[0] = v.mPosition[0]*THSCALE1;
						v.mTexel[1] = v.mPosition[2]*THSCALE1;

						v.mNormal[0] = fnormal.x;
						v.mNormal[1] = fnormal.y;
						v.mNormal[2] = fnormal.z;

						indices[refreshIndex] = i0;

						refreshIndex++;
					}
				}
			}

			renderDebug->refreshTriangleMeshVertices(mMyContext->mContext,refreshIndex,vertices,indices);

			delete []vertexes;
			delete []indices;
			delete []vertices;
		}
		uint32_t						mTimestamp;
		physx::PxHeightFieldGeometry	mGeometry;
		MyRenderContext					*mMyContext;
	};

	class ConvexHullContext
	{
	public:
		physx::PxConvexMesh		*mConvexMesh;
		MyRenderContext			*mMyContext;
	};

	class TriangleMeshContext
	{
	public:
		physx::PxTriangleMesh		*mTriangleMesh;
		MyRenderContext			*mMyContext;
	};

	typedef std::vector< CapsuleContext > CapsuleContextVector;
	typedef std::vector< ConvexHullContext > ConvexHullContextVector;
	typedef std::vector< TriangleMeshContext > TriangleMeshContextVector;
	typedef std::vector< HeightFieldContext > HeightFieldContextVector;

	class MyRenderDebugScenePhysX : public RenderDebugScenePhysX, physx::PxDeletionListener
	{
	public:
		MyRenderDebugScenePhysX(physx::PxScene *scene,RENDER_DEBUG::RenderDebugTyped *renderDebug)
		{

			mMaxActorCount = DEFAULT_MAX_ACTOR_COUNT;
			mActorBuffer = new physx::PxActor *[mMaxActorCount];
			mMaxShapeCount = DEFAULT_MAX_SHAPES;
			mShapeBuffer = new physx::PxShape *[mMaxShapeCount];

			mRenderDebug = renderDebug;
			mScene = scene;
			mPhysics = &mScene->getPhysics();
			mPhysics->registerDeletionListener(*this,physx::PxDeletionEventFlag::eUSER_RELEASE,true);

			{
				physx::PxVec3 bmin(-0.5f,-0.5f,-0.5f);
				physx::PxVec3 bmax(0.5f,0.5f,0.5f);

				physx::PxVec3 box[8];

				box[0]= physx::PxVec3( bmin[0], bmin[1], bmin[2] );
				box[1]= physx::PxVec3( bmax[0], bmin[1], bmin[2] );
				box[2]= physx::PxVec3( bmax[0], bmax[1], bmin[2] );
				box[3]= physx::PxVec3( bmin[0], bmax[1], bmin[2] );
				box[4]= physx::PxVec3( bmin[0], bmin[1], bmax[2] );
				box[5]= physx::PxVec3( bmax[0], bmin[1], bmax[2] );
				box[6]= physx::PxVec3( bmax[0], bmax[1], bmax[2] );
				box[7]= physx::PxVec3( bmin[0], bmax[1], bmax[2] );


				MeshBuilder mb(8*4);

				mb.addTriangle(&box[2].x,&box[1].x,&box[0].x);
				mb.addTriangle(&box[3].x,&box[2].x,&box[0].x);
				
				mb.addTriangle(&box[7].x,&box[2].x,&box[3].x);
				mb.addTriangle(&box[7].x,&box[6].x,&box[2].x);

				mb.addTriangle(&box[5].x,&box[1].x,&box[2].x);
				mb.addTriangle(&box[5].x,&box[2].x,&box[6].x);

				mb.addTriangle(&box[5].x,&box[4].x,&box[1].x);
				mb.addTriangle(&box[4].x,&box[0].x,&box[1].x);

				mb.addTriangle(&box[4].x,&box[6].x,&box[7].x);
				mb.addTriangle(&box[4].x,&box[5].x,&box[6].x);

				mb.addTriangle(&box[4].x,&box[7].x,&box[0].x);
				mb.addTriangle(&box[7].x,&box[3].x,&box[0].x);

				mBoxContext.mContext = mRenderDebug->getMeshId();

				mRenderDebug->createTriangleMesh(mBoxContext.mContext, (physx::PxU32)mb.mVertices.size(), &mb.mVertices[0], 0, NULL);
			}

			{
				MeshBuilder mb(512);
				physx::PxF32 pos[3] = { 0,0, 0 };

				createDetailedSphere(pos,1,16,mb);
				mSphereContext.mContext = mRenderDebug->getMeshId();
				mRenderDebug->createTriangleMesh(mSphereContext.mContext, (physx::PxU32)mb.mVertices.size(), &mb.mVertices[0], 0, NULL);
			}


		}

		~MyRenderDebugScenePhysX(void)
		{

			for (physx::PxU32 i=0; i<mCapsules.size(); i++)
			{
				SAFE_RELEASE_MESH(mCapsules[i].mMyContext->mContext);
				delete mCapsules[i].mMyContext;
			}

			for (physx::PxU32 i=0; i<mHeightFields.size(); i++)
			{
				SAFE_RELEASE_MESH(mHeightFields[i].mMyContext->mContext);
				delete mHeightFields[i].mMyContext;
			}


			for (physx::PxU32 i=0; i<mConvexHulls.size(); i++)
			{
				SAFE_RELEASE_MESH(mConvexHulls[i].mMyContext->mContext);
				delete mConvexHulls[i].mMyContext;
			}

			for (physx::PxU32 i=0; i<mTriangleMeshes.size(); i++)
			{
				SAFE_RELEASE_MESH(mTriangleMeshes[i].mMyContext->mContext);
				delete mTriangleMeshes[i].mMyContext;
			}

			SAFE_RELEASE_MESH(mBoxContext.mContext);
			SAFE_RELEASE_MESH(mSphereContext.mContext);

			mPhysics->unregisterDeletionListener(*this);

			delete []mActorBuffer;
			delete []mShapeBuffer;

		}

		void getSpherePoint(physx::PxF32 *point,physx::PxU32 x,physx::PxU32 y,const physx::PxF32 *center,physx::PxF32 radius,physx::PxF32 scale,physx::PxU32 stepCount)
		{
			if ( x == stepCount ) x = 0;
			if ( y == stepCount ) y = 0;
			physx::PxF32 a = (physx::PxF32)x*scale;
			physx::PxF32 b = (physx::PxF32)y*scale;
			physx::PxF32 tpos[3];

			tpos[0] = sinf(a)*cosf(b);
			tpos[1] = sinf(a)*sinf(b);
			tpos[2] = cosf(a);

			point[0] = center[0]+tpos[0]*radius;
			point[1] = center[1]+tpos[1]*radius;
			point[2] = center[2]+tpos[2]*radius;

		}

		void createDetailedSphere(const physx::PxF32 *pos,physx::PxF32 radius,physx::PxU32 stepCount,MeshBuilder &mb)
		{
			const physx::PxF32 pi = 3.1415926535897932384626433832795028841971693993751f;
			const physx::PxF32 pi2 = pi*2.0f;

			physx::PxF32 scale = pi2 / stepCount;

			for (physx::PxU32 y=0; y<stepCount; y++)
			{
				for (physx::PxU32 x=0; x<stepCount; x++)
				{
					physx::PxF32 p1[3];
					physx::PxF32 p2[3];
					physx::PxF32 p3[3];
					physx::PxF32 p4[3];

					getSpherePoint(p1,x,y,pos,radius,scale,stepCount);
					getSpherePoint(p2,x+1,y,pos,radius,scale,stepCount);
					getSpherePoint(p3,x+1,y+1,pos,radius,scale,stepCount);
					getSpherePoint(p4,x,y+1,pos,radius,scale,stepCount);

					mb.addTriangle(p1,p2,p3);
					mb.addTriangle(p1,p3,p4);
				}
			}
		}


		virtual void render(bool showStatics,bool showDynamics,bool showCloth)
		{
			if ( !mModifySamplesPending.empty() )
			{
				for (ModifySamplesPendingVector::iterator i=mModifySamplesPending.begin(); i!=mModifySamplesPending.end(); ++i)
				{
					ModifySamplesPending &m = (*i);
					for (HeightFieldContextVector::iterator j=mHeightFields.begin(); j!=mHeightFields.end(); ++j)
					{
						HeightFieldContext &h = (*j);
						if ( h.mGeometry.heightField == m.mHeightField )
						{
							h.processModifySamples(mRenderDebug,m.mSubColStart,m.mSubRowStart,m.mNbSubCols,m.mNbSubRows);
						}
					}
				}
				mModifySamplesPending.clear();
			}
			if ( mScene && (showStatics || showDynamics))
			{
				mRenderList = NULL;
				mFrameCount++;
				uint16_t flags = 0;

				if ( showStatics )
				{
					flags = physx::PxActorTypeFlag::eRIGID_STATIC;
				}

				if ( showDynamics )
				{
					flags|=physx::PxActorTypeFlag::eRIGID_DYNAMIC;
				}

				if ( showCloth )
				{
					flags|=physx::PxActorTypeFlag::eCLOTH;
				}

				mScene->lockRead(__FILE__,__LINE__);

				physx::PxU32 nbActors = mScene->getNbActors(physx::PxActorTypeFlags(flags));
				if ( nbActors > mMaxActorCount )
				{
					mMaxActorCount = nbActors+256;
					delete []mActorBuffer;
					mActorBuffer = new physx::PxActor *[mMaxActorCount];
				}

				mScene->getActors(physx::PxActorTypeFlags(flags),mActorBuffer,nbActors,0);
				for (physx::PxU32 i=0; i<nbActors; i++)
				{
					physx::PxActor *actor = mActorBuffer[i];
					if ( actor->is<physx::PxCloth>() )
					{
						physx::PxCloth *cloth = static_cast< physx::PxCloth *>(actor);

						physx::PxU32 capsuleCount = cloth->getNbCollisionCapsules();
						physx::PxU32 sphereCount = cloth->getNbCollisionSpheres();
						physx::PxU32 convexCount = cloth->getNbCollisionConvexes();
						physx::PxU32 triangleCount = cloth->getNbCollisionTriangles();
						physx::PxU32 planeCount = cloth->getNbCollisionPlanes();

						#define MAX_CLOTH_COLLISION 64

						if ( (capsuleCount*2) < MAX_CLOTH_COLLISION &&
							 sphereCount < MAX_CLOTH_COLLISION &&
							 convexCount < MAX_CLOTH_COLLISION &&
							 triangleCount < MAX_CLOTH_COLLISION &&
							 planeCount < MAX_CLOTH_COLLISION )
						{
							physx::PxClothCollisionSphere	spheresBuffer[MAX_CLOTH_COLLISION];
							physx::PxU32					capsulesBuffer[MAX_CLOTH_COLLISION];
							physx::PxClothCollisionPlane	planesBuffer[MAX_CLOTH_COLLISION];
							physx::PxU32					convexesBuffer[MAX_CLOTH_COLLISION];
							physx::PxClothCollisionTriangle	trianglesBuffer[MAX_CLOTH_COLLISION];
							cloth->getCollisionData(spheresBuffer,capsulesBuffer,planesBuffer,convexesBuffer,trianglesBuffer);

							mRenderDebug->pushRenderState();
							mRenderDebug->addToCurrentState(RENDER_DEBUG::DebugRenderState::SolidWireShaded);
							mRenderDebug->setCurrentColor(0xFFFF00,0xFFFFFF);

							for (physx::PxU32 j=0; j<sphereCount; j++)
							{
								physx::PxClothCollisionSphere &sphere = spheresBuffer[j];
								mRenderDebug->debugSphere(sphere.pos,sphere.radius);
							}
							for (physx::PxU32 j=0; j<triangleCount; j++)
							{
								physx::PxClothCollisionTriangle &t = trianglesBuffer[j];
								mRenderDebug->debugTri(t.vertex0,t.vertex1,t.vertex2);
							}
							mRenderDebug->popRenderState();
						}
					}
					else
					{
						physx::PxRigidActor *ra = static_cast< physx::PxRigidActor *>(actor);
						physx::PxU32 scount = ra->getNbShapes();
						if ( scount > mMaxShapeCount )
						{
							mMaxShapeCount = scount+64;
							delete []mShapeBuffer;
							mShapeBuffer = new physx::PxShape *[mMaxShapeCount];
						}
						ra->getShapes(mShapeBuffer,scount,0);
						for (physx::PxU32 j=0; j<scount; j++)
						{
							physx::PxShape *shape = mShapeBuffer[j];
							if ( renderShape(ra,shape,mShapeToContext) )
							{
								physx::PxBase *list[1];
								list[0] = static_cast< physx::PxBase *>(shape);
								mPhysics->registerDeletionListenerObjects(*this,list,1);
							}
						}
					}
				}

				mScene->unlockRead();

				MyRenderContext *scan = mRenderList;
				while ( scan )
				{
					if ( scan->mContext && !scan->mInstances.empty() )
					{
						uint32_t instanceCount = (uint32_t)scan->mInstances.size();
						mRenderDebug->renderTriangleMeshInstances(scan->mContext,instanceCount,&scan->mInstances[0]);
						scan->mInstances.clear();
					}
					scan = scan->mNext;
				}
			}
		}

		virtual void release(void)
		{
			delete this;
		}
	private:
		bool renderShape(physx::PxRigidActor *parent,physx::PxShape *shape,MyRenderContextMap &shapeToContext)
		{
			bool ret = false;

			size_t id = (size_t)shape;
			MyRenderContextMap::iterator found = shapeToContext.find(id);
			MyRenderContext *renderContext = NULL;
			if ( found == shapeToContext.end() )
			{
				renderContext = createRenderContext(shape);
				shapeToContext[id] = renderContext;
				ret = true;
			}
			else
			{
				renderContext = (*found).second;
				if ( shape->getGeometryType() == physx::PxGeometryType::eHEIGHTFIELD )
				{
					physx::PxHeightFieldGeometry geom;
					shape->getHeightFieldGeometry(geom);
					if ( geom.heightField->getTimestamp() != renderContext->mTimeStamp )
					{
						renderContext = createRenderContext(shape); // heightfields can be modified on the fly, so we need to check for this possibility!
					}
				}
			}

			if ( renderContext )
			{
				if ( parent )
				{
					physx::PxTransform localPose = shape->getLocalPose();

					physx::PxVec3 _scale;
					physx::PxVec3 *scale = NULL;
					mScene->lockRead(__FILE__,__LINE__);
					switch ( (uint32_t)shape->getGeometryType() )
					{
						case physx::PxGeometryType::eCONVEXMESH:
							{
								physx::PxConvexMeshGeometry geom;
								shape->getConvexMeshGeometry(geom);
								_scale = geom.scale.scale;
								scale = &_scale;
							}
							break;
						case physx::PxGeometryType::eTRIANGLEMESH:
							{
								physx::PxTriangleMeshGeometry geom;
								shape->getTriangleMeshGeometry(geom);
								_scale = geom.scale.scale;
								scale = &_scale;
							}
							break;
						case physx::PxGeometryType::eBOX:
							{
								physx::PxBoxGeometry geom;
								shape->getBoxGeometry(geom);
								_scale = physx::PxVec3( geom.halfExtents.x*2, geom.halfExtents.y*2, geom.halfExtents.z*2 );
								scale = &_scale;
							}
							break;
						case physx::PxGeometryType::eSPHERE:
							{
								physx::PxSphereGeometry geom;
								shape->getSphereGeometry(geom);
								_scale = physx::PxVec3(geom.radius,geom.radius,geom.radius);
								scale = &_scale;
							}
							break;
						case physx::PxGeometryType::eCAPSULE:
							{
								physx::PxQuat q(0,0,0.70710677f,0.70710677f);
								localPose.q*=q;
							}
							break;
						case physx::PxGeometryType::eHEIGHTFIELD:
							{
								physx::PxHeightFieldGeometry geom;
								shape->getHeightFieldGeometry(geom);
								renderContext->mTimeStamp = geom.heightField->getTimestamp();
							}
							break;
						default:
							break;
					}

					physx::PxTransform xform = parent->getGlobalPose()*localPose;

					mScene->unlockRead();

					if ( scale && scale->x == 1 && scale->y == 1 && scale->z == 1 )
					{
						scale = NULL;
					}

					renderContext->addInstance( xform, scale );

					if ( renderContext->newFrame(mFrameCount))
					{
						renderContext->mNext = mRenderList;
						mRenderList = renderContext;
					}
				}
			}
			return ret;
		}

		MyRenderContext * createRenderContext(physx::PxShape *shape)
		{
			MyRenderContext *ret = NULL;

			switch ( (uint32_t)shape->getGeometryType() )
			{
			case physx::PxGeometryType::eBOX:
				{
					ret = &mBoxContext;
				}
				break;
			case physx::PxGeometryType::eSPHERE:
				{
					ret = &mSphereContext;
				}
				break;
			case physx::PxGeometryType::eCAPSULE:
				{
					physx::PxCapsuleGeometry geom;
					shape->getCapsuleGeometry(geom);

					for (physx::PxU32 i=0; i<mCapsules.size(); i++)
					{
						CapsuleContext &chc = mCapsules[i];
						if ( chc.mHalfHeight == geom.halfHeight && chc.mRadius == geom.radius )
						{
							ret = chc.mMyContext;
							break;
						}
					}
					if ( !ret )
					{
						MeshBuilder mb(512);
						debugCapsule(geom.radius,geom.halfHeight*2,2,mb);
						CapsuleContext cc;
						cc.mHalfHeight = geom.halfHeight;
						cc.mRadius = geom.radius;
						uint32_t meshId = mRenderDebug->getMeshId();
						mRenderDebug->createTriangleMesh(meshId, (physx::PxU32)mb.mVertices.size(), &mb.mVertices[0], 0, NULL);
						ret = new MyRenderContext(meshId);
						cc.mMyContext = ret;
						mCapsules.push_back(cc);
					}
				}
				break;
			case physx::PxGeometryType::eTRIANGLEMESH:
				{
					physx::PxTriangleMeshGeometry geom;
					shape->getTriangleMeshGeometry(geom);

					for (physx::PxU32 i=0; i<mTriangleMeshes.size(); i++)
					{
						TriangleMeshContext &chc = mTriangleMeshes[i];
						if ( chc.mTriangleMesh == geom.triangleMesh )
						{
							ret = chc.mMyContext;
							break;
						}
					}
					if ( ret == NULL )
					{
						uint32_t context = getTriangleMeshContext(geom.triangleMesh);
						ret = new MyRenderContext(context);
						TriangleMeshContext chc;
						chc.mMyContext = ret;
						chc.mTriangleMesh = geom.triangleMesh;
						mTriangleMeshes.push_back(chc);
						physx::PxBase *list[1];
						list[0] = static_cast< physx::PxBase *>(geom.triangleMesh);
						mPhysics->registerDeletionListenerObjects(*this,list,1);
					}
				}
				break;
			case physx::PxGeometryType::eHEIGHTFIELD:
				{
					bool needListen = true;
					physx::PxHeightFieldGeometry geom;
					shape->getHeightFieldGeometry(geom);
					for (HeightFieldContextVector::iterator i=mHeightFields.begin(); i!=mHeightFields.end(); ++i)
					{
						HeightFieldContext &chc = (*i);
						if ( chc.mGeometry.heightField == geom.heightField )
						{
							if ( chc.mTimestamp != geom.heightField->getTimestamp() )
							{
								needListen = false;
								SAFE_RELEASE_MESH(chc.mMyContext->mContext);
								delete chc.mMyContext;
								mHeightFields.erase(i);
								break;
							}
							else
							{
								ret = chc.mMyContext;
								break;
							}
						}
					}
					if ( ret == NULL )
					{
						uint32_t context = getHeightFieldContext(geom);
						ret = new MyRenderContext(context);
						HeightFieldContext chc;
						chc.mTimestamp = geom.heightField->getTimestamp();
						chc.mMyContext = ret;
						chc.mGeometry = geom;
						mHeightFields.push_back(chc);
						if ( needListen )
						{
							physx::PxBase *list[1];
							list[0] = static_cast< physx::PxBase *>(geom.heightField);
							mPhysics->registerDeletionListenerObjects(*this,list,1);
						}
					}
				}
				break;
			case physx::PxGeometryType::eCONVEXMESH:
				{
					physx::PxConvexMeshGeometry geom;
					shape->getConvexMeshGeometry(geom);
					for (physx::PxU32 i=0; i<mConvexHulls.size(); i++)
					{
						ConvexHullContext &chc = mConvexHulls[i];
						if ( chc.mConvexMesh == geom.convexMesh )
						{
							ret = chc.mMyContext;
							break;
						}
					}
					if ( ret == NULL )
					{
						uint32_t context = getConvexContext(geom.convexMesh);
						ret = new MyRenderContext(context);
						ConvexHullContext chc;
						chc.mMyContext = ret;
						chc.mConvexMesh = geom.convexMesh;
						mConvexHulls.push_back(chc);
						physx::PxBase *list[1];
						list[0] = static_cast< physx::PxBase *>(geom.convexMesh);
						mPhysics->registerDeletionListenerObjects(*this,list,1);
					}
				}
				break;
			default:
				break;
			}

			return ret;
		}

		void accumulateNormal(RENDER_DEBUG::RenderDebugMeshVertex &v,const physx::PxVec3 &n)
		{
			v.mNormal[0]+=n.x;
			v.mNormal[1]+=n.y;
			v.mNormal[2]+=n.z;
		}

		uint32_t getHeightFieldContext(const physx::PxHeightFieldGeometry &geom)
		{
			uint32_t ret = 0;

			physx::PxHeightField *hf = geom.heightField;
			// Create render object
			const physx::PxU32		nbCols = hf->getNbColumns();
			const physx::PxU32		nbRows = hf->getNbRows();
			const physx::PxU32		nbVerts = nbRows * nbCols;	
			const physx::PxU32		nbFaces = (nbCols - 1) * (nbRows - 1) * 2;

			physx::PxHeightFieldSample* sampleBuffer = new physx::PxHeightFieldSample[nbVerts];
			hf->saveCells(sampleBuffer, nbVerts * sizeof(physx::PxHeightFieldSample));

			physx::PxVec3* vertexes = new physx::PxVec3[nbVerts];
			for(physx::PxU32 i = 0; i < nbRows; i++) 
			{
				for(physx::PxU32 j = 0; j < nbCols; j++) 
				{
					vertexes[i * nbCols + j] = physx::PxVec3(physx::PxF32(i), physx::PxF32(sampleBuffer[j + (i*nbCols)].height), physx::PxF32(j));
				}
			}

			uint32_t *indices = new uint32_t[nbFaces*3];
			uint32_t *destIndex = indices;
			RENDER_DEBUG::RenderDebugMeshVertex *vertices = new RENDER_DEBUG::RenderDebugMeshVertex[nbVerts];
			physx::PxVec3 scale(geom.rowScale,geom.heightScale,geom.columnScale);

			for (uint32_t i=0; i<nbVerts; i++)
			{
				const physx::PxVec3 &position = vertexes[i];
				RENDER_DEBUG::RenderDebugMeshVertex &v = vertices[i];

				v.mPosition[0] = position.x*scale.x;
				v.mPosition[1] = position.y*scale.y;
				v.mPosition[2] = position.z*scale.z;

				v.mTexel[0] = v.mPosition[0]*THSCALE1;
				v.mTexel[1] = v.mPosition[2]*THSCALE1;
			}

			uint32_t faceCount = 0;
			for(physx::PxU32 i = 0; i < (nbCols - 1); ++i) 
			{
				for(physx::PxU32 j = 0; j < (nbRows - 1); ++j) 
				{

					
					physx::PxU8 tessFlag = sampleBuffer[i+j*nbCols].tessFlag();

					physx::PxU32 i0 = j*nbCols + i;
					physx::PxU32 i1 = j*nbCols + i + 1;
					physx::PxU32 i2 = (j+1) * nbCols + i;
					physx::PxU32 i3 = (j+1) * nbCols + i+ 1;

					// i2---i3
					// |    |
					// |    |
					// i0---i1
					// this is really a corner vertex index, not triangle index
					physx::PxU32 mat0 = hf->getTriangleMaterialIndex((j*nbCols+i)*2);
					physx::PxU32 mat1 = hf->getTriangleMaterialIndex((j*nbCols+i)*2+1);
					bool hole0 = (mat0 == physx::PxHeightFieldMaterial::eHOLE);
					bool hole1 = (mat1 == physx::PxHeightFieldMaterial::eHOLE);
					// first triangle
					{
						physx::PxU32 idx1 = i2; // duplicate i0 to make a hole
						physx::PxU32 idx2 = i0;
						physx::PxU32 idx3 = tessFlag ? i3 : i1;



						physx::PxVec3 v0 = vertexes[idx1];
						physx::PxVec3 v1 = vertexes[idx2];
						physx::PxVec3 v2 = vertexes[idx3];

						v0 = v0.multiply(scale);
						v1 = v1.multiply(scale);
						v2 = v2.multiply(scale);

						physx::PxVec3 fnormal = (v1 - v0).cross(v2 - v0);
						fnormal.normalize();

						vertices[i0].mNormal[0] = fnormal.x;
						vertices[i0].mNormal[1] = fnormal.y;
						vertices[i0].mNormal[2] = fnormal.z;

						if ( !hole0 )
						{
							destIndex[0] = idx1;
							destIndex[1] = idx2;
							destIndex[2] = idx3;

							destIndex+=3;
							faceCount++;
						}

					}
					if ( !hole1 )
					{
						// second triangle
						uint32_t idx1 = i3; // duplicate i1 to make a hole
						uint32_t idx2 = tessFlag ? i0 : i2;
						uint32_t idx3 = i1;

						destIndex[0] = idx1;
						destIndex[1] = idx2;
						destIndex[2] = idx3;
						destIndex+=3;
						faceCount++;

					}
				}
			}

			delete []vertexes;

			ret = mRenderDebug->getMeshId();

			mRenderDebug->createTriangleMesh(ret, nbVerts, vertices, faceCount, indices);

			delete []indices;
			delete []vertices;

			return ret;
		}

		uint32_t getConvexContext(physx::PxConvexMesh *msh)
		{
			uint32_t ret = 0;

			MeshBuilder mb( msh->getNbVertices()*8 );

			const physx::PxU8 *indices = (const physx::PxU8 *) msh->getIndexBuffer();
			const physx::PxVec3 *vertices = msh->getVertices();
			physx::PxU32 pcount = msh->getNbPolygons();

			for (physx::PxU32 i=0; i<pcount; i++)
			{
				physx::PxHullPolygon data;
				msh->getPolygonData(i,data);
				physx::PxU32 i1 = indices[ data.mIndexBase+0];
				physx::PxU32 i2 = indices[ data.mIndexBase+1];
				physx::PxU32 i3 = indices[ data.mIndexBase+2];
				PX_ASSERT( i1 < 256 );
				PX_ASSERT( i2 < 256 );
				PX_ASSERT( i3 < 256 );
				const physx::PxVec3 &p1 = vertices[i1];
				const physx::PxVec3 &p2 = vertices[i2];
				const physx::PxVec3 &p3 = vertices[i3];
				mb.addTriangle(&p1.x,&p2.x,&p3.x);
				// fan out the other triangles...
				for (physx::PxU32 j=3; j<data.mNbVerts; j++)
				{
					i2 = i3;
					i3 = indices[data.mIndexBase+j];
					PX_ASSERT( i3 < 256 );
					const physx::PxVec3 &tp1 = vertices[i1];
					const physx::PxVec3 &tp2 = vertices[i2];
					const physx::PxVec3 &tp3 = vertices[i3];
					mb.addTriangle(&tp1.x,&tp2.x,&tp3.x);
				}
			}
			ret = mRenderDebug->getMeshId();
			mRenderDebug->createTriangleMesh(ret, (physx::PxU32)mb.mVertices.size(), &mb.mVertices[0], 0, NULL);
			return ret;
		}


		uint32_t getTriangleMeshContext(physx::PxTriangleMesh *msh)
		{
			uint32_t ret = 0;

			MeshBuilder mb(msh->getNbTriangles()*3);

			if ( msh->getTriangleMeshFlags() & physx::PxTriangleMeshFlag::e16_BIT_INDICES )
			{
				const physx::PxU16 *indices = (const physx::PxU16 *)msh->getTriangles();
				const physx::PxVec3 *vertices = msh->getVertices();
				physx::PxU32 pcount = msh->getNbTriangles();
				for (physx::PxU32 i=0; i<pcount; i++)
				{
					physx::PxU32 i1 = indices[ i*3+0];
					physx::PxU32 i2 = indices[ i*3+1];
					physx::PxU32 i3 = indices[ i*3+2];
					const physx::PxVec3 &p1 = vertices[i1];
					const physx::PxVec3 &p2 = vertices[i2];
					const physx::PxVec3 &p3 = vertices[i3];
					mb.addTriangle(&p1.x,&p2.x,&p3.x);
				}
			}
			else
			{
				const physx::PxU32 *indices = (const physx::PxU32 *)msh->getTriangles();
				const physx::PxVec3 *vertices = msh->getVertices();
				physx::PxU32 pcount = msh->getNbTriangles();
				for (physx::PxU32 i=0; i<pcount; i++)
				{
					physx::PxU32 i1 = indices[ i*3+0];
					physx::PxU32 i2 = indices[ i*3+1];
					physx::PxU32 i3 = indices[ i*3+2];
					const physx::PxVec3 &p1 = vertices[i1];
					const physx::PxVec3 &p2 = vertices[i2];
					const physx::PxVec3 &p3 = vertices[i3];
					mb.addTriangle(&p1.x,&p2.x,&p3.x);
				}
			}
			ret = mRenderDebug->getMeshId();
			mRenderDebug->createTriangleMesh(ret, (physx::PxU32)mb.mVertices.size(), &mb.mVertices[0], 0, NULL);
			return ret;
		}

		void subdivideOnSphere(int subdivision, const physx::PxVec3& p0, const physx::PxVec3& p1, const physx::PxVec3& p2, physx::PxF32 radius, physx::PxF32 height,MeshBuilder &mb)
		{
			if (subdivision == 0)
			{
				physx::PxVec3 t0;
				physx::PxVec3 t1;
				physx::PxVec3 t2;
				if (height > 0.0f)
				{
					physx::PxF32 averageY = (p0.y + p1.y + p2.y)/3.0f;
					physx::PxVec3 offset(0.0f, physx::intrinsics::sign(averageY) * height, 0.0f);

					t0 = (p0 + offset);
					t1 = (p1 + offset);
					t2 = (p2 + offset);
				}
				else
				{
					t0 = (p0);
					t1 = (p1);
					t2 = (p2);
				}

				mb.addTriangle(&t0.x, &t1.x, &t2.x );
			}
			else
			{
				physx::PxVec3 p01 = p0 + p1; p01.normalize(); p01 *= radius;
				physx::PxVec3 p12 = p1 + p2; p12.normalize(); p12 *= radius;
				physx::PxVec3 p20 = p2 + p0; p20.normalize(); p20 *= radius;

				subdivideOnSphere(subdivision-1, p0, p01, p20, radius, height, mb);
				subdivideOnSphere(subdivision-1, p1, p12, p01, radius, height, mb);
				subdivideOnSphere(subdivision-1, p2, p20, p12, radius, height, mb);
				subdivideOnSphere(subdivision-1, p01, p12, p20, radius, height, mb);

			}
		}

		PX_INLINE void debugCylinder(physx::PxF32 radius,physx::PxF32 height,physx::PxU32 subdivision,MeshBuilder &mb)
		{
			physx::PxU32 numOfSegments = physx::PxU32( physx::PxPow((physx::PxF32)2, (physx::PxF32)(subdivision + 2)) );
			physx::PxF32 segmentAngle = physx::PxTwoPi / numOfSegments;
			physx::PxF32 c = physx::PxCos(segmentAngle);
			physx::PxF32 s = physx::PxSin(segmentAngle);
			physx::PxMat33 rotY(physx::PxVec3(c, 0,-s), physx::PxVec3(0, 1, 0), physx::PxVec3(s, 0, c));

			physx::PxVec3 heightVec	= (physx::PxVec3(0.0f, height / 2.0f, 0.0f));
			physx::PxVec3 currentVec	= physx::PxVec3(radius, 0.0f, 0.0f);
			physx::PxVec3 nextVec;

			for( physx::PxU32 i=0; i<numOfSegments; ++i )
			{
				nextVec = rotY.transform(currentVec);

				physx::PxVec3 e0 = currentVec	+ heightVec;
				physx::PxVec3 e1 = nextVec		+ heightVec;
				physx::PxVec3 e2 = currentVec	- heightVec;
				physx::PxVec3 e3 = nextVec		- heightVec;

				//physx::PxVec3 nC = (-currentVec).getNormalized();
				//physx::PxVec3 nN = (-nextVec).getNormalized();

				mb.addTriangle(&e0.x, &e2.x, &e3.x );
				mb.addTriangle(&e3.x, &e1.x, &e0.x );

				//			if (d.mCloseSides)
				{
					physx::PxVec3 depthVec = - heightVec;
					mb.addTriangle(&e0.x, &e1.x, &heightVec.x);
					mb.addTriangle(&e2.x, &depthVec.x, &e3.x);
				}
				currentVec = nextVec;
			}
		}



		PX_INLINE void debugCapsule(physx::PxF32 radius,physx::PxF32 height,physx::PxU32 subdivision,MeshBuilder &mb)
		{
			debugCylinder(radius,height,subdivision,mb);
			const physx::PxF32 h2 = height * 0.5f;
			for (physx::PxU32 i = 0; i < 8; i++)
			{
				const physx::PxVec3& p0 = simpleSpherePosition[simpleSphereIndices[i*3+0]];
				const physx::PxVec3& p1 = simpleSpherePosition[simpleSphereIndices[i*3+1]];
				const physx::PxVec3& p2 = simpleSpherePosition[simpleSphereIndices[i*3+2]];
				subdivideOnSphere(subdivision, p0 * radius, p1 * radius, p2 * radius, radius, h2, mb);
			}
		}

		virtual void releaseActor(physx::PxRigidActor *actor) 
		{
			physx::PxU32 scount = actor->getNbShapes();
			for (physx::PxU32 i=0; i<scount; i++)
			{
				physx::PxShape *shape;
				actor->getShapes(&shape,1,i);
			}
		}

		virtual physx::PxScene *getScene(void) 
		{
			return mScene;
		}

		virtual void onRelease(const physx::PxBase* observed, void* userData, physx::PxDeletionEventFlag::Enum deletionEvent) 
		{
			size_t id = (size_t)observed;

			if ( observed->is<physx::PxShape>() )
			{
				MyRenderContextMap::iterator found = mShapeToContext.find(id);
				PX_ASSERT( found != mShapeToContext.end() );
				if ( found != mShapeToContext.end() )
				{
					mShapeToContext.erase(found);
				}
			}
			else if ( observed->is<physx::PxTriangleMesh>() )
			{
				const physx::PxTriangleMesh *m = static_cast< const physx::PxTriangleMesh *>(observed);
				for (TriangleMeshContextVector::iterator i=mTriangleMeshes.begin(); i!=mTriangleMeshes.end(); ++i)
				{
					TriangleMeshContext &c = (*i);
					if ( c.mTriangleMesh == m )
					{
						if ( c.mMyContext )
						{
							mRenderDebug->releaseTriangleMesh(c.mMyContext->mContext);
						}
						delete c.mMyContext;
						mTriangleMeshes.erase(i);
						break;
					}
				}
			}
			else if ( observed->is<physx::PxConvexMesh>() )
			{
				const physx::PxConvexMesh *m = static_cast< const physx::PxConvexMesh *>(observed);
				for (ConvexHullContextVector::iterator i=mConvexHulls.begin(); i!=mConvexHulls.end(); ++i)
				{
					ConvexHullContext &c = (*i);
					if ( c.mConvexMesh == m )
					{
						SAFE_RELEASE_MESH(c.mMyContext->mContext);
						delete c.mMyContext;
						mConvexHulls.erase(i);
						break;
					}
				}
			}
			else if ( observed->is<physx::PxHeightField>() )
			{
				const physx::PxHeightField *m = static_cast< const physx::PxHeightField *>(observed);

				{
					ModifySamplesPendingVector::iterator i = mModifySamplesPending.begin();
					while ( i != mModifySamplesPending.end() )
					{
						ModifySamplesPending &p = (*i);
						if ( p.mHeightField == m )
						{
							i = mModifySamplesPending.erase(i);
						}
						else
						{
							i++;
						}
					}
				}

				for (HeightFieldContextVector::iterator i=mHeightFields.begin(); i!=mHeightFields.end(); ++i)
				{
					HeightFieldContext &c = (*i);
					if ( c.mGeometry.heightField == m )
					{
						SAFE_RELEASE_MESH(c.mMyContext->mContext);
						delete c.mMyContext;
						mHeightFields.erase(i);
						break;
					}
				}
			}
			PX_UNUSED(userData);
			PX_UNUSED(deletionEvent);
		}

		virtual void notifyModifySamples(physx::PxHeightField *hf,uint32_t subColStart,uint32_t subRowStart,uint32_t nbSubCols,uint32_t nbSubRows) 
		{
			ModifySamplesPending m;
			m.mHeightField = hf;
			m.mSubColStart = subColStart;
			m.mSubRowStart = subRowStart;
			m.mNbSubCols	= nbSubCols;
			m.mNbSubRows	= nbSubRows;
			mModifySamplesPending.push_back(m);
		}


		RENDER_DEBUG::RenderDebugTyped		*mRenderDebug;
		physx::PxPhysics					*mPhysics;
		physx::PxScene						*mScene;
		physx::PxU32						mFrameCount;
		MyRenderContext						*mRenderList;

		CapsuleContextVector				mCapsules;			// convex hull render contexts
		ConvexHullContextVector				mConvexHulls;			// convex hull render contexts
		HeightFieldContextVector			mHeightFields;
		TriangleMeshContextVector			mTriangleMeshes;		// triangle mesh render contexts

		MyRenderContext						mBoxContext;
		MyRenderContext						mSphereContext;
		MyRenderContextMap					mShapeToContext;

		physx::PxU32						mMaxActorCount;
		physx::PxActor						**mActorBuffer;
		physx::PxU32						mMaxShapeCount;
		physx::PxShape						**mShapeBuffer;
		ModifySamplesPendingVector			mModifySamplesPending;
	};



class DebugScene
{
public:
	physx::PxScene				*mScene;
	RenderDebugScenePhysX		*mRenderDebugScene;
};

typedef std::vector< DebugScene > DebugSceneVector;

class RenderDebugPhysXImpl : public RenderDebugPhysX, physx::PxDeletionListener, public physx::PxQueryFilterCallback, public RENDER_DEBUG::RenderDebug::ServerStallCallback
{
public:
	RenderDebugPhysXImpl(physx::PxScene *s,RENDER_DEBUG::RenderDebugTyped *renderDebug,bool allowRegistry)
	{
		mOwnRenderDebug = false;
		initPhysics(s,renderDebug,allowRegistry);
	}

	RenderDebugPhysXImpl(const RenderDebugPhysX::Desc &rdesc)
	{
		mApplicationName = rdesc.applicationName;
		mOwnRenderDebug = true;
		RENDER_DEBUG::RenderDebug::Desc desc;
		desc.runMode = rdesc.fileOk ? RENDER_DEBUG::RenderDebug::RM_CLIENT_OR_FILE : RENDER_DEBUG::RenderDebug::RM_CLIENT;
		desc.recordFileName = rdesc.recordFileName; // the name of the test recording file
		desc.serverStallCallback = this;
		desc.applicationName = rdesc.applicationName;
		desc.hostName = rdesc.hostName;
		desc.recordRemoteCommands = rdesc.recordRemoteCommands;
		desc.playbackRemoteCommands = rdesc.playbackRemoteCommands;
#ifdef PX_X64
		if ( rdesc.useDebugDLL )
		{
			desc.dllName = "NvRenderDebugDEBUG_x64.dll";
		}
		else
		{
			desc.dllName = "NvRenderDebug_x64.dll";
		}
#else
		if ( rdesc.useDebugDLL )
		{
			desc.dllName = "NvRenderDebugDEBUG_x86.dll";
		}
		else
		{
			desc.dllName = "NvRenderDebug_x86.dll";
		}
#endif

		mRenderDebug = NULL;
		RENDER_DEBUG::RenderDebug *rd = RENDER_DEBUG::createRenderDebug(desc);
		if ( rd )
		{
			mRenderDebug = rd->getRenderDebugTyped();
		}

		initPhysics(rdesc.scene,mRenderDebug,rdesc.allowRegistry);

	}

	void initPhysics(physx::PxScene *s,RENDER_DEBUG::RenderDebugTyped *renderDebug,bool allowRegistry)
	{

		mRenderDebug = renderDebug;

		mSimpleCamera = new SimpleCamera(mRenderDebug);

		mSimulationFilterData.word0 = 0;
		mSimulationFilterData.word1 = 0;
		mSimulationFilterData.word2 = 0;
		mSimulationFilterData.word3 = 0;

		mQueryFilterData.word0 = 0;
		mQueryFilterData.word1 = 0;
		mQueryFilterData.word2 = 0;
		mQueryFilterData.word3 = 0;

		mDraggingActor = NULL;
		mDraggingShape = NULL;
		mCallback = NULL;
		mActive = true;
		mOneFrameAdvance = false;
		mCurrentSimulationScale = 1;
		mSimulationScale = 1;
		mFrameCount = 0;
		mPhysics = &s->getPhysics();
		mDefaultMaterial = NULL;

		mPhysics->registerDeletionListener(*this,physx::PxDeletionEventFlag::eUSER_RELEASE,true);
		if ( mRenderDebug )
		{
			DebugScene ds;
			ds.mScene = s;
			ds.mRenderDebugScene = createRenderDebugScenePhysX(s,mRenderDebug);
			if ( ds.mRenderDebugScene )
			{
				mDebugScenes.push_back(ds);
			}
			mRenderDebug->sendRemoteCommand("AllowRegistry %s", allowRegistry ? "true" : "false" );
			createPhysXMenus(); // create the debug visualization menus on the remote view application
		}
	}

	virtual ~RenderDebugPhysXImpl(void)
	{
		for (DebugSceneVector::iterator i=mDebugScenes.begin(); i!=mDebugScenes.end(); ++i)
		{
			DebugScene &ds = (*i);
			if ( ds.mRenderDebugScene )
			{
				SAFE_RELEASE(ds.mRenderDebugScene);
			}
		}
		mPhysics->unregisterDeletionListener(*this);
		if ( mRenderDebug && mOwnRenderDebug )
		{
			SAFE_RELEASE(mRenderDebug);
		}
		delete mSimpleCamera;
	}

	void createPhysXMenus(void)
	{
		mRenderDebug->sendRemoteCommand("BeginTab \"PhysX SDK Visualization\"");
		mRenderDebug->sendRemoteCommand("BeginGroup \"Actor Visualization\"");

		mRenderDebug->sendRemoteCommand("Slider DEBUG_SCALE 0.1 0.01 1 \"physxvis eSCALE\"");

		mRenderDebug->sendRemoteCommand("CheckBox WORLD_AXES \"physxvis eWORLD_AXES\"");

		mRenderDebug->sendRemoteCommand("CheckBox BODY_AXES \"physxvis eBODY_AXES\"");
		mRenderDebug->sendRemoteCommand("CheckBox BODY_MASS_AXES \"physxvis eBODY_MASS_AXES\"");
		mRenderDebug->sendRemoteCommand("CheckBox BODY_LIN_VELOCITY \"physxvis eBODY_LIN_VELOCITY\"");
		mRenderDebug->sendRemoteCommand("CheckBox BODY_ANG_VELOCITY \"physxvis eBODY_ANG_VELOCITY\"");
		mRenderDebug->sendRemoteCommand("CheckBox BODY_JOINT_GROUPS \"physxvis eBODY_JOINT_GROUPS\"");
		mRenderDebug->sendRemoteCommand("CheckBox CONTACT_POINT \"physxvis eCONTACT_POINT\"");
		mRenderDebug->sendRemoteCommand("CheckBox CONTACT_ERROR \"physxvis eCONTACT_ERROR\"");
		mRenderDebug->sendRemoteCommand("CheckBox CONTACT_FORCE \"physxvis eCONTACT_FORCE\"");
		mRenderDebug->sendRemoteCommand("CheckBox ACTOR_AXES \"physxvis eACTOR_AXES\"");

		mRenderDebug->sendRemoteCommand("EndGroup");


		mRenderDebug->sendRemoteCommand("BeginGroup \"Collision Visualization\"");

		mRenderDebug->sendRemoteCommand("CheckBox COLLISION_AABBS \"physxvis eCOLLISION_AABBS\"");
		mRenderDebug->sendRemoteCommand("CheckBox COLLISION_SHAPES \"physxvis eCOLLISION_SHAPES\"");
		mRenderDebug->sendRemoteCommand("CheckBox COLLISION_AXES \"physxvis eCOLLISION_AXES\"");
		mRenderDebug->sendRemoteCommand("CheckBox COLLISION_COMPOUNDS \"physxvis eCOLLISION_COMPOUNDS\"");
		mRenderDebug->sendRemoteCommand("CheckBox COLLISION_FNORMALS \"physxvis eCOLLISION_FNORMALS\"");
		mRenderDebug->sendRemoteCommand("CheckBox COLLISION_EDGES \"physxvis eCOLLISION_EDGES\"");
		mRenderDebug->sendRemoteCommand("CheckBox COLLISION_STATIC \"physxvis eCOLLISION_STATIC\"");
		mRenderDebug->sendRemoteCommand("CheckBox COLLISION_DYNAMIC \"physxvis eCOLLISION_DYNAMIC\"");
		mRenderDebug->sendRemoteCommand("CheckBox COLLISION_PAIRS \"physxvis eCOLLISION_PAIRS\"");

		mRenderDebug->sendRemoteCommand("EndGroup");

		mRenderDebug->sendRemoteCommand("BeginGroup \"Joint Visualization\"");

		mRenderDebug->sendRemoteCommand("CheckBox JOINT_LOCAL_FRAMES \"physxvis eJOINT_LOCAL_FRAMES\"");
		mRenderDebug->sendRemoteCommand("CheckBox JOINT_LIMITS \"physxvis eJOINT_LIMITS\"");

		mRenderDebug->sendRemoteCommand("EndGroup");


		mRenderDebug->sendRemoteCommand("BeginGroup \"Particle System Visualization\"");

		mRenderDebug->sendRemoteCommand("CheckBox PARTICLE_SYSTEM_POSITION \"physxvis ePARTICLE_SYSTEM_POSITION\"");
		mRenderDebug->sendRemoteCommand("CheckBox PARTICLE_SYSTEM_VELOCITY \"physxvis ePARTICLE_SYSTEM_VELOCITY\"");
		mRenderDebug->sendRemoteCommand("CheckBox PARTICLE_SYSTEM_COLLISION_NORMAL \"physxvis ePARTICLE_SYSTEM_COLLISION_NORMAL\"");
		mRenderDebug->sendRemoteCommand("CheckBox PARTICLE_SYSTEM_BOUNDS \"physxvis ePARTICLE_SYSTEM_BOUNDS\"");
		mRenderDebug->sendRemoteCommand("CheckBox PARTICLE_SYSTEM_GRID \"physxvis ePARTICLE_SYSTEM_GRID\"");
		mRenderDebug->sendRemoteCommand("CheckBox PARTICLE_SYSTEM_BROADPHASE_BOUNDS \"physxvis ePARTICLE_SYSTEM_BROADPHASE_BOUNDS\"");
		mRenderDebug->sendRemoteCommand("CheckBox PARTICLE_SYSTEM_MAX_MOTION_DISTANCE \"physxvis ePARTICLE_SYSTEM_MAX_MOTION_DISTANCE\"");

		mRenderDebug->sendRemoteCommand("EndGroup");


		mRenderDebug->sendRemoteCommand("BeginGroup \"Cloth Visualization\"");

		mRenderDebug->sendRemoteCommand("CheckBox CLOTH_VERTICAL \"physxvis eCLOTH_VERTICAL\"");
		mRenderDebug->sendRemoteCommand("CheckBox CLOTH_HORIZONTAL \"physxvis eCLOTH_HORIZONTAL\"");
		mRenderDebug->sendRemoteCommand("CheckBox CLOTH_BENDING \"physxvis eCLOTH_BENDING\"");
		mRenderDebug->sendRemoteCommand("CheckBox CLOTH_SHEARING \"physxvis eCLOTH_SHEARING\"");
		mRenderDebug->sendRemoteCommand("CheckBox CLOTH_VIRTUAL_PARTICLES \"physxvis eCLOTH_VIRTUAL_PARTICLES\"");
		mRenderDebug->sendRemoteCommand("CheckBox MBP_REGIONS \"physxvis eMBP_REGIONS\"");

		mRenderDebug->sendRemoteCommand("EndGroup");


		mRenderDebug->sendRemoteCommand("EndTab");
	}

	physx::PxVisualizationParameter::Enum getParameterFromString(const char *str)
	{
		physx::PxVisualizationParameter::Enum ret = physx::PxVisualizationParameter::eNUM_VALUES;

		if ( strcmp(str,"eSCALE") == 0 )
		{
			ret = physx::PxVisualizationParameter::eSCALE;
		}
		else if ( strcmp(str,"eWORLD_AXES") == 0 )
		{
			ret = physx::PxVisualizationParameter::eWORLD_AXES;
		}
		else if ( strcmp(str,"eBODY_AXES") == 0 )
		{
			ret = physx::PxVisualizationParameter::eBODY_AXES;
		}
		else if ( strcmp(str,"eBODY_MASS_AXES") == 0 )
		{
			ret = physx::PxVisualizationParameter::eBODY_MASS_AXES;
		}
		else if ( strcmp(str,"eBODY_LIN_VELOCITY") == 0 )
		{
			ret = physx::PxVisualizationParameter::eBODY_LIN_VELOCITY;
		}
		else if ( strcmp(str,"eBODY_ANG_VELOCITY") == 0 )
		{
			ret = physx::PxVisualizationParameter::eBODY_ANG_VELOCITY;
		}
		else if ( strcmp(str,"eBODY_JOINT_GROUPS") == 0 )
		{
			ret = physx::PxVisualizationParameter::eBODY_JOINT_GROUPS;
		}
		else if ( strcmp(str,"eCONTACT_POINT") == 0 )
		{
			ret = physx::PxVisualizationParameter::eCONTACT_POINT;
		}
		else if ( strcmp(str,"eCONTACT_NORMAL") == 0 )
		{
			ret = physx::PxVisualizationParameter::eCONTACT_NORMAL;
		}
		else if ( strcmp(str,"eCONTACT_ERROR") == 0 )
		{
			ret = physx::PxVisualizationParameter::eCONTACT_ERROR;
		}
		else if ( strcmp(str,"eCONTACT_FORCE") == 0 )
		{
			ret = physx::PxVisualizationParameter::eCONTACT_FORCE;
		}
		else if ( strcmp(str,"eACTOR_AXES") == 0 )
		{
			ret = physx::PxVisualizationParameter::eACTOR_AXES;
		}
		else if ( strcmp(str,"eCOLLISION_AABBS") == 0 )
		{
			ret = physx::PxVisualizationParameter::eCOLLISION_AABBS;
		}
		else if ( strcmp(str,"eCOLLISION_SHAPES") == 0 )
		{
			ret = physx::PxVisualizationParameter::eCOLLISION_SHAPES;
		}
		else if ( strcmp(str,"eCOLLISION_AXES") == 0 )
		{
			ret = physx::PxVisualizationParameter::eCOLLISION_AXES;
		}
		else if ( strcmp(str,"eCOLLISION_COMPOUNDS") == 0 )
		{
			ret = physx::PxVisualizationParameter::eCOLLISION_COMPOUNDS;
		}
		else if ( strcmp(str,"eCOLLISION_FNORMALS") == 0 )
		{
			ret = physx::PxVisualizationParameter::eCOLLISION_FNORMALS;
		}
		else if ( strcmp(str,"eCOLLISION_EDGES") == 0 )
		{
			ret = physx::PxVisualizationParameter::eCOLLISION_EDGES;
		}
		else if ( strcmp(str,"eCOLLISION_STATIC") == 0 )
		{
			ret = physx::PxVisualizationParameter::eCOLLISION_STATIC;
		}
		else if ( strcmp(str,"eCOLLISION_DYNAMIC") == 0 )
		{
			ret = physx::PxVisualizationParameter::eCOLLISION_DYNAMIC;
		}
		else if ( strcmp(str,"eCOLLISION_PAIRS") == 0 )
		{
//TODO			ret = physx::PxVisualizationParameter::eCOLLISION_PAIRS;
		}
		else if ( strcmp(str,"eJOINT_LOCAL_FRAMES") == 0 )
		{
			ret = physx::PxVisualizationParameter::eJOINT_LOCAL_FRAMES;
		}
		else if ( strcmp(str,"eJOINT_LIMITS") == 0 )
		{
			ret = physx::PxVisualizationParameter::eJOINT_LIMITS;
		}
		else if ( strcmp(str,"ePARTICLE_SYSTEM_POSITION") == 0 )
		{
			ret = physx::PxVisualizationParameter::ePARTICLE_SYSTEM_POSITION;
		}
		else if ( strcmp(str,"ePARTICLE_SYSTEM_VELOCITY") == 0 )
		{
			ret = physx::PxVisualizationParameter::ePARTICLE_SYSTEM_VELOCITY;
		}
		else if ( strcmp(str,"ePARTICLE_SYSTEM_COLLISION_NORMAL") == 0 )
		{
			ret = physx::PxVisualizationParameter::ePARTICLE_SYSTEM_COLLISION_NORMAL;
		}
		else if ( strcmp(str,"ePARTICLE_SYSTEM_BOUNDS") == 0 )
		{
			ret = physx::PxVisualizationParameter::ePARTICLE_SYSTEM_BOUNDS;
		}
		else if ( strcmp(str,"ePARTICLE_SYSTEM_GRID") == 0 )
		{
			ret = physx::PxVisualizationParameter::ePARTICLE_SYSTEM_GRID;
		}
		else if ( strcmp(str,"ePARTICLE_SYSTEM_BROADPHASE_BOUNDS") == 0 )
		{
			ret = physx::PxVisualizationParameter::ePARTICLE_SYSTEM_BROADPHASE_BOUNDS;
		}
		else if ( strcmp(str,"ePARTICLE_SYSTEM_MAX_MOTION_DISTANCE") == 0 )
		{
			ret = physx::PxVisualizationParameter::ePARTICLE_SYSTEM_MAX_MOTION_DISTANCE;
		}
		else if ( strcmp(str,"eCULL_BOX") == 0 )
		{
			ret = physx::PxVisualizationParameter::eCULL_BOX;
		}
		else if ( strcmp(str,"eCLOTH_VERTICAL") == 0 )
		{
			ret = physx::PxVisualizationParameter::eCLOTH_VERTICAL;
		}
		else if ( strcmp(str,"eCLOTH_HORIZONTAL") == 0 )
		{
			ret = physx::PxVisualizationParameter::eCLOTH_HORIZONTAL;
		}
		else if ( strcmp(str,"eCLOTH_BENDING") == 0 )
		{
			ret = physx::PxVisualizationParameter::eCLOTH_BENDING;
		}
		else if ( strcmp(str,"eCLOTH_SHEARING") == 0 )
		{
			ret = physx::PxVisualizationParameter::eCLOTH_SHEARING;
		}
		else if ( strcmp(str,"eCLOTH_VIRTUAL_PARTICLES") == 0 )
		{
			ret = physx::PxVisualizationParameter::eCLOTH_VIRTUAL_PARTICLES;
		}
		else if ( strcmp(str,"eMBP_REGIONS") == 0 )
		{
			ret = physx::PxVisualizationParameter::eMBP_REGIONS;
		}

		return ret;
	}

	bool processCommand(uint32_t count,const char **argv)
	{
		bool ret = false;

		if ( count )
		{
			const char *cmd = argv[0];
			if ( strcmp(cmd,"physxvis") == 0 )
			{
				ret = true;
				if ( count == 3 )
				{
					physx::PxVisualizationParameter::Enum p = getParameterFromString(argv[1]);
					const char *v = argv[2];
					float value = 0;
					if ( strcmp(v,"true") == 0 )
					{
						value = 1;
					}
					else
					{
						value = (float)atof(argv[2]);
					}
					for (DebugSceneVector::iterator i=mDebugScenes.begin(); i!=mDebugScenes.end(); ++i)
					{
						DebugScene &ds = (*i);
						ds.mScene->lockWrite(__FILE__,__LINE__);
						ds.mScene->setVisualizationParameter(p,value);
						ds.mScene->unlockWrite();
					}
				}
			}
			else if ( strcmp(cmd,"client_forward_one_frame") == 0 )
			{
				mOneFrameAdvance = true;
				mCurrentSimulationScale = 1.0/60.0f;
				ret = true;
			}
			else if ( strcmp(cmd,"client_pause") == 0 )
			{
				bool paused = true;
				ret = true;
				if ( count == 2 )
				{
					paused = strcmp(argv[1],"true") == 0;
				}
				if ( paused )
				{
					mCurrentSimulationScale = 0;
				}
				else
				{
					mCurrentSimulationScale = mSimulationScale;
				}
			}
			else if ( strcmp(cmd,"client_speed") == 0 )
			{
				ret = true;
				mSimulationScale = 1;
				if ( count == 2 )
				{
					mSimulationScale = (float)atof(argv[1]);
				}
				mCurrentSimulationScale = mSimulationScale;
			}
			else if ( strcmp(cmd,"mouseUp") == 0 )
			{
				ret = true;
				if ( mDraggingShape )
				{
					physx::PxBase *list[1];
					list[0] = static_cast< physx::PxBase *>(mDraggingShape);
					mPhysics->unregisterDeletionListenerObjects(*this,list,1); // notify the SDK we no longer care about hearing about this shape
				}
				mDraggingShape = NULL;
				mDraggingActor = NULL;
			}
			else if ( strcmp(cmd,"mouseDown") == 0 )
			{
				ret = true;
				if ( count == 15 )
				{
					physx::PxVec3 world;
					physx::PxVec3 dir;
					bool shiftKey;
					physx::PxVec2 mouseDir;

					world.x = (float)atof(argv[1]);
					world.y = (float)atof(argv[2]);
					world.z = (float)atof(argv[3]);

					dir.x = (float)atof(argv[4]);
					dir.y = (float)atof(argv[5]);
					dir.z = (float)atof(argv[6]);

					shiftKey = strcmp(argv[7],"true") == 0;

					mouseDir.x = (float)atof(argv[8]);
					mouseDir.y = (float)atof(argv[9]);

					bool leftMouseButton = strcmp(argv[10], "true") == 0;
					bool middleMouseButton = strcmp(argv[11], "true") == 0;
					bool rightMouseButton = strcmp(argv[12], "true") == 0;
					int32_t mouseX = atoi(argv[13]);
					int32_t mouseY = atoi(argv[14]);

					processMouse(world, dir, shiftKey, mouseDir, leftMouseButton, middleMouseButton, rightMouseButton, mouseX, mouseY);

				}
			}
			else if ( strcmp(cmd,"mouseWheel") == 0 )
			{
				ret = true;
				if ( count == 9 )
				{
					physx::PxI32 delta;
					bool isShift;
					physx::PxVec3 rayOrigin;
					physx::PxVec3 rayDir;

					delta = atoi( argv[1] );

					isShift = strcmp(argv[2],"true") == 0;

					rayOrigin.x = (float)atof(argv[3]);
					rayOrigin.y = (float)atof(argv[4]);
					rayOrigin.z = (float)atof(argv[5]);

					rayDir.x = (float)atof(argv[6]);
					rayDir.y = (float)atof(argv[7]);
					rayDir.z = (float)atof(argv[8]);

					processMouseWheel(delta,isShift,rayOrigin,rayDir);
				}
			}
		}

		return ret;
	}

	virtual	bool render(float dtime,bool showStatics,bool showDynamics,bool showCloth,bool flushRender)
	{
		if ( !mRenderDebug ) return false;
		if ( !mPhysics ) return false;

		if (mSimpleCamera)
		{
			mSimpleCamera->update(dtime);
		}

		mFrameCount++;

		mRenderDebug->debugText2D(0,0,0.5f,1.0f,false,0xFFFF00,"Application: %s Frame: %d", mApplicationName, mFrameCount );

		uint32_t argc;
		const char **argv = mRenderDebug->getRemoteCommand(argc);
		while ( argc )
		{
			bool snarfed = processCommand(argc,argv);
			if ( mCallback && !snarfed )
			{
				snarfed = mCallback->processDebugCommand(argc,argv);
			}
			argv = mRenderDebug->getRemoteCommand(argc);
		}

		if (mSimpleCamera && mSimpleCamera->getSpacebarSemaphore())
		{
			// ok we are going to shoot a sphere into the scene!
			const float *eyeDir = mSimpleCamera->getEyeDirection();
			const float *eyePos = mSimpleCamera->getEyePosition();
			physx::PxVec3 p(eyePos[0], eyePos[1], eyePos[2]);
			physx::PxVec3 dir(eyeDir[0], eyeDir[1], eyeDir[2]);
			shootSphere(SPHERE_RADIUS, p, dir);
		}

		for (DebugSceneVector::iterator i=mDebugScenes.begin(); i!=mDebugScenes.end(); ++i)
		{
			DebugScene &ds = (*i);
			ds.mScene->lockRead(__FILE__,__LINE__);
			float vscale = ds.mScene->getVisualizationParameter(physx::PxVisualizationParameter::eSCALE);
			ds.mScene->unlockRead();

			if ( vscale > 0 )
			{
				ds.mScene->lockRead(__FILE__,__LINE__);
				const physx::PxRenderBuffer &rb = ds.mScene->getRenderBuffer();
				debugVisualize(rb);
				ds.mScene->unlockRead();
			}
			if ( showStatics || showDynamics || showCloth )
			{
				ds.mRenderDebugScene->render(showStatics,showDynamics,showCloth);
			}
		}

		applyDragForce(); // if doing mouse dragging; apply the drag force

		if ( flushRender )
		{
			mRenderDebug->render(dtime,NULL);
		}

		return mActive;
	}

	void debugVisualize(const physx::PxRenderBuffer &rb)
	{
		mRenderDebug->pushRenderState();
		if ( rb.getNbLines() )
		{
			const physx::PxDebugLine *lines = rb.getLines();
			physx::PxU32 c1 = 0xFFFFFFFF;
			physx::PxU32 c2 = 0xFFFFFFFF;
			mRenderDebug->setCurrentColor(c1,c2);
			physx::PxU32 lcount = rb.getNbLines();
			for (physx::PxU32 i=0; i<lcount; i++)
			{
				if ( lines->color0 != c1 || lines->color1 != c2 )
				{
					mRenderDebug->setCurrentColor(lines->color0,lines->color1);
					c1 = lines->color0;
					c2 = lines->color1;
				}
				mRenderDebug->debugLine(lines->pos0,lines->pos1 );
				lines++;
			}
		}
		if ( rb.getNbTriangles() )
		{
			mRenderDebug->addToCurrentState(RENDER_DEBUG::DebugRenderState::SolidShaded);
			const physx::PxDebugTriangle *triangles = rb.getTriangles();
			for (physx::PxU32 i=0; i<rb.getNbTriangles(); i++)
			{
				const physx::PxDebugTriangle &t = triangles[i];
				mRenderDebug->debugGradientTri(t.pos0,t.pos1,t.pos2,t.color0,t.color1,t.color2);
			}
		}
		mRenderDebug->popRenderState();
	}

	bool isValid(void)
	{
		if ( !mRenderDebug ) return false;
		if ( mDebugScenes.empty() ) return false;
		return true;
	}

	virtual void release(void) 
	{
		delete this;
	}

	virtual float getSimulationScale(void)
	{
		float ret = mCurrentSimulationScale;
		if ( mOneFrameAdvance )
		{
			mCurrentSimulationScale = 0;
			mOneFrameAdvance = false;
		}
		return ret;
	}

	virtual bool isRemoteDissconnect(void) const
	{
		return !mActive;
	}

	virtual void setInterface(Interface *iface) 
	{
		mCallback = iface;
	}

	void processMouse(const physx::PxVec3 &world,
					  const physx::PxVec3 &dir,
					  bool shiftKey,
					  const physx::PxVec2 &mouseDir,
					  bool leftMouseButton,
					  bool middleMouseButton,
					  bool rightMouseButton,
					  int32_t	mouseX,
					  int32_t	mouseY)
	{
		if (!rightMouseButton )
			return;
		PX_UNUSED(shiftKey);
		PX_UNUSED(mouseDir);
		PX_UNUSED(leftMouseButton);
		PX_UNUSED(middleMouseButton);
		PX_UNUSED(rightMouseButton);
		PX_UNUSED(mouseX);
		PX_UNUSED(mouseY);
		physx::PxU32 scount = mPhysics->getNbScenes();
		if ( scount == 0 )
			return;
		physx::PxScene *scene = NULL;
		mPhysics->getScenes(&scene,1,0);
		if ( scene == NULL )
			return;

		if ( !mDraggingShape )
		{
			scene->lockRead(__FILE__,__LINE__);
			physx::PxHitFlags flags = physx::PxHitFlag::ePOSITION | physx::PxHitFlag::eNORMAL;
			physx::PxRaycastBuffer buffer;
			physx::PxQueryFilterData filterData;
			filterData.data.word0 = 0; 
			filterData.data.word1 = 0; 
			filterData.data.word2 = 0; 
			filterData.data.word3 = 0; 
			if ( scene->raycast(world,dir,1000,buffer,flags,filterData,this,NULL) )
			{
				physx::PxU32 hitCount = buffer.getNbAnyHits();
				if ( hitCount )
				{
					physx::PxRaycastHit hit = buffer.getAnyHit(0);
					physx::PxRigidActor *actor = hit.actor;
					if ( actor && actor->is<physx::PxRigidDynamic>() )
					{
						mDraggingShape = hit.shape;
						mDraggingActor = static_cast< physx::PxRigidDynamic *>(actor);

						physx::PxBase *list[1];
						list[0] = static_cast< physx::PxBase *>(mDraggingShape);
						mPhysics->registerDeletionListenerObjects(*this,list,1); // notify the SDK we need to know if this shape is deleted!

						mDragStartLocation = hit.position;
						mCurrentDragLocation = hit.position;
						mDragTo = hit.position;
						physx::PxRigidDynamic *rd = static_cast< physx::PxRigidDynamic *>(actor);
						physx::PxTransform pose = rd->getGlobalPose();
						physx::PxTransform ipose = pose.getInverse();
						mLocalDragStartLocation = ipose.transform(mDragStartLocation);
						physx::PxVec3 diff = world - mDragStartLocation;
						mDragStartDistance = diff.magnitude();
						mDragDirection = dir;
					}
				}
			}
			scene->unlockRead();
		}
		else
		{
			mDragDirection = dir;
			mCurrentDragLocation = world;
			mDragTo = mCurrentDragLocation+mDragDirection*mDragStartDistance;
		}
	}

	void applyDragForce(void)
	{
		if ( !mDraggingShape )
			return;

		physx::PxU32 scount = mPhysics->getNbScenes();
		if ( scount == 0 ) 
			return;

		physx::PxScene *scene = NULL;
		mPhysics->getScenes(&scene,1,0);
		if ( scene == NULL )
			return;
		scene->lockRead(__FILE__,__LINE__);
		physx::PxU32 flags = mDraggingActor->getRigidBodyFlags();
		scene->unlockRead();
		if ( flags & physx::PxRigidBodyFlag::eKINEMATIC)
		{
			return;
		}
		scene->lockWrite(__FILE__,__LINE__);
		RENDER_DEBUG::RenderDebug *renderDebug = mRenderDebug;
		physx::PxRigidDynamic *actor = mDraggingActor;
		mDragStartLocation = actor->getGlobalPose().transform(mLocalDragStartLocation);
		physx::PxF32 mass = actor->getMass();

		renderDebug->pushRenderState();
		renderDebug->setCurrentColor(0xFFFFFF,0xFF0000);
		renderDebug->debugRay(&mDragStartLocation.x,&mDragTo.x);
		renderDebug->popRenderState();

		physx::PxVec3 force = mDragTo - mDragStartLocation;
		force *= DRAG_FORCE;
		force*=mass;
		physx::PxRigidBodyExt::addForceAtLocalPos(*actor,force,mLocalDragStartLocation,physx::PxForceMode::eIMPULSE);

		scene->unlockWrite();
	}

	void processMouseWheel(physx::PxI32 delta,bool isShift,const physx::PxVec3 &rayOrigin,const physx::PxVec3 &rayDir)
	{
		PX_UNUSED(delta);
		PX_UNUSED(isShift);
		PX_UNUSED(rayOrigin);
		PX_UNUSED(rayDir);

		mDragStartDistance+=(physx::PxF32)delta*0.01f;
		mDragTo = mCurrentDragLocation+mDragDirection*mDragStartDistance;
	}


	virtual void onRelease(const physx::PxBase* observed, void* userData, physx::PxDeletionEventFlag::Enum deletionEvent) 
	{
		if ( observed->is<physx::PxShape>() )
		{
			const physx::PxShape *shape = static_cast< const physx::PxShape *>(observed);
			if ( shape == mDraggingShape )
			{
				mDraggingShape = NULL;
				mDraggingActor = NULL;
			}
		}
		PX_UNUSED(userData);
		PX_UNUSED(deletionEvent);
	}

	virtual bool isConnected(void) const 
	{
		bool ret = false;

		if ( mRenderDebug )
		{
			ret = mRenderDebug->getRunMode() == RENDER_DEBUG::RenderDebug::RM_CLIENT;
		}

		return ret;
	}

	void shootSphere(physx::PxF32 radius,physx::PxVec3 &p,const physx::PxVec3 &dir)
	{
		if ( mDebugScenes.empty() ) return;
		DebugScene ds = mDebugScenes[0];

		ds.mScene->lockWrite(__FILE__,__LINE__);

		if ( mDefaultMaterial == NULL )
		{
			mDefaultMaterial = mPhysics->createMaterial(0.5f,0.5f,0.1f);
		}

		physx::PxTransform pose(p);
		physx::PxRigidDynamic *rd = mPhysics->createRigidDynamic(pose);
		rd->setMass(SPHERE_MASS);
		physx::PxShape *shape=NULL;
		physx::PxSphereGeometry geom;
		geom.radius = radius;
		shape = rd->createShape(geom,*mDefaultMaterial);
		if ( shape )
		{
			shape->setLocalPose(physx::PxTransform(physx::PxIdentity));
			shape->setFlag(physx::PxShapeFlag::eVISUALIZATION,true);
			shape->setSimulationFilterData(mSimulationFilterData);
			shape->setQueryFilterData(mQueryFilterData);
		}
		physx::PxVec3 linearVelocity = dir*SPHERE_VELOCITY;
		rd->setLinearVelocity(linearVelocity);
		physx::PxVec3 rotationalVelocity(10,10,10);
		rd->setAngularVelocity(rotationalVelocity);
		rd->setLinearDamping(0.1f);
		ds.mScene->addActor(*rd);
		ds.mScene->unlockWrite();
	}

	virtual void setFilterData(const physx::PxFilterData &simData,physx::PxFilterData &queryData) 
	{
		mSimulationFilterData = simData;
		mQueryFilterData = queryData;
	}

	virtual physx::PxQueryHitType::Enum preFilter(const physx::PxFilterData& filterData, const physx::PxShape* shape, const physx::PxRigidActor* actor, physx::PxHitFlags& queryFlags) 
	{
		PX_UNUSED(queryFlags);
		PX_UNUSED(shape);
		PX_UNUSED(actor);
		PX_UNUSED(filterData);
		return physx::PxQueryHitType::eBLOCK;
	}
	virtual physx::PxQueryHitType::Enum postFilter(const physx::PxFilterData& filterData, const physx::PxQueryHit& hit) 
	{
		PX_UNUSED(filterData);
		PX_UNUSED(hit);
		return physx::PxQueryHitType::eBLOCK;
	}

	virtual bool continueWaitingForServer(uint32_t ms)
	{
		if ( (ms%1000) == 0 )
		{
			printf("Stalled %d seconds waiting for server.\r\n", ms/1000);
		}
		return true;
	}


	virtual void setRemoteCamera(const physx::PxVec3 &eye,const physx::PxVec3 &lookAt) 
	{
		if ( mRenderDebug )
		{
			mRenderDebug->sendRemoteCommand("lookat %0.4f %0.4f %0.4f   %0.4f %0.4f %0.4f", eye.x, eye.y, eye.z, lookAt.x, lookAt.y, lookAt.z );
		}
	}


	virtual void reportError(physx::PxErrorCode::Enum code, const char* message, const char* file, int line)
	{
		PX_UNUSED(code);
		if ( mRenderDebug )
		{
			mRenderDebug->debugMessage("PhysXError: %s : %s : %d\r\n", message, file, line );
		}
	}

	/**
	\brief If you need a rendering context for an additional scene, then create it here...
	*/
	virtual bool addRenderDebugScene(physx::PxScene *scene) 
	{
		if ( !mRenderDebug ) return false;
		for (DebugSceneVector::iterator i=mDebugScenes.begin(); i!=mDebugScenes.end(); ++i)
		{
			if ( (*i).mScene == scene ) // can't add a scene that has already been added!!!
			{
				return false; 
			}
		}
		DebugScene ds;
		ds.mScene = scene;
		ds.mRenderDebugScene = createRenderDebugScenePhysX(scene,mRenderDebug);
		if ( ds.mRenderDebugScene )
		{
			mDebugScenes.push_back(ds);
			return true;
		}
		return false;
	}

	/**
	\brief Remove this scene from debug rendering.
	*/
	virtual bool removeRenderDebugScene(physx::PxScene *scene) 
	{
		bool ret = false;

		for (DebugSceneVector::iterator i=mDebugScenes.begin(); i!=mDebugScenes.end(); ++i)
		{
			if ( (*i).mScene == scene ) // can't add a scene that has already been added!!!
			{
				mDebugScenes.erase(i);
				ret = true;
				break;
			}
		}

		return ret;
	}

	virtual RENDER_DEBUG::RenderDebug *getRenderDebug(void) const
	{
		return mRenderDebug;
	}

	virtual RENDER_DEBUG::RenderDebugTyped *getRenderDebugTyped(void) const 
	{
		return mRenderDebug ? mRenderDebug->getRenderDebugTyped() : NULL;
	}

	virtual void notifyModifySamples(physx::PxHeightField *hf,uint32_t subColStart,uint32_t subRowStart,uint32_t nbSubCols,uint32_t nbSubRows) 
	{
		for (size_t i=0; i<mDebugScenes.size(); i++)
		{
			mDebugScenes[i].mRenderDebugScene->notifyModifySamples(hf,subColStart,subRowStart,nbSubCols,nbSubRows);
		}
	}

	// return pointer to the currently selected/dragging actor
	virtual physx::PxRigidActor *getSelectedActor(void) const final
	{
		return mDraggingActor;
	}

private:
	physx::PxFilterData			mSimulationFilterData;
	physx::PxFilterData			mQueryFilterData;
	bool						mActive;
	bool						mOneFrameAdvance;
	float						mCurrentSimulationScale;
	float						mSimulationScale;
	uint32_t					mFrameCount;
	physx::PxPhysics			*mPhysics;

	DebugSceneVector			mDebugScenes;

	Interface					*mCallback;
	physx::PxRigidDynamic		*mDraggingActor;
	physx::PxShape				*mDraggingShape;
	physx::PxVec3				mDragStartLocation;
	physx::PxVec3				mCurrentDragLocation;
	physx::PxVec3				mDragTo;
	physx::PxVec3				mLocalDragStartLocation;
	physx::PxF32				mDragStartDistance;
	physx::PxVec3				mDragDirection;
	physx::PxMaterial			*mDefaultMaterial;
	bool						mOwnRenderDebug;
	RENDER_DEBUG::RenderDebugTyped	*mRenderDebug;
	const char					*mApplicationName{ "RenderDebugPhysX" };
	SimpleCamera				*mSimpleCamera{ nullptr };
};

} // end of RENDER_DEUBG_PHYSX namespace

using namespace RENDER_DEBUG_PHYSX;

RenderDebugPhysX *createRenderDebugPhysX(const RenderDebugPhysX::Desc &desc)
{
	RenderDebugPhysXImpl *ret = new RenderDebugPhysXImpl(desc);
	if ( !ret->isValid() )
	{
		ret->release();
		ret = NULL;
	}
	return static_cast< RenderDebugPhysX *>(ret);
}

RenderDebugScenePhysX *createRenderDebugScenePhysX(physx::PxScene *scene,RENDER_DEBUG::RenderDebugTyped *shapeRenderInterface)
{
	MyRenderDebugScenePhysX *psr = new MyRenderDebugScenePhysX(scene,shapeRenderInterface);
	return static_cast< RenderDebugScenePhysX *>(psr);
}

RenderDebugPhysX *createRenderDebugPhysX(physx::PxScene *s,	// The PhysX scene to render
										RENDER_DEBUG::RenderDebug *renderDebug,
										bool allowRegistry)
{
	RenderDebugPhysXImpl *ret = NULL;
	if ( renderDebug )
	{
		ret = new RenderDebugPhysXImpl(s,renderDebug->getRenderDebugTyped(),allowRegistry);
		if ( !ret->isValid() )
		{
			ret->release();
			ret = NULL;
		}
	}
	return static_cast< RenderDebugPhysX *>(ret);
}

} // end of namespace
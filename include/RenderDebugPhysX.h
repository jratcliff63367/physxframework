#ifndef RENDER_DEBUG_PHYSX_H

#define RENDER_DEBUG_PHYSX_H

#ifndef _INTPTR
#define _INTPTR 0
#endif

#include "stdint.h"

#include "foundation/PxErrorCallback.h"


namespace physx
{
	class PxVec3;
	class PxScene;
	class PxRigidActor;
	class PxHeightField;
	struct PxFilterData;
}

namespace RENDER_DEBUG
{
	class RenderDebug;
	class RenderDebugTyped;
}

namespace NV_PHYSX_FRAMEWORK
{

	class RenderDebugPhysX : public physx::PxErrorCallback
	{
	public:
		class Desc
		{
		public:
			Desc(void)
			{
				scene = 0;
				fileOk = false;
				allowRegistry = false;
				useDebugDLL = false;
				recordFileName = "RenderDebugPhysX.rec";
				hostName = "localhost";
				applicationName = "RenderDebugPhysX";
				recordRemoteCommands = 0;
				playbackRemoteCommands = 0;
			}
			/**
			\brief The PhysX scene to render.
			*/
			physx::PxScene	*scene;
			/**
			\brief Whether or not it is ok to record the debug visualization to a file on disk.
			*/
			bool			fileOk;
			/**
			\brief Whether or not to allow all of the debug visualization settings to be recorded to the registry and retained between multiple runs of the app
			*/
			bool			allowRegistry;
			/**
			\brief Whether or not the debug version of the DLL or shared-object file should be loaded; default is false.
			*/
			bool			useDebugDLL;
			/**
			\brief If file recording is enabled, then this is the name of the file to record to.
			*/
			const char		*recordFileName;
			/**
			\brief The name of the remote host to connect to.
			*/
			const char		*hostName;
			/**
			\brief The name of this application; this is important to set as all registry settins are saved relative to this application name
			*/
			const char		*applicationName;
			/**
			\brief Whether or not to record all remote commands which came from the server.  This is a debugging feature. If not null then this is the name of the file to record to.
			*/
			const char		*recordRemoteCommands;
			/**
			\brief Whether or not to play back previously recorded file containing remote commands.  This is a debugging feature. If not null then this is the name of the previously recorded file to play back from.
			*/
			const char		*playbackRemoteCommands;
		};
		class Interface
		{
		public:
			/**
			*\brief Optional callback to the application to process an arbitrary console command.

			This allows the application to process an incoming command from the server.  If the application consumes the command, then it will not be passed on
			to the rest of the default processing.  Return true to indicate that you consumed the command, false if you did not.

			\return Return true if your application consumed the command, return false if it did not.
			*/
			virtual bool processDebugCommand(uint32_t argc, const char **argv) = 0;
		protected:
			virtual ~Interface(void) { }
		};

		virtual void setInterface(Interface *iface) = 0;

		/**
		* \brief Cause a remote debug visualization of the physx system to be rendered.  If this returns true, then keep simulating.  If it returns false, than the server has requested an exit.
		*/
		virtual	bool render(float dtime, bool showStatics, bool showDynamics, bool showCloth, bool flushRender) = 0;

		/**
		* \brief Returns the timing scale factor to use on the simulation.  0 means pause.
		*/
		virtual float getSimulationScale(void) = 0;

		/**
		* \brief Return's true if the server requested a remote disconnect.
		*/
		virtual bool isRemoteDissconnect(void) const = 0;

		/**
		*\brief Return's true if we have a remote connection to the server
		*/
		virtual bool isConnected(void) const = 0;

		/**
		\brief Set the simulation filter data and query filter data fields which should be used when generating a sphere to be shot into the scene.  Default values are all zero bits
		*/
		virtual void setFilterData(const physx::PxFilterData &simData, physx::PxFilterData &queryData) = 0;

		/**
		\brief Sets the camera position and look at location on the DebugView app
		*/
		virtual void setRemoteCamera(const physx::PxVec3 &eye, const physx::PxVec3 &lookAt) = 0;

		/**
		\brief If you need a rendering context for an additional scene, then create it here...
		*/
		virtual bool addRenderDebugScene(physx::PxScene *scene) = 0;

		/**
		\brief Remove this scene from debug rendering.
		*/
		virtual bool removeRenderDebugScene(physx::PxScene *scene) = 0;

		virtual RENDER_DEBUG::RenderDebug *getRenderDebug(void) const = 0;

		virtual RENDER_DEBUG::RenderDebugTyped *getRenderDebugTyped(void) const = 0;

		virtual void notifyModifySamples(physx::PxHeightField *hf, uint32_t subColStart, uint32_t subRowStart, uint32_t nbSubCols, uint32_t nbSubRows) = 0;

		// return pointer to the currently selected/dragging actor
		virtual physx::PxRigidActor *getSelectedActor(void) const = 0;

		/**
		*\brief Release the RenderDebugPhysX interface
		*/
		virtual void release(void) = 0;

	protected:
		virtual ~RenderDebugPhysX(void)
		{
		}
	};

	/**
	*\brief creates a render debug interface relative to this scene.
	*/
	RenderDebugPhysX *createRenderDebugPhysX(const RenderDebugPhysX::Desc &desc);

	RenderDebugPhysX *createRenderDebugPhysX(physx::PxScene *s,	// The PhysX scene to render
		RENDER_DEBUG::RenderDebug *renderDebug,
		bool allowRegistry);


} // end of namespace

#endif


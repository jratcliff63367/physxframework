#include "NvCTypes.h"

#if NV_WINDOWS_FAMILY

#include <Windows.h>
#include "NvPreprocessor.h"
#include "NvPhysXFramework.h"

#pragma warning(disable:4100 4127)


extern "C"
{

__declspec(dllexport) NV_PHYSX_FRAMEWORK::PhysXFramework *createPhysXFrameworkExport(uint32_t version,const char *dllName)
{
	return NV_PHYSX_FRAMEWORK::createPhysXFramework(version,dllName);
};


};

BOOL APIENTRY DllMain( HANDLE ,
					  DWORD ul_reason_for_call,
					  LPVOID )
{
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
		break;
	case DLL_THREAD_ATTACH:
		break;
	case DLL_THREAD_DETACH:
		break;
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}
#endif
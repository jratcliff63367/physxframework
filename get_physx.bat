@rem This batch file grabs the PhysX SDK binaries; DLLs and Libs
@echo off
set BIN32=F:\p4sw\sw\physx\PhysXSDK\3.4\trunk\Bin\vc14win32
set BIN64=F:\p4sw\sw\physx\PhysXSDK\3.4\trunk\Bin\vc14win32
set LIB32=F:\p4sw\sw\physx\PhysXSDK\3.4\trunk\Lib\vc14win32
set LIB64=F:\p4sw\sw\physx\PhysXSDK\3.4\trunk\Lib\vc14win64
set SHARED32=F:\p4sw\sw\physx\PxShared\1.0\trunk\lib\vc14win32
set SHARED64=F:\p4sw\sw\physx\PxShared\1.0\trunk\lib\vc14win64
cd bin
cd win32
copy %BIN32%\nvToolsExt32_1.dll
copy %BIN32%\PhysX3CharacterKinematicCHECKED_x86.dll
copy %BIN32%\PhysX3CHECKED_x86.dll
copy %BIN32%\PhysX3CommonCHECKED_x86.dll
copy %BIN32%\PhysX3CookingCHECKED_x86.dll
copy %BIN32%\PhysX3GpuCHECKED_x86.dll
copy %BIN32%\PhysXDevice.dll
copy %BIN32%\PxFoundationCHECKED_x86.dll
copy %BIN32%\PxPvdSDKCHECKED_x86.dll
copy %BIN32%\PhysX3CharacterKinematicDEBUG_x86.dll
copy %BIN32%\PhysX3DEBUG_x86.dll
copy %BIN32%\PhysX3CommonDEBUG_x86.dll
copy %BIN32%\PhysX3CookingDEBUG_x86.dll
copy %BIN32%\PhysX3GpuDEBUG_x86.dll
copy %BIN32%\PhysXDevice.dll
copy %BIN32%\PxFoundationDEBUG_x86.dll
copy %BIN32%\PxPvdSDKDEBUG_x86.dll
cd ..
cd win64
copy %BIN64%\nvToolsExt32_1.dll
copy %BIN64%\PhysX3CharacterKinematicCHECKED_x86.dll
copy %BIN64%\PhysX3CHECKED_x86.dll
copy %BIN64%\PhysX3CommonCHECKED_x86.dll
copy %BIN64%\PhysX3CookingCHECKED_x86.dll
copy %BIN64%\PhysX3GpuCHECKED_x86.dll
copy %BIN64%\PhysXDevice.dll
copy %BIN64%\PxFoundationCHECKED_x86.dll
copy %BIN64%\PxPvdSDKCHECKED_x86.dll
copy %BIN64%\PhysX3CharacterKinematicDEBUG_x86.dll
copy %BIN64%\PhysX3DEBUG_x86.dll
copy %BIN64%\PhysX3CommonDEBUG_x86.dll
copy %BIN64%\PhysX3CookingDEBUG_x86.dll
copy %BIN64%\PhysX3GpuDEBUG_x86.dll
copy %BIN64%\PhysXDevice.dll
copy %BIN64%\PxFoundationDEBUG_x86.dll
copy %BIN64%\PxPvdSDKDEBUG_x86.dll
cd ..
cd ..

cd ext
cd physx
cd lib
cd win32
copy %LIB32%\PhysX3CommonCHECKED_x86.lib
copy %LIB32%\PhysX3CookingCHECKED_x86.lib
copy %LIB32%\PhysX3CHECKED_x86.lib
copy %LIB32%\PhysX3GpuCHECKED_x86.lib
copy %LIB32%\PhysX3ExtensionsCHECKED.lib
copy %SHARED32%\PsFastXmlCHECKED_x86.lib
copy %SHARED32%\PxCudaContextManagerCHECKED_x86.lib
copy %SHARED32%\PxFoundationCHECKED_x86.lib
copy %SHARED32%\PxPvdSDKCHECKED_x86.lib
copy %SHARED32%\PxTaskCHECKED_x86.lib
copy %LIB32%\PhysX3CommonDEBUG_x86.lib
copy %LIB32%\PhysX3CookingDEBUG_x86.lib
copy %LIB32%\PhysX3DEBUG_x86.lib
copy %LIB32%\PhysX3GpuDEBUG_x86.lib
copy %LIB32%\PhysX3ExtensionsDEBUG.lib
copy %SHARED32%\PsFastXmlDEBUG_x86.lib
copy %SHARED32%\PxCudaContextManagerDEBUG_x86.lib
copy %SHARED32%\PxFoundationDEBUG_x86.lib
copy %SHARED32%\PxPvdSDKDEBUG_x86.lib
copy %SHARED32%\PxTaskDEBUG_x86.lib
cd ..
cd win64
copy %LIB64%\PhysX3CommonCHECKED_x64.lib
copy %LIB64%\PhysX3CookingCHECKED_x64.lib
copy %LIB64%\PhysX3CHECKED_x64.lib
copy %LIB64%\PhysX3GpuCHECKED_x64.lib
copy %LIB64%\PhysX3ExtensionsCHECKED.lib
copy %SHARED64%\PsFastXmlCHECKED_x64.lib
copy %SHARED64%\PxCudaContextManagerCHECKED_x64.lib
copy %SHARED64%\PxFoundationCHECKED_x64.lib
copy %SHARED64%\PxPvdSDKCHECKED_x64.lib
copy %SHARED64%\PxTaskCHECKED_x64.lib
copy %LIB64%\PhysX3CommonDEBUG_x64.lib
copy %LIB64%\PhysX3CookingDEBUG_x64.lib
copy %LIB64%\PhysX3DEBUG_x64.lib
copy %LIB64%\PhysX3GpuDEBUG_x64.lib
copy %LIB64%\PhysX3ExtensionsDEBUG.lib
copy %SHARED64%\PsFastXmlDEBUG_x64.lib
copy %SHARED64%\PxCudaContextManagerDEBUG_x64.lib
copy %SHARED64%\PxFoundationDEBUG_x64.lib
copy %SHARED64%\PxPvdSDKDEBUG_x64.lib
copy %SHARED64%\PxTaskDEBUG_x64.lib
cd ..
cd ..
cd ..
cd ..


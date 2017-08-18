@echo off

set XPJ="xpj4.exe"

%XPJ% -v 1 -t VC14 -p WIN32 -x SimpleHelloWorld.xpj
%XPJ% -v 1 -t VC14 -p WIN64 -x SimpleHelloWorld.xpj

cd ..
cd vc14win64

goto cleanExit

:pauseExit
pause

:cleanExit


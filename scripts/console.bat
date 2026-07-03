@echo off

setlocal
call win32env.bat

start "ODM Console" cmd /k "echo  ____________________________ && echo /   ____    _____    __  __  \ && echo ^|  / __ \  ^|  __ \  ^|  \/  ^| ^| && echo ^| ^| ^|  ^| ^| ^| ^|  ^| ^| ^| \  / ^| ^| && echo ^| ^| ^|  ^| ^| ^| ^|  ^| ^| ^| ^|\/^| ^| ^| && echo ^| ^| ^|__^| ^| ^| ^|__^| ^| ^| ^|  ^| ^| ^| && echo ^|  \____/  ^|_____/  ^|_^|  ^|_^| ^| && echo \____________________________/ && @echo off && FOR /F %%i in (VERSION) do echo        version: %%i && @echo on && echo. && run --help

endlocal

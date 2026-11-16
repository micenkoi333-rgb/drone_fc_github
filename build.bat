@echo off
chcp 65001 >nul
cd /d "%~dp0"
rem build.bat — збирає консольний симулятор, тести й графічний інтерфейс (drone_gui.exe). `build.bat nolaunch` — без запуску вікна.

set "VSDIR=D:\Visual Studio"
if not exist "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" (
    for /f "usebackq tokens=*" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -property installationPath` 2^>nul) do set "VSDIR=%%i"
)
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 (
    echo Не вдалося підключити Visual Studio. Перевір, що встановлено "Розробка класичних програм на C++".
    pause
    exit /b 1
)

echo === Збираю консольний симулятор ===
cl /nologo /std:c++17 /O2 /EHsc /utf-8 main.cpp /Fe:drone_sim.exe || goto :fail
echo === Збираю тести ===
cl /nologo /std:c++17 /O2 /EHsc /utf-8 tests\test_main.cpp /Fe:run_tests.exe || goto :fail
del /q *.obj 2>nul

echo === Збираю графічний інтерфейс ===
call "%~dp0build_gui.bat" || goto :fail

echo.
echo Готово!
if /i "%~1"=="nolaunch" exit /b 0
start "" drone_gui.exe
exit /b 0

:fail
echo.
echo ПОМИЛКА компіляції — скинь цей текст.
pause
exit /b 1
@echo off
chcp 65001 >nul
cd /d "%~dp0"
rem train_pilot.bat — навчає ШІ-пілота з нуля на всіх ядрах (довго: години). Можна закрити вікно будь-коли:
rem найкраща мережа вже збережена в ai\policy_hover.txt, а наступний запуск продовжить з неї.
rem   train_pilot.bat          300 поколінь
rem   train_pilot.bat 1000     1000 поколінь
rem   train_pilot.bat eval     лише порівняти зі звичайним контролером

set "VSDIR=D:\Visual Studio"
if not exist "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" (
    for /f "usebackq tokens=*" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -property installationPath` 2^>nul) do set "VSDIR=%%i"
)
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 (
    echo Не вдалося підключити Visual Studio.
    pause
    exit /b 1
)
cl /nologo /std:c++17 /O2 /EHsc /utf-8 tools\ai_rl.cpp /Fe:ai_rl.exe || goto :fail
del /q *.obj 2>nul
set "N=%~1"
if "%N%"=="" set "N=300"
if /i "%N%"=="eval" goto :eval
ai_rl.exe train %N% cont
:eval
ai_rl.exe eval
echo.
pause
exit /b 0
:fail
echo Помилка збирання.
pause
exit /b 1

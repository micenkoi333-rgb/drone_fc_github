@echo off
chcp 65001 >nul
cd /d "%~dp0"
rem train_tricks.bat — ШІ вчиться пілотажних трюків на ЦЬОМУ комп'ютері (без вікна; те саме — у програмі, вкладка «Трюки»).
rem   train_tricks.bat               усі вісім трюків по 100 спроб
rem   train_tricks.bat tumble 300    лише один трюк: loop, stall, roll, hammerhead, harrier, cobra, flip, tumble
rem   train_tricks.bat roll 100 new  з нуля (типово — продовжує з навченого в ai\tricks_learned.ini, гіршим не стане)
rem   Кожна спроба летить у штиль і ще в одних із 3 умов з вітром до 6 м/с і вантажем — навчене не підігнане під штиль.
rem   Рахує на всіх ядрах; 4-й аргумент — скільки ядер (0 — усі, 1 — тихо на одному).
rem Результат: ai\tricks_learned.ini (для QP-5 — ai\tricks_learned_qp5.ini; вчиться активний апарат з aircraft\active.txt) і data\tricks_*.csv.
set "WHICH=%~1"
if "%WHICH%"=="" set "WHICH=all"
set "N=%~2"
if "%N%"=="" set "N=100"
set "MODE=%~3"
if "%MODE%"=="" set "MODE=cont"
set "CORES=%~4"
if "%CORES%"=="" set "CORES=0"
set "VSDIR=D:\Visual Studio"
if not exist "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" (
    for /f "usebackq tokens=*" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -property installationPath` 2^>nul) do set "VSDIR=%%i"
)
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul || ( echo Не вдалося підключити Visual Studio & pause & exit /b 1 )
if not exist data mkdir data
cl /nologo /std:c++17 /O2 /EHsc /utf-8 tools\ai_tricks.cpp /Fe:ai_tricks.exe || ( echo ПОМИЛКА компіляції & pause & exit /b 1 )
ai_tricks.exe %WHICH% %N% %RANDOM% %MODE% %CORES% 3
pause

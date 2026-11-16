@echo off
chcp 65001 >nul
cd /d "%~dp0"
rem train_qp_ai.bat — перенавчає ШІ під АКТИВНИЙ апарат (aircraft\active.txt), усе підряд на всіх ядрах:
rem   1) пілотажні трюки (усі вісім, по 100 спроб, починаючи з навченого раніше);
rem   2) ШІ-пілот v2 — утримання точки у вітрі (150 поколінь; можна закрити вікно будь-коли — найкраща мережа вже збережена);
rem   3) порівняння зі звичайним автопілотом і перезбірка симулятора.
rem   train_qp_ai.bat 300      — 300 поколінь пілота замість 150
rem Модель сил навчає окремо train_ai.bat.
set "GENS=%~1"
if "%GENS%"=="" set "GENS=150"
set "VSDIR=D:\Visual Studio"
if not exist "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" (
    for /f "usebackq tokens=*" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -property installationPath` 2^>nul) do set "VSDIR=%%i"
)
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul || ( echo Не вдалося підключити Visual Studio & pause & exit /b 1 )
if not exist data mkdir data
echo === Збираю інструменти ===
cl /nologo /std:c++17 /O2 /EHsc /utf-8 tools\ai_tricks.cpp /Fe:ai_tricks.exe || goto :fail
cl /nologo /std:c++17 /O2 /EHsc /utf-8 tools\ai_rl.cpp /Fe:ai_rl.exe || goto :fail
del /q *.obj 2>nul
echo.
echo === 1/3 Трюки ===
ai_tricks.exe all 100 %RANDOM% cont 0 3 > data\train_tricks_log.txt
type data\train_tricks_log.txt | findstr /c:"Найкраща спроба" /c:"== "
echo.
echo === 2/3 ШІ-пілот v2: %GENS% поколінь ===
ai_rl.exe train2 %GENS% cont
echo.
echo === 3/3 Порівняння і перезбірка ===
ai_rl.exe eval2 > data\pilot2_eval.txt
type data\pilot2_eval.txt
call build.bat nolaunch
echo Готово. Результати: data\train_tricks_log.txt, data\pilot2_eval.txt
pause
exit /b 0
:fail
echo ПОМИЛКА збирання
pause
exit /b 1

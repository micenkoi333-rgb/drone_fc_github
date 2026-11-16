@echo off
chcp 65001 >nul
cd /d "%~dp0"
rem Навчає для АКТИВНОГО апарата (aircraft\active.txt): для QP-5 ваги йдуть в ai\weights_force_qp5.h, старий апарат не зачіпається.
rem train_ai.bat — навчання нейромережі «модель сил» на ЦЬОМУ комп'ютері: дані з симулятора → навчання → порівняння → перезбірка.
rem   train_ai.bat              120 польотів (≈ 5.5 год польоту), 40 епох — як навчено зараз
rem   train_ai.bat 300 60       300 польотів, 60 епох — точніше, довше
rem   train_ai.bat keep 80      не генерувати дані заново (взяти data\part_*.bin), лише перенавчити, 80 епох
rem   (мережі з пам'яттю потрібні нові дані — 26 чисел у рядку; старі data\part_*.bin згенеруйте заново)
rem Потрібні: Visual Studio (як для build.bat), Python 3.9+ з numpy (pip install numpy).

set "FLIGHTS=%~1"
if "%FLIGHTS%"=="" set "FLIGHTS=120"
set "EPOCHS=%~2"
if "%EPOCHS%"=="" set "EPOCHS=40"
set "KEEP="
if /i "%FLIGHTS%"=="keep" ( set "KEEP=--keep" & set "FLIGHTS=0" )

where python >nul 2>nul || ( echo Не знайдено Python. Встановіть з python.org і поставте галочку "Add to PATH". & pause & exit /b 1 )
python -c "import numpy" 2>nul || ( echo Встановлюю numpy... & python -m pip install numpy || ( echo Не вдалося встановити numpy & pause & exit /b 1 ) )

set "VSDIR=D:\Visual Studio"
if not exist "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" (
    for /f "usebackq tokens=*" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -property installationPath` 2^>nul) do set "VSDIR=%%i"
)
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul || ( echo Не вдалося підключити Visual Studio & pause & exit /b 1 )

if "%KEEP%"=="" (
    echo === Збираю генератор даних ===
    cl /nologo /std:c++17 /O2 /EHsc /utf-8 tools\ai_datagen.cpp /Fe:ai_datagen.exe || goto :fail
)
echo === Генерація даних і навчання (процесів: %NUMBER_OF_PROCESSORS%) ===
python tools\ai_pipeline.py --flights %FLIGHTS% --jobs %NUMBER_OF_PROCESSORS% --epochs %EPOCHS% %KEEP% || goto :fail

echo === Порівняння «без ШІ / з ШІ» з новими вагами (6 польотів, ≈ 15–30 хв; Ctrl+C — пропустити) ===
cl /nologo /std:c++17 /O2 /EHsc /utf-8 tools\ai_eval.cpp /Fe:ai_eval.exe || goto :fail
ai_eval.exe 6

echo === Перезбираю симулятор з новими вагами ===
call build.bat nolaunch
echo Готово. Нові ваги — ai\weights_force.h (основний апарат) або ai\weights_force_qp5.h (QP-5, якщо він активний); у програмі: «ШІ: модель сил».
pause
exit /b 0
:fail
echo ПОМИЛКА — див. повідомлення вище.
pause
exit /b 1

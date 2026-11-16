@echo off
chcp 65001 >nul
cd /d "%~dp0"
rem run_tests.bat — збирає тести й запускає всі групи паралельно на всіх ядрах (вивід кожної групи — у tests_out\).
rem   run_tests.bat            усі групи, 12 прогонів Монте-Карло
rem   run_tests.bat 4          лише 4 ядра (решта — для роботи)
rem   run_tests.bat 0 testRoute   одна група (її назва або частина назви), без паралелі

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
echo === Збираю тести ===
cl /nologo /std:c++17 /O2 /EHsc /utf-8 tests\test_main.cpp /Fe:run_tests.exe || goto :fail
del /q *.obj 2>nul
if not "%~2"=="" (
    run_tests.exe 12 %2
) else (
    run_tests.exe 12 -j %1
)
echo.
pause
exit /b 0
:fail
echo Помилка збирання.
pause
exit /b 1

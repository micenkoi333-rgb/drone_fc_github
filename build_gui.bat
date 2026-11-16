@echo off
rem build_gui.bat — збирає drone_gui.exe (GLFW + ImGui + ImPlot + модель QuadPlane). Бібліотеки ImGui/ImPlot компілюються один раз у build\imgui_implot.lib.
rem Компілятор: Visual Studio (як у build.bat).
cd /d "%~dp0"
set "VSDIR=D:\Visual Studio"
if not exist "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" (
    for /f "usebackq tokens=*" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -property installationPath`) do set "VSDIR=%%i"
)
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 ( echo Не вдалось підключити Visual Studio & exit /b 1 )
if not exist build mkdir build
if not exist build\imgui_implot.lib (
    echo === Збираю ImGui + ImPlot ^(один раз^) ===
    cl /nologo /std:c++17 /O2 /EHsc /MD /utf-8 /c /Fobuild\ /Ithird_party\imgui /Ithird_party\implot /Ithird_party\glfw\include ^
       third_party\imgui\imgui.cpp third_party\imgui\imgui_draw.cpp third_party\imgui\imgui_tables.cpp third_party\imgui\imgui_widgets.cpp ^
       third_party\imgui\backends\imgui_impl_glfw.cpp third_party\imgui\backends\imgui_impl_opengl3.cpp ^
       third_party\implot\implot.cpp third_party\implot\implot_items.cpp || goto :fail
    lib /nologo /out:build\imgui_implot.lib build\imgui.obj build\imgui_draw.obj build\imgui_tables.obj build\imgui_widgets.obj build\imgui_impl_glfw.obj build\imgui_impl_opengl3.obj build\implot.obj build\implot_items.obj || goto :fail
)
echo === Збираю drone_gui.exe ===
cl /nologo /std:c++17 /O2 /EHsc /MD /utf-8 /DNOMINMAX /Fobuild\ /Ithird_party\imgui /Ithird_party\implot /Ithird_party\glfw\include gui\main_gui.cpp ^
   /Fe:drone_gui.exe /link build\imgui_implot.lib third_party\glfw\lib\glfw3.lib opengl32.lib gdi32.lib user32.lib shell32.lib winmm.lib || goto :fail
echo Готово: drone_gui.exe ^(запускати з цієї теки^)
exit /b 0
:fail
echo ПОМИЛКА компіляції
exit /b 1
@echo off
setlocal
if "%QT_DIR%"=="" (
  echo Please set QT_DIR first, for example:
  echo set QT_DIR=C:\Qt\6.8.3\mingw_64
  exit /b 1
)
cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH="%QT_DIR%"
if errorlevel 1 exit /b 1
cmake --build build --config Release
endlocal

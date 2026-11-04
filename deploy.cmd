@echo off
rem Double-click entry point for deploy.ps1: transfers the project to the
rem robot and rebuilds the docker stack (everything deploy.ps1 does by
rem default). Any arguments are forwarded, e.g.:
rem     deploy.cmd -NoRebuild
rem     deploy.cmd -UseKey
chcp 65001 >nul
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0deploy.ps1" %*
set RC=%ERRORLEVEL%
echo.
if "%RC%"=="0" echo Deploy completed successfully.
if not "%RC%"=="0" echo Deploy FAILED with exit code %RC%.
echo Press any key to close...
pause
exit /b %RC%

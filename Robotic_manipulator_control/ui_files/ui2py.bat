@echo off
title .UI to .py files converter

echo ""
echo UI file Name
set /p UiName=Enter .UI file Name: 
echo ""
echo ""
echo PY file Name
set /p PyName=Enter .PY file Name: 
echo ""

echo Start Converting Files Please wait.

call pyuic5 -x "%UiName%" -o "%PyName%"

echo Files Converted.
pause
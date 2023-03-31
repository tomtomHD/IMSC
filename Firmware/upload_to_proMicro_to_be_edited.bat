@echo off
setlocal

for /f "tokens=1* delims==" %%I in ('wmic path win32_pnpentity get caption  /format:list ^| find "Arduino Leonardo"') do (
    call :resetCOM "%%~J"
)

:continue

ping 127.255.255.255 -n 1 -w 1000>nul

:: wmic /format:list strips trailing spaces (at least for path win32_pnpentity)
for /f "tokens=1* delims==" %%I in ('wmic path win32_pnpentity get caption  /format:list ^| find "Arduino Leonardo bootloader"') do (
    call :setCOM "%%~J"
)


:: end main batch
goto :EOF

:resetCOM <WMIC_output_line>
:: sets _COM#=line
setlocal
set "str=%~1"
set "num=%str:*(COM=%"
set "num=%num:)=%"
set port=COM%num%
echo %port%
mode %port%: BAUD=1200 parity=N data=8 stop=1
goto :continue

:setCOM <WMIC_output_line>
:: sets _COM#=line
setlocal
set "str=%~1"
set "num=%str:*(COM=%"
set "num=%num:)=%"
set port=COM%num%
echo %port%
goto :flash

:flash
"Absolute_Path_To...\avrdude.exe" -C"Absolute_Path_To...\avrdude.conf" -v -patmega32u4 -cavr109 -P%port% -b57600 -D -Uflash:w:"Absolute_Path_To...\EIT_Arduino_V3.hex":i
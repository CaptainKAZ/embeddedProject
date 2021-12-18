cd /d %~dp0
chdir
xcopy ".\Middlewares\Third_Party\FreeRTOS\Source\portable\GCC\ARM_CM7\r0p1\" ".\Middlewares\Third_Party\FreeRTOS\Source\portable\RVDS\ARM_CM7\r0p1" /Y

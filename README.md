# cvbs3
Testbed project for the next revision of WekaOSD https://wekaosd.home.blog/ which will be a full-colour open source OSD system 
for radio-controlled aircraft and drones.

This project implements the code to lock to a CSYNC signal (as provided by a sync separator IC such as the LM1881)
and generate a rainbow test pattern, which will be output on GPIO pins F0 to F6, where pins 0-1 are red, 2-3 are green, 4-5 are blue
and pin 6 is the pixel switch to enable/disable the overlay.

CSYNC is input to pin PA9, and PC6 will output the pixel clock for scoping.

The pixel bus (F0-F5) is suitable for connecting to the RGB inputs of a composite video encoder such as the AD724 or AD725, after 
converting to analog line levels by resistor DACs.

The code currently runs on an STM32F413 Nucleo board (https://www.st.com/en/evaluation-tools/nucleo-f413zh.html).

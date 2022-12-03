# TDisplayS3_mandelbrot

This is a demonstration of using htcw_gfx with the LCD Panel API in the ESP-IDF

You don't use a driver in this case. You draw everything to a bitmap, and then send that bitmap to the display.

It's less than ideal in some cases, because it's not as flexible, but that's due to limitations of the LCD Panel API itself.

In this demonstration we use a frame buffer that's 1/5 of the total screen height so it will fit in SRAM.

Then it gets sent to the display, drawn over, sent to the display, drawn over, etc. 5 times to make 1 full screen.

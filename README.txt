uQuad source files.

These files implement the Minima quat-copter OS.
They are still under development and subject to change.
You may use them as is and/or change them to suit your needs
at your own risk.

This code is distributed under the GNU Public License
which can be found at http://www.gnu.org/licenses/gpl.txt

external dependencies:
This code relies heavily on the brilliant, free AVRlib written by Pascal Stang.
You will need to point your project file to it.
It can be downloaded here: http://www.procyonengineering.com/embedded/avr/avrlib

files:

ctrl.c		This is the meat. It implements the balancing (in fact several approaches)
global.h	Hardware definition, pin layouts, settings, etc.
i2cconf.h	i2c settings used by avrlib
ITG3200.c	gyro driver
lis3lv02_spi.c	accel driver. spi version
QuadCtrl.c	main loop and initialisation

QuadCtrl.aps	AvrStudio project files
QuadCtrl.aws	

include /c/PROGRA~1/Quanser/QUARC/include/linux_x64_tools.mk
all:
	$(CC) $(SYSROOT) -std=gnu99 -O0 -march=x86-64 -static -o /d/QDrone/QDrone_Control/QDrone_RuntimeProbe.rt-linux_x64 /d/QDrone/QDrone_Control/capture_tool/QDrone_RuntimeProbe.c


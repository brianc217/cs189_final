#************** Example Makefile ***********************
#
# 2/26/2005
# Manual makefile to go with Eclipse
#   NOTE: enviornment variables must be set to find linker & compiler.
#   NOTE: runs with GNU Make. Might also run with Microsoft nmake
# with tweaks.
#
# This Makefile will compile any .c file in the current directory and link the
# objects into a .hex file to load into the robot.
#
# Rename this file "makefile" and put in your C source code folder.
#
# The OBJFILES variable is filled with .o targets using the wildcard
# command and patsubst. The $(wildcard *.c) will retrieve any .c
# file in the current directory and store it in a variable. The patsubstr
# functions is used to convert a file from one format to another. In
# this case each .c file is converted into a .o extension and then
# stored into OBJFILES. This variable is then used to compile each .c file.
# The rule %.o: %.c rebuilds any .o file if the cooresponding .c file has
# changed.
# In the compile line you will see two variables $@ and $<.
# $@ will match the target and the $< will match the dependency. So, for
# example, $< will contain main.c whenever $@ contains main.o.
# Note: makefiles are counterintuitive in that the rules don't run in the order
# listed in the file, but instead run whenever they are matched.
# Also be careful with tabs and spaces. Rules need a tab (not spaces)
# before each action.

#RENAME AS NEEDED
PROJECT=lab2

CC = xc16-gcc
BIN2HEX = xc16-bin2hex

PICFLAG=-mcpu=30F6014A
DEBUG = -D__DEBUG -g
CFLAGS = -I. -Ia2d/ -Ibluetooth/ -Imotor_led/ -Imotor_led/advance_one_timer/ -Iuart/ -Wall
GLD = p30f6014A.gld

E_PUCK = 2117
EPUCKUPLOAD = epuckuploadbt/epuckuploadbt

# source files
INC := $(patsubst %.c,%.o,$(wildcard *.h)) $(patsubst %.c,%.o,$(wildcard */*.h))
OBJFILES := $(patsubst %.c,%.o,$(wildcard *.c))
I2C := $(patsubst %.c,%.o,$(wildcard I2C/*.c))
CAMERA := $(patsubst %.c,%.o,$(wildcard camera/*/*.c)) $(patsubst %.s,%.o,$(wildcard camera/*/*.s))
A2D := $(patsubst %.c,%.o,$(wildcard a2d/*.c))
BLUETOOTH := $(patsubst %.c,%.o,$(wildcard bluetooth/*.c))
MOTOR_LED := $(patsubst %.c,%.o,$(wildcard motor_led/*.c)) $(patsubst %.c,%.o,$(wildcard motor_led/*/*.c))
UART := $(patsubst %.s,%.o,$(wildcard uart/*.s))

# targets
all: upload

upload: $(PROJECT).hex
	$(EPUCKUPLOAD) $^ $(E_PUCK)

$(PROJECT): $(PROJECT).hex

#re-link if any object file changed
$(PROJECT).hex: $(PROJECT).cof
	$(BIN2HEX) $^

$(PROJECT).cof: $(OBJFILES) $(A2D) $(BLUETOOTH) $(MOTOR_LED) $(UART) $(CAMERA) $(I2C)
	$(CC) $(PICFLAG) $^ -o $@ -Wl,--script=$(GLD),--heap=512,-Map=$(PROJECT).map,--report-mem

# Recompile a file if it's c-file changes,
# OR recompile everything if ANY header file changes
%.o: %.c $(INC)
	$(CC) $(PICFLAG) -x c -c $< -o $@ $(CFLAGS) $(DEBUG)

%.o: %.s $(INC)
	$(CC) $(PICFLAG) -c $< -o $@ $(CFLAGS) $(DEBUG)

#delete all the build files so you can start from scratch.
clean:
	$(RM) $(OBJFILES)
	$(RM) $(A2D)
	$(RM) $(BLUETOOTH)
	$(RM) $(MOTOR_LED)
	$(RM) $(UART)
	$(RM) $(CAMERA)
	$(RM) $(I2C)
	$(RM) $(PROJECT).hex
	$(RM) $(PROJECT).map
	$(RM) $(PROJECT).lst
	$(RM) $(PROJECT).cof


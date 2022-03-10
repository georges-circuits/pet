#!/bin/sh

echo "//THIS FILE GETS OVERWRITTEN BY A SCRIPT THAT RUNS AFTER THE CUBEMX CODE GENERATOR" > Core/Src/init.c
cat Core/Src/main.c >> Core/Src/init.c
sed -i 's/#include "main.h"/#include "init.h"/g' Core/Src/init.c
rm Core/Src/main.c

echo "//THIS FILE GETS OVERWRITTEN BY A SCRIPT THAT RUNS AFTER THE CUBEMX CODE GENERATOR" > Core/Inc/pin_defs.h
egrep "^#define [A-Z]" Core/Inc/main.h >> Core/Inc/pin_defs.h
rm Core/Inc/main.h

sed -i 's/#include "main.h"/#include "init.h"/g' Core/Src/stm32l0xx_hal_msp.c
sed -i 's/#include "main.h"/#include "init.h"/g' Core/Src/stm32l0xx_it.c


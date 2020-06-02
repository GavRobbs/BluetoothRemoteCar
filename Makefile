F_CPU = 1000000UL
BAUD = 9600UL

build_car: car.c
	avr-gcc -mmcu=atmega328p -DF_CPU=$(F_CPU) -DBAUD=$(BAUD) -Wall -Os -o car.elf car.c

hex_car: car.elf
	avr-objcopy -j .text -j .data -O ihex car.elf car.hex

flash_car: car.hex
	avrdude -p m328 -c usbtiny -e -U flash:w:car.hex

do_car: car.c
	avr-gcc -mmcu=atmega328p -DF_CPU=$(F_CPU) -DBAUD=$(BAUD) -Wall -Os -o car.elf car.c
	avr-objcopy -j .text -j .data -O ihex car.elf car.hex
	avrdude -p m328 -c usbtiny -e -U flash:w:car.hex
	rm car.elf car.hex

build_remote: remote.c
	avr-gcc -mmcu=atmega328p -DF_CPU=$(F_CPU) -DBAUD=$(BAUD) -Wall -Os -o remote.elf remote.c

hex_remote: remote.elf
	avr-objcopy -j .text -j .data -O ihex remote.elf remote.hex

flash_remote: remote.hex
	avrdude -p m328 -c usbtiny -e -U flash:w:remote.hex

do_remote: remote.c
	avr-gcc -mmcu=atmega328p -DF_CPU=$(F_CPU) -DBAUD=$(BAUD) -Wall -Os -o remote.elf remote.c
	avr-objcopy -j .text -j .data -O ihex remote.elf remote.hex
	avrdude -p m328 -c usbtiny -e -U flash:w:remote.hex
	rm remote.elf remote.hex
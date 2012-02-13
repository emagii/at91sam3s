#!/bin/sh
#PROBE=jlink
APPLICATIONS=~/projects/Applications
CROSS_COMPILE=arm-none-eabi-

OPEN_OCD_VERSION=0.5.0
OPEN_OCD=openocd-${OPEN_OCD_VERSION}

LIBFTDI_VERSION=0.18
LIBFTDI=libftdi-${LIBFTDI_VERSION}

OCD_PORT=4444

XPROBE=olimex-arm-usb-ocd-h
XBOARD=atmel_sam3s_ek

OCD_SCRIPTS=/usr/share/openocd/scripts
OCD_PROBE=${OCD_SCRIPTS}/interface/${XPROBE}.cfg
OCD_BOARD=${OCD_SCRIPTS}/board/${XBOARD}.cfg


alias	armddd='ddd --debugger arm-none-eabi-gdb'
alias	ocd='openocd -f ${OCD_PROBE} -f ${OCD_BOARD} -s /usr/share/openocd'
#  -s ${BOARD_SCRIPT}
add	()
{
	sudo	apt-get	install	$1
}

install_ocd ()
{
	add	openocd
	# Version 0.18-1build1
	add	libftdi1
	add	libftdu-dev
	add	libusb-dev
	add	libusb-1.0-0
}

build_libftdi ()
{
	mkdir	-p	${APPLICATIONS}
	cd 		${APPLICATIONS}
	wget		http://www.intra2net.com/en/developer/${LIBFTDI}.tar.gz
	tar	zxf	${LIBFTDI}.tar.gz
	cd		${LIBFTDI}
	./configure	--prefix=/usr
	make	-j 4
	sudo	make	install
}

build_openocd ()
{
	mkdir	-p	${APPLICATIONS}
	cd 		${APPLICATIONS}
	wget		http://downloads.sourceforge.net/project/openocd/openocd/${OPEN_OCD_VERSION}/${OPEN_OCD}.tar.bz2
	tar	jxvf	${OPEN_OCD}.tar.bz2
	cd		${OPEN_OCD}
	./configure	--prefix=/usr	--enable-ft2232_libftdi
	make		-j 4
	sudo	make	install
}

ocd_init ()
{
	openocd -f ${OCD_PROBE} -f ${OCD_BOARD} -s /usr/share/openocd
}

ocd_connect ()
{
	telnet	localhost	${OCD_PORT}
	help
	halt
	
}

ocd_load ()
{
	flash	probe	0
	flash	info	0
	flash	write_bank	0	<app>	0
	flash write_bank 0 spi_dataflash_sam3s_ek_sam3s4-flash.bin 0
	step		<addr>
}

armdbg ()
{
	ddd	--debugger	${CROSS_COMPILE}gdb	&	
	target remote localhost:3333

}

if !	[ "x$1" == "x"	] ; then
	$1
fi


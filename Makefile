LIB=./libraries

LIBDIR=../lib
INCDIR=../include

LIBRARIES=$(LIBDIR)/.install

INCLUDES=				\
	$(INCDIR)/.usb			\
	$(INCDIR)/.fat			\
	$(INCDIR)/.mem			\
	$(INCDIR)/.jpeg			\
	$(INCDIR)/.touch		\
	$(INCDIR)/.cmsis		\
	$(INCDIR)/.chip			\
	$(INCDIR)/.pirrd		\
	$(INCDIR)/.gui_widgets		\
	$(INCDIR)/.gui_wgt_core		\
	$(INCDIR)/.gui_wgt_frontends	\
	$(INCDIR)/.gui_file		\
	$(INCDIR)/.gui_common		\
	$(INCDIR)/.gui_porting		\
	$(INCDIR)/.gui_disp		\
	$(INCDIR)/.gui			\
	$(INCDIR)/.FreeRTOS		\
	$(INCDIR)/.CoOS			\
	$(INCDIR)/.board		\
	$(INCDIR)/at91lib.h

BUILD	+= $(HOME)/bin/cs-rm


TARGETS += $(LIBRARIES)
TARGETS += $(INCLUDES)

all:	$(BUILD)
	(cd	build/gcc ; make)

install:	$(TARGETS)
	
$(HOME)/bin/cs-rm:
	mkdir	-p	$(HOME)/bin
	ln	-s	/bin/rm	~/bin/cs-rm

$(LIBRARIES):
	mkdir	-p	$(LIBDIR)
	cp $(LIB)/usb/lib/libusb_sam3s_gcc_dbg.a				$(LIBDIR)
	cp $(LIB)/usb/lib/libusb_sam3s_gcc_rel.a				$(LIBDIR)
	cp $(LIB)/memories/lib/libmemories_sam3s_gcc_dbg.a			$(LIBDIR)
	cp $(LIB)/memories/lib/libmemories_sam3s_gcc_rel.a			$(LIBDIR)
	cp $(LIB)/libjpeg/lib/libjpeg_CM3_gcc_rel.a				$(LIBDIR)
	cp $(LIB)/libjpeg/lib/libjpeg_CM3_gcc_dbg.a				$(LIBDIR)
	cp $(LIB)/libqtouch/lib/libsam3s-32qt-k-8rs-gnu.a			$(LIBDIR)
	cp $(LIB)/libqtouch/lib/libsam3s-32qt-k-8rs-iar.a			$(LIBDIR)
	cp $(LIB)/libqtouch/lib/libqtouch_sam3s_gcc_rel.a			$(LIBDIR)
	cp $(LIB)/libqtouch/lib/libqtouch_sam3s_ewarm_rel.a			$(LIBDIR)
	cp $(LIB)/libchip_sam3s/lib/libchip_sam3s4_gcc_dbg.a			$(LIBDIR)
	cp $(LIB)/libchip_sam3s/lib/libchip_sam3s4_gcc_rel.a			$(LIBDIR)
	cp $(LIB)/libchip_sam3s/lib/libchip_sam3s1_gcc_rel.a			$(LIBDIR)
	cp $(LIB)/libchip_sam3s/lib/libchip_sam3s2_gcc_dbg.a			$(LIBDIR)
	cp $(LIB)/libchip_sam3s/lib/libchip_sam3s1_gcc_dbg.a			$(LIBDIR)
	cp $(LIB)/libchip_sam3s/lib/libchip_sam3s2_gcc_rel.a			$(LIBDIR)
	cp $(LIB)/libboard_sam3s-ek/lib/libboard_sam3s_ek_gcc_dbg.a		$(LIBDIR)
	cp $(LIB)/libboard_sam3s-ek/lib/libboard_sam3s_ek_gcc_rel.a		$(LIBDIR)
	touch	$@

$(INCDIR)/at91lib.h:
	cp $(LIB)/at91lib.h							$(INCDIR)

$(INCDIR)/.usb:
	mkdir	-p	$(INCDIR)/usb/include
	mkdir	-p	$(INCDIR)/usb/device/ccid
	cp $(LIB)/usb/include/DUALCDCDDriver.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/AUDDSpeakerDriver.h				$(INCDIR)/usb/include
	cp $(LIB)/usb/include/HIDDFunction.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/HIDReports.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/MSDLun.h						$(INCDIR)/usb/include
	cp $(LIB)/usb/include/USBDescriptors.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/USBLib_Trace.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/USBLib_Types.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/AUDDStream.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/AUDDSpeakerPhone.h				$(INCDIR)/usb/include
	cp $(LIB)/usb/include/USBDDriver.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/CDCRequests.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/HIDRequests.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/HIDDescriptors.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/AUDDFunction.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/AUDDSpeakerPhoneDriver.h				$(INCDIR)/usb/include
	cp $(LIB)/usb/include/MSDDriver.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/AUDRequests.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/HIDUsages.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/HIDDKeyboard.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/CDCNotifications.h				$(INCDIR)/usb/include
	cp $(LIB)/usb/include/CDCAUDDDriver.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/MSD.h						$(INCDIR)/usb/include
	cp $(LIB)/usb/include/HIDDMouseDriver.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/HIDAUDDDriver.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/MSDescriptors.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/USBD.h						$(INCDIR)/usb/include
	cp $(LIB)/usb/include/CDCDSerialDriver.h				$(INCDIR)/usb/include
	cp $(LIB)/usb/include/CDCHIDDDriver.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/MSDFunction.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/MSDDStateMachine.h				$(INCDIR)/usb/include
	cp $(LIB)/usb/include/SBC.h						$(INCDIR)/usb/include
	cp $(LIB)/usb/include/MSDIOFifo.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/HIDMSDDriver.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/AUDDescriptors.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/USBRequests.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/USBD_HAL.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/CDCMSDDriver.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/CDCDescriptors.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/SBCMethods.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/CDCDSerialPort.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/CDCDSerial.h					$(INCDIR)/usb/include
	cp $(LIB)/usb/include/HIDDKeyboardDriver.h				$(INCDIR)/usb/include
	cp $(LIB)/usb/include/HIDDTransferDriver.h				$(INCDIR)/usb/include
	cp $(LIB)/usb/device/ccid/cciddriver.h					$(INCDIR)/usb/device/ccid
	cp $(LIB)/usb/device/ccid/cciddriverdescriptors.h			$(INCDIR)/usb/device/ccid
	cp $(LIB)/usb/usb.h							$(INCDIR)/usb
	touch	$@

$(INCDIR)/.fat:
	mkdir	-p	$(INCDIR)/fat
	cp $(LIB)/fat/fatfs/src/integer.h					$(INCDIR)/fat
	cp $(LIB)/fat/fatfs/src/ffconf.h					$(INCDIR)/fat
	cp $(LIB)/fat/fatfs/src/diskio.h					$(INCDIR)/fat
	cp $(LIB)/fat/fatfs/src/ff.h						$(INCDIR)/fat
	ln	-s	.							$(INCDIR)/fat/fatfs
	ln	-s	.							$(INCDIR)/fat/src
	touch	$@

$(INCDIR)/.mem:
	mkdir	-p	$(INCDIR)/mem/include
	cp $(LIB)/memories/include/ManagedNandFlash.h				$(INCDIR)/mem/include
	cp $(LIB)/memories/include/NandSpareScheme.h				$(INCDIR)/mem/include
	cp $(LIB)/memories/include/MEDSdmmc.h					$(INCDIR)/mem/include
	cp $(LIB)/memories/include/NandFlashModel.h				$(INCDIR)/mem/include
	cp $(LIB)/memories/include/MEDFlash.h					$(INCDIR)/mem/include
	cp $(LIB)/memories/include/SkipBlockNandFlash.h				$(INCDIR)/mem/include
	cp $(LIB)/memories/include/at26d.h					$(INCDIR)/mem/include
	cp $(LIB)/memories/include/MEDSdram.h					$(INCDIR)/mem/include
	cp $(LIB)/memories/include/Media.h					$(INCDIR)/mem/include
	cp $(LIB)/memories/include/sdio.h					$(INCDIR)/mem/include
	cp $(LIB)/memories/include/NandFlashModelList.h				$(INCDIR)/mem/include
	cp $(LIB)/memories/include/NandCommon.h					$(INCDIR)/mem/include
	cp $(LIB)/memories/include/NorFlashCommon.h				$(INCDIR)/mem/include
	cp $(LIB)/memories/include/MEDNandFlash.h				$(INCDIR)/mem/include
	cp $(LIB)/memories/include/sdmmc_cmd.h					$(INCDIR)/mem/include
	cp $(LIB)/memories/include/MappedNandFlash.h				$(INCDIR)/mem/include
	cp $(LIB)/memories/include/MEDRamDisk.h					$(INCDIR)/mem/include
	cp $(LIB)/memories/include/NorFlashCFI.h				$(INCDIR)/mem/include
	cp $(LIB)/memories/include/MEDSdcard.h					$(INCDIR)/mem/include
	cp $(LIB)/memories/include/MEDDdram.h					$(INCDIR)/mem/include
	cp $(LIB)/memories/include/NorFlashApi.h				$(INCDIR)/mem/include
	cp $(LIB)/memories/include/TranslatedNandFlash.h			$(INCDIR)/mem/include
	cp $(LIB)/memories/include/sdmmc.h					$(INCDIR)/mem/include
	cp $(LIB)/memories/include/EccNandFlash.h				$(INCDIR)/mem/include
	cp $(LIB)/memories/include/at26.h					$(INCDIR)/mem/include
	cp $(LIB)/memories/include/RawNandFlash.h				$(INCDIR)/mem/include
	cp $(LIB)/memories/include/NorFlashAmd.h				$(INCDIR)/mem/include
	cp $(LIB)/memories/include/NorFlashIntel.h				$(INCDIR)/mem/include
	cp $(LIB)/memories/memories.h						$(INCDIR)/mem
	touch	$@

$(INCDIR)/.jpeg:
	mkdir	-p	$(INCDIR)/jpeg/include
	cp $(LIB)/libjpeg/jconfig.h						$(INCDIR)/jpeg/include
	cp $(LIB)/libjpeg/include/jmorecfg.h					$(INCDIR)/jpeg/include
	cp $(LIB)/libjpeg/include/jinclude.h					$(INCDIR)/jpeg/include
	cp $(LIB)/libjpeg/include/jconfig.h					$(INCDIR)/jpeg/include
	cp $(LIB)/libjpeg/include/jpeglib.h					$(INCDIR)/jpeg/include
	cp $(LIB)/libjpeg/include/cderror.h					$(INCDIR)/jpeg/include
	cp $(LIB)/libjpeg/include/jerror.h					$(INCDIR)/jpeg/include
	cp $(LIB)/libjpeg/include/jversion.h					$(INCDIR)/jpeg/include
	cp $(LIB)/libjpeg/include/jdct.h					$(INCDIR)/jpeg/include
	cp $(LIB)/libjpeg/include/jmemsys.h					$(INCDIR)/jpeg/include
	cp $(LIB)/libjpeg/include/jpegint.h					$(INCDIR)/jpeg/include
	cp $(LIB)/libjpeg/include/cdjpeg.h					$(INCDIR)/jpeg/include
	cp $(LIB)/libjpeg/include/transupp.h					$(INCDIR)/jpeg/include
	cp $(LIB)/libjpeg/jconfig_gcc.h						$(INCDIR)/jpeg/include
	cp $(LIB)/libjpeg/jconfig_iar.h						$(INCDIR)/jpeg/include
	cp $(LIB)/libjpeg/libjpeg.h						$(INCDIR)/jpeg
	touch	$@

$(INCDIR)/.touch:
	mkdir	-p	$(INCDIR)/touch/include
	cp $(LIB)/libqtouch/include/touch_qt_config.h				$(INCDIR)/touch/include
	cp $(LIB)/libqtouch/include/touch_api.h					$(INCDIR)/touch/include
	cp $(LIB)/libqtouch/libqtouch.h						$(INCDIR)/touch
	touch	$@

$(INCDIR)/.cmsis:
	mkdir	-p	$(INCDIR)/cmsis
	cp $(LIB)/libchip_sam3s/cmsis/CMSIS_Core.htm				$(INCDIR)/cmsis
	cp $(LIB)/libchip_sam3s/cmsis/core_cm3.h				$(INCDIR)/cmsis
	touch	$@

$(INCDIR)/.chip:
	mkdir	-p	$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/spi.h					$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/pmc.h					$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/wdt.h					$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/flashd.h				$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/USBD_Config.h				$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/crccu.h					$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/hsmci.h					$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/ssc.h					$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/acc.h					$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/supc.h					$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/USBD_LEDs.h				$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/spi_pdc.h				$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/rtc.h					$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/async.h					$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/usart.h					$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/twid.h					$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/rtt.h					$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/pio_it.h				$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/tc.h					$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/dacc.h					$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/twi.h					$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/adc.h					$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/efc.h					$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/pio_capture.h				$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/pwmc.h					$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/trace.h					$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/pio.h					$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/SAM3S.h					$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/include/exceptions.h				$(INCDIR)/chip/include
	cp $(LIB)/libchip_sam3s/chip.h						$(INCDIR)/chip
	touch	$@

$(INCDIR)/.pirrd:
	mkdir	-p	$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/timetick.h			$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/syscalls.h			$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/lcd_gimp_image.h			$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/hamming.h			$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/bmp.h				$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/omnivision_ov7740.h		$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/lcd_font10x14.h			$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/lcd_draw.h			$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/tsd.h				$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/led.h				$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/rand.h				$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/tsd_ads7843.h			$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/ili9325.h			$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/lcd_color.h			$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/ov7740.h				$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/board_lowlevel.h			$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/board_memories.h			$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/bitbanding.h			$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/uart_console.h			$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/clock.h				$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/wav.h				$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/lcd_font.h			$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/math.h				$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/ads7843.h			$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/wm8731.h				$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/tsd_com.h			$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/lcdd.h				$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/include/OV7740_Register_table.h		$(INCDIR)/pirrd
	cp $(LIB)/libboard_sam3s-pirrd/board.h					$(INCDIR)/pirrd
	touch	$@

$(INCDIR)/.gui_widgets:
	mkdir	-p	$(INCDIR)/gui/wgt/widgets
	cp $(LIB)/sam-gui/source/wgt/widgets/wgt_widget_page.h			$(INCDIR)/gui/wgt/widgets
	cp $(LIB)/sam-gui/source/wgt/widgets/wgt_widget_button.h		$(INCDIR)/gui/wgt/widgets
	cp $(LIB)/sam-gui/source/wgt/widgets/wgt_widget_static.h		$(INCDIR)/gui/wgt/widgets
	touch	$@

$(INCDIR)/.gui_wgt_core:
	mkdir	-p	$(INCDIR)/gui/wgt/core
	cp $(LIB)/sam-gui/source/wgt/core/wgt_core_pointer.h			$(INCDIR)/gui/wgt/core
	cp $(LIB)/sam-gui/source/wgt/core/wgt_core_behaviour.h			$(INCDIR)/gui/wgt/core
	cp $(LIB)/sam-gui/source/wgt/core/wgt_core_message.h			$(INCDIR)/gui/wgt/core
	cp $(LIB)/sam-gui/source/wgt/core/wgt_core_timer.h			$(INCDIR)/gui/wgt/core
	cp $(LIB)/sam-gui/source/wgt/core/wgt_core_frontend.h			$(INCDIR)/gui/wgt/core
	cp $(LIB)/sam-gui/source/wgt/core/wgt_core_widget.h			$(INCDIR)/gui/wgt/core
	cp $(LIB)/sam-gui/source/wgt/core/wgt_core.h				$(INCDIR)/gui/wgt/core
	cp $(LIB)/sam-gui/source/wgt/core/wgt_core_screen.h			$(INCDIR)/gui/wgt/core
	touch	$@

$(INCDIR)/.gui_wgt_frontends:
	mkdir	-p	$(INCDIR)/gui/wgt/frontends
	cp $(LIB)/sam-gui/source/wgt/frontends/frontend_qtouch.h		$(INCDIR)/gui/wgt/frontends
	cp $(LIB)/sam-gui/source/wgt/frontends/frontend_ADS7843.h		$(INCDIR)/gui/wgt/frontends
	cp $(LIB)/sam-gui/source/wgt/frontends/frontend_potentiometer.h		$(INCDIR)/gui/wgt/frontends
	cp $(LIB)/sam-gui/source/wgt/frontends/frontend_pushbuttons.h		$(INCDIR)/gui/wgt/frontends
	touch	$@

$(INCDIR)/.gui_file:
	mkdir	-p	$(INCDIR)/gui/file
	cp $(LIB)/sam-gui/source/file/fatfs_config.h				$(INCDIR)/gui/file
	cp $(LIB)/sam-gui/source/file/file_fs.h					$(INCDIR)/gui/file
	touch	$@

$(INCDIR)/.gui_common:
	mkdir	-p	$(INCDIR)/gui/common
	cp $(LIB)/sam-gui/source/common/sam_gui_point.h				$(INCDIR)/gui/common
	cp $(LIB)/sam-gui/source/common/sam_gui_font10x14.h			$(INCDIR)/gui/common
	cp $(LIB)/sam-gui/source/common/sam_gui_errors.h			$(INCDIR)/gui/common
	cp $(LIB)/sam-gui/source/common/sam_gui_color.h				$(INCDIR)/gui/common
	cp $(LIB)/sam-gui/source/common/sam_gui_size.h				$(INCDIR)/gui/common
	cp $(LIB)/sam-gui/source/common/sam_gui_colors.h			$(INCDIR)/gui/common
	cp $(LIB)/sam-gui/source/common/sam_gui_bitmap_raw.h			$(INCDIR)/gui/common
	cp $(LIB)/sam-gui/source/common/sam_gui_font.h				$(INCDIR)/gui/common
	touch	$@


$(INCDIR)/.gui_porting:
	mkdir	-p	$(INCDIR)/gui/porting
	cp $(LIB)/sam-gui/source/porting/sam_gui_porting.h			$(INCDIR)/gui/porting
	touch	$@

$(INCDIR)/.gui_disp:
	mkdir	-p	$(INCDIR)/gui/disp
	mkdir	-p	$(INCDIR)/gui/disp/backends/HX8347
	mkdir	-p	$(INCDIR)/gui/disp/backends/ILI9325
	cp $(LIB)/sam-gui/source/disp/backends/HX8347/backend_HX8347.h		$(INCDIR)/gui/disp/backends/HX8347
	cp $(LIB)/sam-gui/source/disp/backends/ILI9325/backend_ILI9325.h	$(INCDIR)/gui/disp/backends/ILI9325
	cp $(LIB)/sam-gui/source/disp/disp_core.h				$(INCDIR)/gui/disp
	cp $(LIB)/sam-gui/source/disp/disp_backend.h				$(INCDIR)/gui/disp
	touch	$@

$(INCDIR)/.gui:
	mkdir	-p	$(INCDIR)/gui
	cp $(LIB)/sam-gui/libsam_gui.h						$(INCDIR)/gui
	ln	-s	.							$(INCDIR)/gui/source
	touch	$@

$(INCDIR)/.FreeRTOS:
	mkdir	-p	$(INCDIR)/FreeRTOS
	cp $(LIB)/rtos/FreeRTOS/FreeRTOSConfig.h				$(INCDIR)/FreeRTOS
	cp $(LIB)/rtos/FreeRTOS/libfreertos.h					$(INCDIR)/FreeRTOS
	cp $(LIB)/rtos/FreeRTOS/Source/portable/GCC/ARM_CM3/portmacro.h		$(INCDIR)/FreeRTOS
	cp $(LIB)/rtos/FreeRTOS/Source/portable/GCC/ARM_CM3_MPU/portmacro.h	$(INCDIR)/FreeRTOS
	cp $(LIB)/rtos/FreeRTOS/Source/portable/IAR/ARM_CM3/portmacro.h		$(INCDIR)/FreeRTOS
	cp $(LIB)/rtos/FreeRTOS/Source/include/queue.h				$(INCDIR)/FreeRTOS
	cp $(LIB)/rtos/FreeRTOS/Source/include/mpu_wrappers.h			$(INCDIR)/FreeRTOS
	cp $(LIB)/rtos/FreeRTOS/Source/include/task.h				$(INCDIR)/FreeRTOS
	cp $(LIB)/rtos/FreeRTOS/Source/include/semphr.h				$(INCDIR)/FreeRTOS
	cp $(LIB)/rtos/FreeRTOS/Source/include/projdefs.h			$(INCDIR)/FreeRTOS
	cp $(LIB)/rtos/FreeRTOS/Source/include/portable.h			$(INCDIR)/FreeRTOS
	cp $(LIB)/rtos/FreeRTOS/Source/include/list.h				$(INCDIR)/FreeRTOS
	cp $(LIB)/rtos/FreeRTOS/Source/include/StackMacros.h			$(INCDIR)/FreeRTOS
	cp $(LIB)/rtos/FreeRTOS/Source/include/croutine.h			$(INCDIR)/FreeRTOS
	cp $(LIB)/rtos/FreeRTOS/Source/include/FreeRTOS.h			$(INCDIR)/FreeRTOS
	touch	$@

$(INCDIR)/.CoOS:
	mkdir	-p	$(INCDIR)/CoOS
	cp $(LIB)/rtos/CoOS/portable/OsArch.h					$(INCDIR)/CoOS
	cp $(LIB)/rtos/CoOS/libcoos.h						$(INCDIR)/CoOS
	cp $(LIB)/rtos/CoOS/kernel/OsQueue.h					$(INCDIR)/CoOS
	cp $(LIB)/rtos/CoOS/kernel/OsEvent.h					$(INCDIR)/CoOS
	cp $(LIB)/rtos/CoOS/kernel/OsServiceReq.h				$(INCDIR)/CoOS
	cp $(LIB)/rtos/CoOS/kernel/OsFlag.h					$(INCDIR)/CoOS
	cp $(LIB)/rtos/CoOS/kernel/OsTask.h					$(INCDIR)/CoOS
	cp $(LIB)/rtos/CoOS/kernel/OsMM.h					$(INCDIR)/CoOS
	cp $(LIB)/rtos/CoOS/kernel/OsError.h					$(INCDIR)/CoOS
	cp $(LIB)/rtos/CoOS/kernel/OsTime.h					$(INCDIR)/CoOS
	cp $(LIB)/rtos/CoOS/kernel/OsConfig.h					$(INCDIR)/CoOS
	cp $(LIB)/rtos/CoOS/kernel/OsCore.h					$(INCDIR)/CoOS
	cp $(LIB)/rtos/CoOS/kernel/OsMutex.h					$(INCDIR)/CoOS
	cp $(LIB)/rtos/CoOS/kernel/OsKernelHeap.h				$(INCDIR)/CoOS
	cp $(LIB)/rtos/CoOS/kernel/CoOS.h					$(INCDIR)/CoOS
	cp $(LIB)/rtos/CoOS/kernel/OsTimer.h					$(INCDIR)/CoOS
	cp $(LIB)/rtos/CoOS/kernel/utility.h					$(INCDIR)/CoOS
	cp $(LIB)/rtos/CoOS/kernel/coocox.h					$(INCDIR)/CoOS
	touch	$@

$(INCDIR)/.board:
	mkdir	-p	$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/timetick.h				$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/syscalls.h				$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/lcd_gimp_image.h			$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/hamming.h				$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/bmp.h				$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/lcd_font10x14.h			$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/lcd_draw.h				$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/tsd.h				$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/led.h				$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/rand.h				$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/tsd_ads7843.h			$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/iso7816_4.h				$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/ili9325.h				$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/lcd_color.h				$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/board_lowlevel.h			$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/board_memories.h			$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/bitbanding.h			$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/uart_console.h			$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/at45_spi.h				$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/clock.h				$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/wav.h				$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/lcd_font.h				$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/math.h				$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/ads7843.h				$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/wm8731.h				$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/tsd_com.h				$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/at45d.h				$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/frame_buffer.h			$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/include/lcdd.h				$(INCDIR)/board/include
	cp $(LIB)/libboard_sam3s-ek/board.h					$(INCDIR)/board/
	touch	$@

clean:
	rm	-fr	$(INCDIR)
	rm	-fr	$(LIBDIR)



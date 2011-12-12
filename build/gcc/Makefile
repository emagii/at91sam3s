# ----------------------------------------------------------------------------
#         ATMEL Microcontroller Software Support 
# ----------------------------------------------------------------------------
# Copyright (c) 2010, Atmel Corporation
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice,
# this list of conditions and the disclaimer below.
#
# Atmel's name may not be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
# DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ----------------------------------------------------------------------------

EXAMPLES = acc_irq               \
  adc12                          \
  adc12_temp_sensor              \
  adc12_threshold_wakeup         \
  cm3_bit_banding                \
  crccu                          \
  dac12_sinewave                 \
  eefc_pgm                       \
  eefc_uniqueid                  \
  getting-started                \
  lowpower                       \
  pio_alternate_function         \
  pio_capture                    \
  pmc_clock_failure_detect       \
  pmc_clock_switching            \
  pwm_pdc                        \
  rtc                            \
  rtt                            \
  smc_psram                      \
  spi_dataflash                  \
  spi_slave                      \
  ssc_pdc_audio                  \
  tc_capture_waveform            \
  twi_eeprom                     \
  twi_slave                      \
  twi_temp_sensor                \
  usart_hard_handshaking         \
  usart_irda                     \
  usart_iso7816                  \
  usart_rs485                    \
  usart_spi                      \
  usart_synchronous              \
  wdg_irq

EXAMPLES_FILESYSTEM = fatfs_nandflash

EXAMPLES_GRAPHICS = smc_lcd      \
  spi_touchscreen

EXAMPLES_QTOUCH  = qtouch

EXAMPLES_STORAGE = hsmci_multimedia_card \
  hsmci_sdcard                   \
  hsmci_sdio                     \
  smc_nandflash                  \
  smc_norflash                   \

EXAMPLES_USB = usb_audio_headphone \
  usb_audio_looprec              \
  usb_audio_speaker              \
  usb_cdc_serial                 \
  usb_core                       \
  usb_hid_aud                    \
  usb_hid_keyboard               \
  usb_hid_mouse                  \
  usb_hid_msd                    \
  usb_hid_transfer               \
  usb_iad_cdc_aud                \
  usb_iad_cdc_cdc                \
  usb_iad_cdc_hid                \
  usb_iad_cdc_msd                \
  usb_massstorage

all: libraries $(EXAMPLES) $(EXAMPLES_FILESYSTEM) $(EXAMPLES_GRAPHICS) $(EXAMPLES_QTOUCH) $(EXAMPLES_STORAGE) $(EXAMPLES_USB)

libraries:
	(cd ../../libraries/build/gcc && "$(MAKE)" -f Makefile)

define COMPILS
$(1):
	(cd ../../$(2)/$(1)/build/gcc && "$(MAKE)" -f Makefile clean all)
endef


$(foreach EXAMPLE, $(EXAMPLES), $(eval $(call COMPILS,$(EXAMPLE),examples)))
$(foreach EXAMPLE, $(EXAMPLES_FILESYSTEM), $(eval $(call COMPILS,$(EXAMPLE),examples_filesystem)))
$(foreach EXAMPLE, $(EXAMPLES_GRAPHICS), $(eval $(call COMPILS,$(EXAMPLE),examples_graphics)))
$(foreach EXAMPLE, $(EXAMPLES_QTOUCH), $(eval $(call COMPILS,$(EXAMPLE),examples_qtouch)))
$(foreach EXAMPLE, $(EXAMPLES_STORAGE), $(eval $(call COMPILS,$(EXAMPLE),examples_storage)))
$(foreach EXAMPLE, $(EXAMPLES_USB), $(eval $(call COMPILS,$(EXAMPLE),examples_usb)))





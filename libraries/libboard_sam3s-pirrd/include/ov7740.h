#ifndef _OV7740_
#define _OV7740_

/* Registers */

/* Register definitions */
#define	OV7740_GAIN               0x00
#define	OV7740_BLUE_GAIN          0x01
#define	OV7740_RED_GAIN           0x02
#define	OV7740_GREEN_GAIN         0x03
#define	OV7740_REG04              0x04
#define	OV7740_BLUE_AVG           0x05
#define	OV7740_GREEN_AVG          0x06
#define	OV7740_RED_AVG            0x07
#define	OV7740_PIDH               0x0a
#define	OV7740_PIDL               0x0b
#define	OV7740_REG0C              0x0c
#define	OV7740_REG0C_FLIP              0x80
#define	OV7740_REG0C_MIRROR            0x40
#define	OV7740_REG0C_YUV_SWAP          0x10
#define	OV7740_REG0C_ML_SWAP           0x08

#define	OV7740_REG0D              0x0d
#define	OV7740_REG0E              0x0e // default: OV7740_REG0E_BLC_BOTH|OV7740_REG0E_BLC_OPTICAL
#define	OV7740_REG0E_OUTPUT_1x         0x00
#define	OV7740_REG0E_OUTPUT_2x         0x01
#define	OV7740_REG0E_OUTPUT_3x         0x02
#define	OV7740_REG0E_OUTPUT_4x         0x03
#define	OV7740_REG0E_SLEEP             0x08
#define	OV7740_REG0E_BLC_RED           0x20
#define	OV7740_REG0E_BLC_BLUE          0x40
#define	OV7740_REG0E_BLC_BOTH          0x60
#define	OV7740_REG0E_BLC_ELECTRICAL    0x00
#define	OV7740_REG0E_BLC_OPTICAL       0x80

#define	OV7740_HAEC               0x0f
#define	OV7740_AEC                0x10
#define	OV7740_CLK                0x11
#define	OV7740_CLK_DIV_MASK            0x3f
#define	OV7740_CLK_PLL_MASK            0xc0

#define	OV7740_REG12              0x12
#define	OV7740_REG12_RAW_RGB           0x01
#define	OV7740_REG12_RAW               0x10
#define	OV7740_REG12_CC656             0x20
#define	OV7740_REG12_VSKIP             0x40
#define	OV7740_REG12_RESET             0x80

#define	OV7740_REG13              0x13
#define	OV7740_REG13_EXP_AUTO          0x01
#define	OV7740_REG13_WBAL_AUTO         0x02
#define	OV7740_REG13_AGC_AUTO          0x04
#define	OV7740_REG13_LAEC_ENABLE       0x08
#define	OV7740_REG13_BANDING_OPT       0x10
#define	OV7740_REG13_BANDING_ENABLE    0x20
#define	OV7740_REG13_FRAME_DROP        0x40
#define	OV7740_REG13_AEC_FASTER        0x80

#define	OV7740_REG14              0x14
#define	OV7740_REG15              0x15
#define	OV7740_REG15_GAIN_LSB_MASK     0x03
#define	OV7740_REG15_NIGHT_2X_GAIN     0x00
#define	OV7740_REG15_NIGHT_4X_GAIN     0x04
#define	OV7740_REG15_NIGHT_8X_GAIN     0x08
#define	OV7740_REG15_NIGHT_16X_GAIN    0x0c
#define	OV7740_REG15_CEIL_0            0x00
#define	OV7740_REG15_CEIL_1            0x10
#define	OV7740_REG15_CEIL_2            0x20
#define	OV7740_REG15_CEIL_3            0x30
#define	OV7740_REG15_CEIL_7            0x70
#define	OV7740_REG15_ENABLE_NIGHT      0x80

#define	OV7740_REG16              0x16
#define	OV7740_AHSTART            0x17
#define	OV7740_AHSIZE             0x18
#define	OV7740_AVSTART            0x19
#define	OV7740_AVSIZE             0x1a
#define	OV7740_PSHFT              0x1b
#define	OV7740_MIDH               0x1c
#define	OV7740_MIDL               0x1d
#define	OV7740_REG1E              0x1e
#define	OV7740_REG1F              0x1f
#define	OV7740_REG1E              0x1e
#define	OV7740_REG20              0x20
#define	OV7740_REG21              0x21
#define	OV7740_WPT                0x24
#define	OV7740_BPT                0x25
#define	OV7740_VPT                0x26
#define	OV7740_REG27              0x27
#define	OV7740_REG27_BLACKSUN          0x80

#define	OV7740_REG28              0x28
#define	OV7740_REG28_VSYNC_NEG         0x02
#define	OV7740_REG28_NO_VSYNC_DROP     0x08
#define	OV7740_REG28_HREF_NEG          0x10
#define	OV7740_REG28_HSYNC_NEG         0x20
#define	OV7740_REG28_HSYNC             0x40
#define	OV7740_REG28_OUTPUT_REVERSE    0x80

#define	OV7740_YUV422CTRL         0xd9
#define	OV7740_YUV422CTRL_AVERAGE      0x00
#define	OV7740_YUV422CTRL_DROP         0x01
#define	OV7740_YUV422CTRL_YUYV         0x00
#define	OV7740_YUV422CTRL_YVYU         0x02

#define OV7740_REG065             0x65
#define OV7740_REG065_BIT_SWAP         0x08


#endif // _OV7740_

/*
 * NAU7802.h
 *
 * Created: 2023/4/20 11:05:22
 *  Author: hans
 */ 
/**************************************************************************//**
* @file      Seesaw.h
* @brief     Driver for the seesaw 4x4 LED button from Adafruit. See https://cdn-learn.adafruit.com/downloads/pdf/adafruit-seesaw-atsamd09-breakout.pdf
 and https://github.com/adafruit/Adafruit_Seesaw

* @author    Eduardo Garcia
* @date      2020-01-01

******************************************************************************/


#define _NEO_TRELLIS_H
#include "I2cDriver/I2cDriver.h"




// The order of primary colors in the NeoPixel data stream can vary
// among device types, manufacturers and even different revisions of
// the same item.  The third parameter to the seesaw_NeoPixel
// constructor encodes the per-pixel byte offsets of the red, green
// and blue primaries (plus white, if present) in the data stream --
// the following #defines provide an easier-to-use named version for
// each permutation.  e.g. NEO_GRB indicates a NeoPixel-compatible
// device expecting three bytes per pixel, with the first byte
// containing the green value, second containing red and third
// containing blue.  The in-memory representation of a chain of
// NeoPixels is the same as the data-stream order; no re-ordering of
// bytes is required when issuing data to the chain.

// Bits 5,4 of this value are the offset (0-3) from the first byte of
// a pixel to the location of the red color byte.  Bits 3,2 are the
// green offset and 1,0 are the blue offset.  If it is an RGBW-type
// device (supporting a white primary in addition to R,G,B), bits 7,6
// are the offset to the white byte...otherwise, bits 7,6 are set to
// the same value as 5,4 (red) to indicate an RGB (not RGBW) device.
// i.e. binary representation:
// 0bWWRRGGBB for RGBW devices
// 0bRRRRGGBB for RGB

// RGB NeoPixel permutations; white and red offsets are always same
// Offset:         W          R          G          B




//
/*************************************************************NAU7802************************************************/
//


#define ADC_SLAVE_ADDR        0x2A
/* Register address */

#define PU_CTRL_ADDR          0x00
#define CTRL1_ADDR            0x01
#define CTRL2_ADDR            0x02
#define OCAL1_B2_ADDR         0x03
#define OCAL1_B1_ADDR         0x04
#define OCAL1_B0_ADDR         0x05
#define GCAL1_B3_ADDR         0x06
#define GCAL1_B2_ADDR         0x07
#define GCAL1_B1_ADDR         0x08
#define GCAL1_B0_ADDR         0x09
#define OCAL2_B2_ADDR         0x0A
#define OCAL2_B1_ADDR         0x0B
#define OCAL2_B0_ADDR         0x0C
#define GCAL2_B3_ADDR         0x0D
#define GCAL2_B2_ADDR         0x0E
#define GCAL2_B1_ADDR         0x0F
#define GCAL2_B0_ADDR         0x10
#define I2C_CONTROL_ADDR      0x11
#define ADCO_B2_ADDR          0x12
#define ADCO_B1_ADDR          0x13
#define ADCO_B0_ADDR          0x14
#define OTP_B1_ADDR           0x15
#define OTP_B0_ADDR           0x16
#define PGA_PWR_ADDR          0x1B
#define DEVICE_REVISION_ADDR  0x1F



/* AVDD source select */

#define AVDDS_Pos  (7)
#define AVDDS_Msk  (1<<AVDDS_Pos)
#define AVDDS_LDO  (1<<AVDDS_Pos)  /* Internal LDO */
#define AVDDS_PIN  (0<<AVDDS_Pos)  /* AVDD pin input (default) */





/* System clock source select */

#define OSCS_Pos  (6)
#define OSCS_Msk  (1<<OSCS_Pos)
#define OSCS_EXT  (1<<OSCS_Pos)  /* External Crystal */
#define OSCS_IRC  (0<<OSCS_Pos)  /* Internal RC oscillator (default) */

/* Cycle ready (Read only Status) */

#define CR_Pos       (5)
#define CR_Msk       (1<<CR_Pos)
#define CR_DATA_RDY  (1<<CR_Pos)  /* ADC DATA is ready */

/* Cycle start */

#define CS_Pos               (4)
#define CS_Msk               (1<<CS_Pos)
#define CS_START_CONVERSION  (1<<CS_Pos)  /* Synchronize conversion to the rising edge of this register */


/* Power up ready (Read Only Status) */

#define PUR_Pos         (3)
#define PUR_Msk         (1<<PUR_Pos)
#define PUR_POWER_UP    (1<<PUR_Pos)    /* Power Up ready */
#define PUR_POWER_DOWN  (0<<PUR_Pos)    /* Power down, not ready */


/* Power up analog circuit */

#define PUA_Pos               (2)
#define PUA_Msk               (1<<PUA_Pos)
#define PUA_POWER_UP   (1<<PUA_Pos)    /* Power up the chip analog circuits (PUD must be 1) */
#define PUA_POWER_DOWN (0<<PUA_Pos)    /* Power down (default) */


/* Power up digital circuit */

#define PUD_Pos                 (1)
#define PUD_Msk                 (1<<PUD_Pos)
#define PUD_POWER_UP     (1<<PUD_Pos)    /* Power up the chip digital logic */
#define PUD_POWER_DOWN   (0<<PUD_Pos)    /* power down (default) */


/* Register reset */

#define RR_Pos    (0)
#define RR_Msk    (1<<RR_Pos)
#define RR_RESET  (1<<RR_Pos)  /* Register Reset, reset all register except RR */
#define RR_NORMAL (0<<RR_Pos)  /* Normal Operation (default) */

/* Conversion Ready Pin Polarity (16 Pin Package Only) */

#define CRP_Pos         (7)
#define CRP_Msk         (1<<CRP_Pos)
#define CRP_ACTIVE_LOW  (1<<CRP_Pos)   /* CRDY pin is LOW Active (Ready when 0) */
#define CRP_ACTIVE_HIGH (0<<CRP_Pos)   /* CRDY pin is High Active(Ready when 1) (default) */



/* Select the function of DRDY pin */

#define DRDY_SEL_Pos           (6)
#define DRDY_SEL_Msk           (1<<DRDY_SEL_Pos)
#define DRDY_SEL_OUTPUT_CLOCK  (1<<DRDY_SEL_Pos)       /* DRDY output the Buffered Crystal Clock if OSCS=1 */
#define DRDY_SEL_OUTPUT_CONVERSION  (0<<DRDY_SEL_Pos)  /* DRDY output the conversion ready (default) */


/* LDO Voltage */

#define VLDO_Pos  (3)
#define VLDO_Msk  (7<<VLDO_Pos)
#define VLDO_2V4  (7<<VLDO_Pos)  /* 2.4v */
#define VLDO_2V7  (6<<VLDO_Pos)  /* 2.7v */
#define VLDO_3V0  (5<<VLDO_Pos)  /* 3.0v */
#define VLDO_3V3  (4<<VLDO_Pos)  /* 3.3v */
#define VLDO_3V6  (3<<VLDO_Pos)  /* 3.6v */
#define VLDO_3V9  (2<<VLDO_Pos)  /* 3.9v */
#define VLDO_4V2  (1<<VLDO_Pos)  /* 4.2v */
#define VLDO_4V5  (0<<VLDO_Pos)  /* 4.5v (default) */


/* Gain select */

#define GAINS_Pos  (0)
#define GAINS_Msk  (7<<GAINS_Pos)
#define GAINS_128  (7<<GAINS_Pos)  /* x128 */
#define GAINS_64   (6<<GAINS_Pos)  /* x64 */
#define GAINS_32   (5<<GAINS_Pos)  /* x32 */
#define GAINS_16   (4<<GAINS_Pos)  /* x16 */
#define GAINS_8    (3<<GAINS_Pos)  /* x8 */
#define GAINS_4    (2<<GAINS_Pos)  /* x4 */
#define GAINS_2    (1<<GAINS_Pos)  /* x2 */
#define GAINS_1    (0<<GAINS_Pos)  /* x1 (default) */


/* Analog input channel select */

#define CHS_Pos  (7)
#define CHS_Msk  (1<<CHS_Pos)
#define CHS_CH2  (1<<CHS_Pos)  /* 1 = Ch2 */
#define CHS_CH1  (0<<CHS_Pos)  /* 0 = Ch1 (default) */


/* Conversion rate select */

#define CRS_Pos  (4)
#define CRS_Msk  (7<<CRS_Pos)
#define CRS_320  (7<<CRS_Pos)  /* 111 = 320SPS */
#define CRS_80   (3<<CRS_Pos)  /* 011 = 80SPS */
#define CRS_40   (2<<CRS_Pos)  /* 010 = 40SPS */
#define CRS_20   (1<<CRS_Pos)  /* 001 = 20SPS */
#define CRS_10   (0<<CRS_Pos)  /* 000 = 10SPS (default) */


/* Read Only calibration result */
#define CAL_ERR_Pos      (3)
#define CAL_ERR_Msk      (1<<CAL_ERR_Pos)
#define CAL_ERR_ERROR    (1<<CAL_ERR_Pos)  /* 1: there is error in this calibration */
#define CAL_ERR_NO_ERROR (0<<CAL_ERR_Pos)  /* 0: there is no error */


/* Write 1 to this bit will trigger calibration based on the selection in CALMOD[1:0] */

/* This is an "Action" register bit. When calibration is finished, it will reset to 0 */

#define CALS_Pos      (2)
#define CALS_Msk      (1<<CALS_Pos)
#define CALS_ACTION   (1<<CALS_Pos)
#define CALS_FINISHED (0<<CALS_Pos)


/* Calibration mode */

#define CALMOD_Pos              (0)
#define CALMOD_Msk              (3<<CALMOD_Pos)
#define CALMOD_GAIN             (3<<CALMOD_Pos)  /* 11 = Gain Calibration System */
#define CALMOD_OFFSET           (2<<CALMOD_Pos)  /* 10 = Offset Calibration System */
#define CALMOD_ RESERVED        (1<<CALMOD_Pos)  /* 01 = Reserved */
#define CALMOD_OFFSET_INTERNAL  (0<<CALMOD_Pos)  /* 00 = Offset Calibration Internal (default) */


/* Enable bit for Pull SDA low when conversion complete and I2C IDLE(special non-standard I2C) 1 = enable 0 = disable (default) */

#define CRSD_Pos            (7)
#define CRSD_Msk            (1<<CRSD_Pos)
#define CRSD_PULL_SDA_LOW   (1<<CRSD_Pos)
#define CRSD_PULL_SDA_HIGH  (0<<CRSD_Pos)


/* Enable bit for Fast Read ADC DATA (special non-standard I2C) 1 = enable fast read ADC Data special non-standard I2C */

#define FRD_Pos                  (6)
#define FRD_Msk                  (1<<FRD_Pos)
#define FRD_FAST_READ_ENABLE     (1<<FRD_Pos)
#define FRD_FAST_READ_DISENABLE  (0<<FRD_Pos) /* Disable fast read ADC Data feature(default) */



/* Enable bit for Strong Pull Up for I2C SCLK and SDA */

#define SPE_Pos                  (5)
#define SPE_Msk                  (1<<SPE_Pos)
#define SPE_STRONG_PULL_ENABLE   (1<<SPE_Pos)  /* enable strong pull up (nominal 1.6 k ohm) */
#define SPE_STRONG_PULL_DISABLE  (0<<SPE_Pos)  /* disable strong pull up (default) */



/* Disable bit for Weak Pull Up for I2C SCLK and SDA */

#define WPD_Pos                (4)
#define WPD_Msk                (1<<WPD_Pos)
#define WPD_WEAK_PULL_DISABLE  (1<<WPD_Pos)  /* disable weak pull up */
#define WPD_WEAK_PULL_ENABLE  (0<<WPD_Pos)   /* enable weak pull up (default nominal 50 k ohm) */



/* Short the input together, measure offset */

#define SI_Pos          (3)
#define SI_Msk          (1<<SI_Pos)
#define SI_SHORT_INPUT  (1<<SI_Pos)
#define SI_OPEN_INPUT   (0<<SI_Pos)

/* Enables the 2.5uA burnout current source to the PGA positive input when set to 1. */

#define BOPGA_Pos          (2)
#define BOPGA_Msk          (1<<BOPGA_Pos)
#define BOPGA_CURRENT_EN   (1<<BOPGA_Pos)
#define BOPGA_CURRENT_DIS  (0<<BOPGA_Pos)   /* 0: Disables the current source.(default) */

/* Switches PGA input to temperature sensor when set to 1. */

#define TS_Pos             (1)
#define TS_Msk             (1<<TS_Pos)
#define TS_TEMP_TO_PGA     (1<<TS_Pos)
#define TS_TEMP_NO_TO_PGA  (0<<TS_Pos)   /* 0: Uses VINx as PGA input (default)*/


/* Disables bandgap chopper when set to 1. */

#define BGPCP_Pos                      (0)
#define BGPCP_Msk                      (1<<BGPCP_Pos)
#define BGPCP_BANDGAP_CHOPPER_DISABLE  (1<<BGPCP_Pos)  /* Disables bandgap chopper */
#define BGPCP_BANDGAP_CHOPPER_ENABLE   (0<<BGPCP_Pos)  /* Enables the bandgap chopper.(default) */


/* Read REG0x15 output select */

#define RD_OTP_SEL_Pos  (7)
#define RD_OTP_SEL_Msk  (1<<RD_OTP_SEL_Pos)
#define RD_OTP_SEL_OTP  (1<<RD_OTP_SEL_Pos)  /* Read REG0x15 will read OTP[31:24] */
#define RD_OTP_SEL_ADC  (0<<RD_OTP_SEL_Pos)  /* Read REG0x15 will read ADC Registers */


/* LDOMODE */

#define LDOMODE_Pos  (6)
#define LDOMODE_Msk  (1<<LDOMODE_Pos)
#define LDOMODE_LOW_DC  (1<<LDOMODE_Pos)   /* Improved stability and lower DC gain */
#define LDOMODE_HIGH_DC  (0<<LDOMODE_Pos)  /* Improved accuracy and higher DC gain */


/* PGA output buffer enable */

#define PGA_OUTPUT_BUFFER_Pos  (5)
#define PGA_OUTPUT_BUFFER_Msk  (1<<PGA_OUTPUT_BUFFER_Pos)
#define PGA_OUTPUT_BUFFER_ENABLE  (1<<PGA_OUTPUT_BUFFER_Pos)   /* 1: PGA output buffer enable */
#define PGA_OUTPUT_BUFFER_DISABLE  (0<<PGA_OUTPUT_BUFFER_Pos)  /* 0: PGA output buffer disable */



/* PGA bypass enable */

#define PGA_BYPASS_Pos  (4)
#define PGA_BYPASS_Msk  (1<<PGA_BYPASS_Pos)
#define PGA_BYPASS_ENABLE  (1<<PGA_BYPASS_Pos)   /* 1: PGA bypass enable */
#define PGA_BYPASS_DISABLE  (0<<PGA_BYPASS_Pos)  /* 0: PGA bypass disable */

/* 3 */

/* PGAINV */

#define PGAINV_Pos  (3)
#define PGAINV_Msk  (1<<PGAINV_Pos)
#define PGAINV_INVERT  (1<<PGAINV_Pos)  /* 1: invert PGA input phase */
#define PGAINV_NORMAL  (0<<PGAINV_Pos)  /* 0: default */



/* PGACHPDIS */

#define PGACHPDIS_Pos  (0)
#define PGACHPDIS_Msk  (1<<PGACHPDIS_Pos)
#define PGACHPDIS_DISABLE  (1<<PGACHPDIS_Pos)  /* 1: Chopper disabled */
#define PGACHPDIS_ENABLE  (0<<PGACHPDIS_Pos)   /* 0: default */


/* Enables PGA output bypass capacitor connected across pins Vin2P Vin2N */

#define PGA_CAP_EN_Pos (7)

#define PGA_CAP_EN_Msk (1<<PGA_CAP_EN_Pos)

#define PGA_CAP_EN_SET (1<<PGA_CAP_EN_Pos)
#define PGA_CAP_EN_CLR  (0<<PGA_CAP_EN_Pos)


/* Master bias Current */

#define MASTER_BIAS_CURR_Pos  (4)
#define MASTER_BIAS_CURR_Msk  (7<<MASTER_BIAS_CURR_Pos)
#define MASTER_BIAS_CURR_100 (0<<MASTER_BIAS_CURR_Pos)  /* 100% (default) */
#define MASTER_BIAS_CURR_90  (1<<MASTER_BIAS_CURR_Pos)  /* 90% (lower power & accuracy) */
#define MASTER_BIAS_CURR_80  (2<<MASTER_BIAS_CURR_Pos)  /* 0 1 0 80% */
#define MASTER_BIAS_CURR_73  (3<<MASTER_BIAS_CURR_Pos)  /* 0 1 1 73% */
#define MASTER_BIAS_CURR_67  (4<<MASTER_BIAS_CURR_Pos)  /* 1 0 0 67% */
#define MASTER_BIAS_CURR_62  (5<<MASTER_BIAS_CURR_Pos)  /* 1 0 1 62% */
#define MASTER_BIAS_CURR_58  (6<<MASTER_BIAS_CURR_Pos)  /* 1 1 0 58% */
#define MASTER_BIAS_CURR_54  (7<<MASTER_BIAS_CURR_Pos)  /* 1 1 1 54% */


/* ADC Current */

#define ADC_CURR_Pos  (2)
#define ADC_CURR_Msk  (3<<ADC_CURR_Pos)
#define ADC_CURR_100  (0<<ADC_CURR_Pos)     /* 0 0 100% of master bias */
#define ADC_CURR_75  (1<<ADC_CURR_Pos)      /* 0 1 75%  of master bias */
#define ADC_CURR_50  (2<<ADC_CURR_Pos)      /* 1 0 50%  of master bias */
#define ADC_CURR_25  (3<<ADC_CURR_Pos)      /* 1 1 25%  of master bias */

/* PGA Current */

#define PGA_CURR_Pos  (0)
#define PGA_CURR_Msk  (3<<PGA_CURR_Pos)
#define PGA_CURR_100  (0<<PGA_CURR_Pos)  /* 0 0 100% of master bias (default) */
#define PGA_CURR_95  (1<<PGA_CURR_Pos)   /* 0 1 95% of master bias (lower power & accuracy) */
#define PGA_CURR_86  (2<<PGA_CURR_Pos)   /* 1 0 86% of master bias */
#define PGA_CURR_70  (3<<PGA_CURR_Pos)   /* 1 1 70% of master bias */
uint32_t ADC_Read_Conversion_Data(void);
void ADC_StartConversion(void);
void  ADCchip_Init(void);
uint8_t ADC_ReadReg(uint8_t u8RegAddr);
uint8_t ADC_WriteReg(uint8_t u8RegAddr, uint8_t data);
void ADC_Config(uint8_t ch, uint8_t rate, uint8_t gain);
void ADC_Calibration(void);
int32_t I2C_ReadMultiBytesOneReg(uint8_t u8RegAddr1, uint8_t u8RegAddr2,uint8_t *data,uint32_t len);
static  int32_t reg_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t reg_write(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len);
void Value_conversion(int value,int final[2]);
void Get_Weight(int final_adc[2]);
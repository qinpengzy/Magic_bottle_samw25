/*
 * NAU7802.c
 *
 * Created: 2023/4/20 11:04:50
 *  Author: hans
 */
#include "Loadcell/Nau7802.h"
#include "I2cDriver/I2cDriver.h"
#include "SerialConsole.h"
uint8_t msgOut[64];
I2C_Data adcData; 
static int32_t reg_write(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len)
{
	int32_t error = ERROR_NONE;
	msgOut[0]=reg;
	for(int i=0;i<len;i++)
	{
		msgOut[1+i]=bufp[i];
	}
	
	adcData.address=ADC_SLAVE_ADDR;	///<Address of the I2C device
	adcData.msgOut=&msgOut;		///<Pointer to array buffer that we will write from
	adcData.lenOut=1+len;
	error= I2cWriteDataWait(&adcData, 100);
		//char help[64];
		//snprintf(help, 64, "error_write = %d\r\n",error);
		//SerialConsoleWriteString(help);
	return error;

}

static  int32_t reg_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	int32_t error = ERROR_NONE;
	
	adcData.address=ADC_SLAVE_ADDR;	///<Address of the I2C device

	adcData.lenIn=len;			///<Length of message to read/write;	
	adcData.msgIn=bufp;
	msgOut[0]=reg;
	msgOut[1]=0;
	adcData.msgOut=&msgOut;	
	adcData.lenOut=1;	///<Pointer to array buffer that we will write from
	error= I2cReadDataWait(&adcData, 5, 100);
	
	//char help[64];
	//snprintf(help, 64, "error_read = %d\r\n",error);
	//SerialConsoleWriteString(help);
	
	return error;
}
void *handle;
uint8_t ADC_ReadReg(uint8_t u8RegAddr)
{
	/*//uint8_t rdata = 0U;
	uint8_t ADC_IN[1]={u8RegAddr};
	uint8_t ADC_bytes[1];
	
	adcData.address = ADC_SLAVE_ADDR;
	adcData.msgIn=&ADC_bytes;
	adcData.lenIn=sizeof(ADC_bytes);
	adcData.msgOut = &ADC_IN;
	adcData.lenOut = sizeof(ADC_IN);

	//rdata = I2C_ReadByteOneReg(I2C1, ADC_SLAVE_ADDR, u8RegAddr);
	int32_t err= I2cReadData(&adcData);
	return ADC_bytes[0];*/
	static uint8_t read_bytes;
	
	int32_t err= reg_read(handle, u8RegAddr, &read_bytes,1);
	return read_bytes;
	
}


uint8_t ADC_WriteReg(uint8_t u8RegAddr, uint8_t data)
{

	/*adcData.address = ADC_SLAVE_ADDR;
	adcData.msgOut=&data;
	adcData.lenOut=sizeof(data);
	int32_t err=I2cWriteData(&adcData);
	
	adcData.address = u8RegAddr;
	adcData.msgOut=&data;
	adcData.lenOut=sizeof(data);
	int32_t err=I2cWriteData(&adcData);
	return err;*/
	int32_t err=reg_write(handle, u8RegAddr,&data,1);
	return err;
}

// void ADC_Config(uint8_t ch, uint8_t rate, uint8_t gain)
// {
// 	uint8_t reg = 0;
// 	/* Set channel and Conversion rate */
// 
// 	reg = ADC_ReadReg(CTRL2_ADDR);
// 	reg &= ~(CHS_Msk | CRS_Msk);
// 	reg |= (ch |   /* CHS: 0 = ch1,1 = ch2 */
// 	rate); /* CRS: Sample rate can be set to 10, 20, 40, 80, or 320Hz */
// 	ADC_WriteReg(CTRL2_ADDR, reg);
// 
// 	/* Set gain */
// 
// 	reg = ADC_ReadReg(CTRL1_ADDR);
// 	reg &= ~(GAINS_Msk);
// 	reg |= gain;  /* Gain can be set to 1, 2, 4, 8, 16, 32, 64, or 128. */
// 
// 	ADC_WriteReg(CTRL1_ADDR, reg);
// }

void ADC_Calibration(void)
{
	uint8_t reg = 0;
	while (1)
	{
		reg = ADC_ReadReg(CTRL2_ADDR);
		reg &= ~(CALMOD_Msk | CALS_Msk);

		/* Set Calibration mode */
		reg |= CALMOD_OFFSET_INTERNAL;   /* Calibration mode = Internal Offset Calibration */
		ADC_WriteReg(CTRL2_ADDR, reg);
		/* Start calibration */
		reg |= CALS_ACTION;              /* Start calibration */
		ADC_WriteReg(CTRL2_ADDR, reg);

		while (1)
		{
			/* Wait for calibration finish */
			delay_ms(50); /* Wait 50ms */
			/* Read calibration result */
			reg = ADC_ReadReg(CTRL2_ADDR);

			if ((reg & CALS_Msk) == CALS_FINISHED)
			break;
		}
		reg &= CAL_ERR_Msk;
		if ((reg & CAL_ERR_Msk) == 0) /* There is no error */
		break;
	}
	delay_ms(1);    /* Wait 1 ms */

}

void  ADCchip_Init(void)
{
	uint8_t reg = 0;

	/* Reset */
	reg =  0x01;                   /* Enter reset mode */
	ADC_WriteReg(PU_CTRL_ADDR, reg);
	delay_ms(1);         /* Wait 1 ms */

	reg =  0x02 ;                  /* Enter Noraml mode */
	ADC_WriteReg(PU_CTRL_ADDR, reg);
	delay_ms(50);         /* Wait 50 ms */

/*
	reg =  0x06 ;                  //Enter PUR 
	ADC_WriteReg(PU_CTRL_ADDR, reg);
	delay_ms(1);
	
	// Setting 
	reg = (AVDDS_PIN | // AVDD = external pin input 
	OSCS_IRC);  //Clock = Internal RC oscillator 
	ADC_WriteReg(PU_CTRL_ADDR, reg);

	reg = (CRP_ACTIVE_LOW |             // DRDY = LOW Active 
	DRDY_SEL_OUTPUT_CONVERSION | // DRDY output = conversion ready 
	VLDO_3V3 |                   // LDO = 3.3V  (no use) 
	GAINS_1);                    // PGA = x1 
*/	
	reg=0x27;
	ADC_WriteReg(CTRL1_ADDR, reg);
	delay_ms(1);
	
	reg=0x86;
	ADC_WriteReg(PU_CTRL_ADDR, reg);
	delay_ms(1);
	
	reg=0x30;
	ADC_WriteReg(OTP_B1_ADDR , reg);
	delay_ms(1);
   

	/* Calibration */
	ADC_Calibration();
}

void ADC_StartConversion(void)
{
	uint8_t reg = 0;
	/* Start conversion */
	reg = ADC_ReadReg(PU_CTRL_ADDR);
	reg |= CS_START_CONVERSION; /* CS=1 */
	ADC_WriteReg(PU_CTRL_ADDR, reg);
}

int32_t I2C_ReadMultiBytesOneReg(uint8_t u8RegAddr1, uint8_t u8RegAddr2,uint8_t *data,uint32_t len)
{
	int32_t temp1, temp2, temp3, x;
	temp1 = ADC_ReadReg( 0x02);
	temp1 |= 0 << 7;
	ADC_WriteReg(0x02, temp1);
	vTaskDelay(10);

	temp1 = ADC_ReadReg(0x12);
	temp2 = ADC_ReadReg(0x13);
	temp3 = ADC_ReadReg( 0x14);

	x = temp1 << 16 | temp2 << 8 | temp3 << 0;
	return x;
	/*uint8_t ADC_IN[2]={u8RegAddr1,u8RegAddr2};
	uint8_t ADC_bytes[len];
	adcData.address = ADC_SLAVE_ADDR;
	adcData.msgIn=&ADC_bytes;
	adcData.lenIn=sizeof(ADC_bytes);
	adcData.msgOut = &data;
	adcData.lenOut = sizeof(data);

	int32_t err= I2cReadData(&adcData);*/
	
	//int32_t err= reg_read(handle, u8RegAddr2, data,len);
}

uint32_t ADC_Read_Conversion_Data(void)
{
	uint8_t rdata[3];
	uint32_t result;
	result = I2C_ReadMultiBytesOneReg( ADC_SLAVE_ADDR, ADCO_B2_ADDR, &rdata[0], 3);
	//((rdata[0] << 16) | (rdata[1] << 8) | rdata[2]);
	return result;
}

void Value_conversion(int value,int final[2]){
	float calibrate_adc;
	float gain;
	float offset;
	uint8_t gain_reg[4];
	uint8_t offset_reg[3];

	gain_reg[0]=ADC_ReadReg(GCAL1_B3_ADDR);
	gain_reg[1]=ADC_ReadReg(GCAL1_B2_ADDR);
	gain_reg[2]=ADC_ReadReg(GCAL1_B1_ADDR);
	gain_reg[3]=ADC_ReadReg(GCAL1_B0_ADDR);
	offset_reg[0]=ADC_ReadReg(OCAL1_B2_ADDR);
	offset_reg[1]=ADC_ReadReg(OCAL1_B1_ADDR);
	offset_reg[2]=ADC_ReadReg(OCAL1_B0_ADDR);
	
 	char help[32];
// 	for(int j=0;j<4;j++)
// 	{
// 			snprintf(help, 32, "gain_reg[%d] = %d\r\n",j,gain_reg[j]);
// 			SerialConsoleWriteString(help);
// 			snprintf(help, 32, "offset_reg [%d]= %d\r\n",j%4,offset_reg[j%4]);
// 		    SerialConsoleWriteString(help);
// 	}

	for(int i=31;i>=0;i--){
		gain+=(float)(((gain_reg[3-i/8]>>(i%8))&0x01)*(2<<(i-23)*10000));
	}
	for(int i=22;i>=0;i--){
		offset+=(float)(((offset_reg[2-i/8]>>(i%8))&0x01)*(2<<(i-23)*10000));
	}
	offset*=(float)(1-(offset_reg[0]>>7)&0x01);
	//snprintf(help, 32, " gain= %d\r\n",gain);
	//SerialConsoleWriteString(help);
	//snprintf(help, 32, "final offset= %d\r\n",offset);
	//SerialConsoleWriteString(help);
	
	calibrate_adc=(float)gain/10000*((float)value-(float)offset/10000);
	
	final[0]=(int)calibrate_adc; //CALI
	final[1]=10000*(calibrate_adc-final[0]);
//  	snprintf(help, 32, "final_int= %d\r\n",final[0]);
//  	SerialConsoleWriteString(help);
// 	snprintf(help, 32, "final_decimal= %d\r\n",final[1]);
// 	SerialConsoleWriteString(help);
}
void Get_Weight(int final_adc[2])
{
	    ADC_StartConversion();
	    while ((ADC_ReadReg(PU_CTRL_ADDR)&CR_Msk) != CR_DATA_RDY);
	    uint32_t ADC_value=ADC_Read_Conversion_Data();
	    Value_conversion(ADC_value,final_adc);
}
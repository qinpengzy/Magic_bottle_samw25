/**************************************************************************//**
* @file      BootMain.c
* @brief     Main file for the ESE516 bootloader. Handles updating the main application
* @details   Main file for the ESE516 bootloader. Handles updating the main application
* @author    Eduardo Garcia
* @date      2020-02-15
* @version   2.0
* @copyright Copyright University of Pennsylvania
******************************************************************************/


/******************************************************************************
* Includes
******************************************************************************/
#include <asf.h>
#include "conf_example.h"
#include <string.h>
#include "sd_mmc_spi.h"

#include "SD Card/SdCard.h"
#include "Systick/Systick.h"
#include "SerialConsole/SerialConsole.h"
#include "ASF/sam0/drivers/dsu/crc32/crc32.h"





/******************************************************************************
* Defines
******************************************************************************/
#define APP_START_ADDRESS  ((uint32_t)0x12000) ///<Start of main application. Must be address of start of main application
#define APP_START_RESET_VEC_ADDRESS (APP_START_ADDRESS+(uint32_t)0x04) ///< Main application reset vector address
//#define MEM_EXAMPLE 1 //COMMENT ME TO REMOVE THE MEMORY WRITE EXAMPLE BELOW

/******************************************************************************
* Structures and Enumerations
******************************************************************************/

struct usart_module cdc_uart_module; ///< Structure for UART module connected to EDBG (used for unit test output)

/******************************************************************************
* Local Function Declaration
******************************************************************************/
static void jumpToApplication(void);
static bool StartFilesystemAndTest(void);
static void configure_nvm(void);

static void free_fw_mem(void);
//static void check_erased(int num2erase);
static int choose_bin();
static void load_bin(char *helpStr);

/******************************************************************************
* Global Variables
******************************************************************************/
//INITIALIZE VARIABLES
char test_file_name[] = "0:sd_mmc_test.txt";	///<Test TEXT File name
char test_bin_file[] = "0:sd_binary.bin";	///<Test BINARY File name
Ctrl_status status; ///<Holds the status of a system initialization
FRESULT res; //Holds the result of the FATFS functions done on the SD CARD TEST
FATFS fs; //Holds the File System of the SD CARD
FIL file_object; //FILE OBJECT used on main for the SD Card Test



/******************************************************************************
* Global Functions
******************************************************************************/

/**************************************************************************//**
* @fn		int main(void)
* @brief	Main function for ESE516 Bootloader Application

* @return	Unused (ANSI-C compatibility).
* @note		Bootloader code initiates here.
*****************************************************************************/

int main(void)
{

	/*1.) INIT SYSTEM PERIPHERALS INITIALIZATION*/
	system_init();
	delay_init();
	InitializeSerialConsole();
	system_interrupt_enable_global();
	/* Initialize SD MMC stack */
	sd_mmc_init();

	//Initialize the NVM driver
	configure_nvm();

	irq_initialize_vectors();
	cpu_irq_enable();

	//Configure CRC32
	dsu_crc32_init();

	SerialConsoleWriteString("ESE516 - ENTER BOOTLOADER");	//Order to add string to TX Buffer

	/*END SYSTEM PERIPHERALS INITIALIZATION*/


	/*2.) STARTS SIMPLE SD CARD MOUNTING AND TEST!*/

	//EXAMPLE CODE ON MOUNTING THE SD CARD AND WRITING TO A FILE
	//See function inside to see how to open a file
	SerialConsoleWriteString("\x0C\n\r-- SD/MMC Card Example on FatFs --\n\r");

	if(StartFilesystemAndTest() == false)
	{
		SerialConsoleWriteString("SD CARD failed! Check your connections. System will restart in 5 seconds...");
		delay_cycles_ms(5000);
		system_reset();
	}
	else
	{
		SerialConsoleWriteString("SD CARD mount success! Filesystem also mounted. \r\n");
	}

	/*END SIMPLE SD CARD MOUNTING AND TEST!*/


	/*3.) STARTS BOOTLOADER HERE!*/
	/*********************************************************** BOOTLOADER START ******************************************************************/
	
	// Get NVM information
	struct nvm_parameters parameters;
	char helpStr[64]; //Used to help print values
	nvm_get_parameters (&parameters); //Get NVM parameters
	snprintf(helpStr, 63,"NVM Info: Number of Pages %d. Size of a page: %d bytes. \r\n", parameters.nvm_number_of_pages, parameters.page_size);
	SerialConsoleWriteString(helpStr);
	

	//SerialConsoleWriteString("no flag A \r\n");	//Order to add string to TX Buffer
	
	// Choose bin and load
	int flag = choose_bin();
	
	if (flag == 0)//file doesn't exist
	{
	}
	else if (flag == 1) // A flag exists
	{
		//res = f_open(&fno, (char const *)test_fw_A, FA_READ);
		// Free the FW memory before writing
		free_fw_mem(); // Calculate the number of rows to erase. Erase them and check each time erasing a row.
		strcpy(test_bin_file, "0:FlagA.bin");
		load_bin(helpStr); //update TestA.bin to NVM
	}
	
	
	// Read the binary file. Check CRC each time a chunk is read.
	
	/*********************************************************** BOOTLOADER END ********************************************************************/
	


	//4.) DEINITIALIZE HW AND JUMP TO MAIN APPLICATION!
	SerialConsoleWriteString("ESE516 - EXIT BOOTLOADER");	//Order to add string to TX Buffer
	delay_cycles_ms(100); //Delay to allow print
		
		//Deinitialize HW - deinitialize started HW here!
		DeinitializeSerialConsole(); //Deinitializes UART
		sd_mmc_deinit(); //Deinitialize SD CARD


		//Jump to application
		jumpToApplication();

		//Should not reach here! The device should have jumped to the main FW.
	
}







/******************************************************************************
* Static Functions
******************************************************************************/



/**************************************************************************//**
* function      static void StartFilesystemAndTest()
* @brief        Starts the filesystem and tests it. Sets the filesystem to the global variable fs
* @details      Jumps to the main application. Please turn off ALL PERIPHERALS that were turned on by the bootloader
*				before performing the jump!
* @return       Returns true is SD card and file system test passed. False otherwise.
******************************************************************************/
static bool StartFilesystemAndTest(void)
{
	bool sdCardPass = true;
	uint8_t binbuff[256];

	//Before we begin - fill buffer for binary write test
	//Fill binbuff with values 0x00 - 0xFF
	for(int i = 0; i < 256; i++)
	{
		binbuff[i] = i;
	}

	//MOUNT SD CARD
	Ctrl_status sdStatus= SdCard_Initiate();
	if(sdStatus == CTRL_GOOD) //If the SD card is good we continue mounting the system!
	{
		SerialConsoleWriteString("SD Card initiated correctly!\n\r");

		//Attempt to mount a FAT file system on the SD Card using FATFS
		SerialConsoleWriteString("Mount disk (f_mount)...\r\n");
		memset(&fs, 0, sizeof(FATFS));
		res = f_mount(LUN_ID_SD_MMC_0_MEM, &fs); //Order FATFS Mount
		if (FR_INVALID_DRIVE == res)
		{
			LogMessage(LOG_INFO_LVL ,"[FAIL] res %d\r\n", res);
			sdCardPass = false;
			goto main_end_of_test;
		}
		SerialConsoleWriteString("[OK]\r\n");

		//Create and open a file
		SerialConsoleWriteString("Create a file (f_open)...\r\n");

		test_file_name[0] = LUN_ID_SD_MMC_0_MEM + '0';
		res = f_open(&file_object,
		(char const *)test_file_name,
		FA_CREATE_ALWAYS | FA_WRITE);
		
		if (res != FR_OK)
		{
			LogMessage(LOG_INFO_LVL ,"[FAIL] res %d\r\n", res);
			sdCardPass = false;
			goto main_end_of_test;
		}

		SerialConsoleWriteString("[OK]\r\n");

		//Write to a file
		SerialConsoleWriteString("Write to test file (f_puts)...\r\n");

		if (0 == f_puts("Test SD/MMC stack\n", &file_object))
		{
			f_close(&file_object);
			LogMessage(LOG_INFO_LVL ,"[FAIL]\r\n");
			sdCardPass = false;
			goto main_end_of_test;
		}

		SerialConsoleWriteString("[OK]\r\n");
		f_close(&file_object); //Close file
		SerialConsoleWriteString("Test is successful.\n\r");


		//Write binary file
		//Read SD Card File
		test_bin_file[0] = LUN_ID_SD_MMC_0_MEM + '0';
		res = f_open(&file_object, (char const *)test_bin_file, FA_WRITE | FA_CREATE_ALWAYS);
		
		if (res != FR_OK)
		{
			SerialConsoleWriteString("Could not open binary file!\r\n");
			LogMessage(LOG_INFO_LVL ,"[FAIL] res %d\r\n", res);
			sdCardPass = false;
			goto main_end_of_test;
		}

		//Write to a binaryfile
		SerialConsoleWriteString("Write to test file (f_write)...\r\n");
		uint32_t varWrite = 0;
		if (0 != f_write(&file_object, binbuff,256, &varWrite))
		{
			f_close(&file_object);
			LogMessage(LOG_INFO_LVL ,"[FAIL]\r\n");
			sdCardPass = false;
			goto main_end_of_test;
		}

		SerialConsoleWriteString("[OK]\r\n");
		f_close(&file_object); //Close file
		SerialConsoleWriteString("Test is successful.\n\r");
		
		main_end_of_test:
		SerialConsoleWriteString("End of Test.\n\r");

	}
	else
	{
		SerialConsoleWriteString("SD Card failed initiation! Check connections!\n\r");
		sdCardPass = false;
	}

	return sdCardPass;
}



/**************************************************************************//**
* function      static void jumpToApplication(void)
* @brief        Jumps to main application
* @details      Jumps to the main application. Please turn off ALL PERIPHERALS that were turned on by the bootloader
*				before performing the jump!
* @return       
******************************************************************************/
static void jumpToApplication(void)
{
// Function pointer to application section
void (*applicationCodeEntry)(void);

// Rebase stack pointer
__set_MSP(*(uint32_t *) APP_START_ADDRESS);

// Rebase vector table
SCB->VTOR = ((uint32_t) APP_START_ADDRESS & SCB_VTOR_TBLOFF_Msk);

// Set pointer to application section
applicationCodeEntry =
(void (*)(void))(unsigned *)(*(unsigned *)(APP_START_RESET_VEC_ADDRESS));

// Jump to application. By calling applicationCodeEntry() as a function we move the PC to the point in memory pointed by applicationCodeEntry, 
//which should be the start of the main FW.
applicationCodeEntry();
}



/**************************************************************************//**
* function      static void configure_nvm(void)
* @brief        Configures the NVM driver
* @details      
* @return       
******************************************************************************/
static void configure_nvm(void)
{
    struct nvm_config config_nvm;
    nvm_get_config_defaults(&config_nvm);
    config_nvm.manual_page_write = false;
    nvm_set_config(&config_nvm);
}


/**************************************************************************//**
* function      static void free_fw_mem(void)
* @brief        Free the application memory
* @details		Calculate the number of rows which is available to erase and erase them. 
				Each a row is erased, check whether it is successful. If there is error, 
				return and tell the user.
* @return
******************************************************************************/
static void free_fw_mem(void){
	uint32_t current_address = APP_START_ADDRESS;
	enum status_code nvmError =  STATUS_OK;
	
	//Calculate the rows to erase
	int row = (0x40000 - 0x12000) / 256;
	
	//Erase the memory
	for (int i = 0; i < row; i++){
		enum status_code nvmError = nvm_erase_row(current_address); // Erase the row starts from current address
		if (nvmError != STATUS_OK){ SerialConsoleWriteString("Erase error!\r\n");}
		
		// Make sure it got erased
    	for(int iter = 0; iter < 256; iter++){
	    	char *a = (char *)(APP_START_ADDRESS + iter); //Pointer pointing to address APP_START_ADDRESS
	    	if(*a != 0xFF){	SerialConsoleWriteString("Error - test page is not erased!");	return;}
		}
		current_address += 256;
	}
	
	// Report the result
	SerialConsoleWriteString("Data erased successfully! \r\n");
}



/**************************************************************************//**
* function      static void choose_bin()
* @brief        Choose the binary file to load
* @details		If FlagA exists, we will load TestA this time, the we delete FlagA and create FlagB for the next time of loading.
				Creating FlagB is not in this function.
* @return
******************************************************************************/
static int choose_bin()
{
	int flag = 0;
	char test_file_name[] = "0:FlagA.bin";
	test_file_name[0] = LUN_ID_SD_MMC_0_MEM + '0';
	res = f_open(&file_object,  (char const *)test_file_name, FA_READ);
	if (res == FR_OK)
	{
		SerialConsoleWriteString("detect flag A!\r\n");
		flag = 1;
	}
	return flag;

}


/**************************************************************************//**
* function      static void load_bin(char *helpStr)
* @brief        Load the binary file.
* @details		Write data in chunks to load the opened file into MCU firmware region. Keep reading if there are bytes left.
				Check CRC for each chunk read. If CRC is different, break. Tell the user CRC status when finshed.
* @return
******************************************************************************/
static void load_bin(char *helpStr){
	test_bin_file[0] = LUN_ID_SD_MMC_0_MEM + '0';
	res = f_open(&file_object, (char const *)test_bin_file, FA_READ);
		
	if (res != FR_OK)
	{
		SerialConsoleWriteString("Could not open file!\r\n");
	}
	
	#define BUFFER_SIZE 64
	int fileSize  = f_size(&file_object);
	int numBytesLeft = fileSize;
	uint8_t readBuffer[BUFFER_SIZE];
	
	
	uint32_t numBytesRead = 0;
	int numberBytesTotal = 0;
	int pos = 0;
	while(numBytesLeft  > 0) 
	{	
		int chunkSize = (numBytesLeft > BUFFER_SIZE) ? BUFFER_SIZE : numBytesLeft;
		
		res = f_read(&file_object, &readBuffer, chunkSize, &numBytesRead); //Question to students: What is numBytesRead? What are we doing here?
		//numBytesRead stores the actual number of bytes that were read from the file. 
		//This value can be used to determine the next readBuffer's start address and determine whether all the data are read.
		
		res = nvm_write_buffer (APP_START_ADDRESS + pos, &readBuffer[0], chunkSize);
		pos+=chunkSize;
		if (res != FR_OK)
		{
			SerialConsoleWriteString("Test write to NVM failed!\r\n");
			break;
		}
		
		numBytesLeft -= numBytesRead;
		numberBytesTotal += numBytesRead;
		
		
		
		uint32_t resultCrcSd = 0;
		*((volatile unsigned int*) 0x41007058) &= ~0x30000UL;

		//CRC of SD Card
		enum status_code crcres = dsu_crc32_cal	(readBuffer,numBytesRead, &resultCrcSd); //Instructor note: Was it the third parameter used for? Please check how you can use the third parameter to do the CRC of a long data stream in chunks - you will need it!
		// It is a pointer to a uint32_t variable that will store the calculated CRC-32 value. The function will update the value of this variable.
		//In this function, we check every time the function reads data.
	
		//Errata Part 2 - To be done after RAM CRC
		*((volatile unsigned int*) 0x41007058) |= 0x20000UL;
	 
	 
		//CRC of memory (NVM)
		uint32_t resultCrcNvm = 0;
		crcres |= dsu_crc32_cal	(APP_START_ADDRESS +pos - chunkSize	,numBytesRead, &resultCrcNvm);
		if (resultCrcSd != resultCrcSd)
		{
			SerialConsoleWriteString("CRC different! CRC check fails.\r\n");
			res = FR_INVALID_OBJECT;
			break;
		}
	}
	

		if (res != FR_OK)
		{
			SerialConsoleWriteString("Test write to NVM failed!\r\n");
		}
		else
		{
			SerialConsoleWriteString("Test write to NVM succeeded!\r\n");
			SerialConsoleWriteString("CRC check succeeded!\r\n\n\n\n\n\n\n\n");
		}

}

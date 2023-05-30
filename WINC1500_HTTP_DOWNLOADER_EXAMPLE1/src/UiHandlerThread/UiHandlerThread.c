/**
* @file      UiHandlerThread.c
* @brief     File that contains the task code and supporting code for the UI
Thread for ESE516 Spring (Online) Edition
* @author    You! :)
* @date      2020-04-09

******************************************************************************/

/******************************************************************************
 * Includes
 ******************************************************************************/
#include "UiHandlerThread/UiHandlerThread.h"

#include <errno.h>

#include "DistanceDriver/DistanceSensor.h"
#include "IMU/lsm6dso_reg.h"
#include "SeesawDriver/Seesaw.h"
#include "SerialConsole.h"
#include "WifiHandlerThread/WifiHandler.h"
#include "asf.h"
#include "gfx_mono.h"
#include "main.h"

#include "Accelerate/LIS2DH12.h"
#include "Loadcell/Nau7802.h"

/******************************************************************************
 * Defines
 ******************************************************************************/
#define BUTTON_PRESSES_MAX 16  ///< Number of maximum button presses to analize in one go

/******************************************************************************
 * Variables
 ******************************************************************************/
uiStateMachine_state uiState;         ///< Holds the current state of the UI
struct GameDataPacket gamePacketIn;   ///< Holds the game packet to show
struct GameDataPacket gamePacketOut;  ///< Holds the game packet to send back
volatile uint8_t red = 0;             ///< Holds the color of the red LEDs. Can be set by MQTT
volatile uint8_t green = 100;         ///< Holds the color of the green LEDs. Can be set by MQTT
volatile uint8_t blue = 50;           ///< Holds the color of the blue LEDs. Can be set by MQTT

uint8_t pressedKeys = 0;              ///< Variable to count how many presses the player has done
uint8_t keysToPress = 0;              ///< Variable that holds the number of new keypresses the user should do
bool playIsDone = false;              ///< Boolean flag to indicate if the player has finished moving.
                                      ///< Useful for COntrol to determine when to send back a play.
uint8_t buttons[BUTTON_PRESSES_MAX];  ///< Array to hold button presses

bool load_reset = false; 
uint8_t cali = 0;
/******************************************************************************
 * Forward Declarations
 ******************************************************************************/

/******************************************************************************
 * Callback Functions
 ******************************************************************************/

/******************************************************************************
 * Task Function
 ******************************************************************************/

/**
 * @fn		void vUiHandlerTask( void *pvParameters )
 * @brief	STUDENT TO FILL THIS
 * @details 	student to fill this
 * @param[in]	Parameters passed when task is initialized. In this case we can ignore them!
 * @return		Should not return! This is a task defining function.
 * @note
 */
void vUiHandlerTask(void *pvParameters)
{
    // Do initialization code here
    SerialConsoleWriteString("UI Task Started!");
    uiState = UI_STATE_START;  // Initial state
	
	char help[32];
	
    // Graphics Test - Students to uncomment to test out the OLED driver if you are using it! 
	
    gfx_mono_init();
	gfx_mono_draw_string("Loading..",0,8, &sysfont);
    /*
	gfx_mono_draw_line(0, 0, 64, 48, GFX_PIXEL_SET);
    gfx_mono_draw_filled_circle(54, 24, 10, GFX_PIXEL_SET, GFX_WHOLE);
	gfx_mono_draw_string("ESE516",0,0, &sysfont);
	*/
	TickType_t xtime = pdMS_TO_TICKS(200);
	int is_dev;

	bool is_fluctuating = false;
	uint16_t pre_fluctuation_value;
	struct WaterLoadPacket LoadDataVar;
	//struct WaterDataPacket bottle_data;
	uint32_t ADC_value;
	int final[2];

	// Here we start the loop for the UI State Machine
	while (1) {
		vTaskDelay(200);
		switch (uiState) {
			case (UI_STATE_START): {
				uiState = UI_STATE_READY;
				LoadDataVar.diff = 0;
				LoadDataVar.total = 0;
				LoadDataVar.weight = 0;
				ADCchip_Init();
				Get_Weight(final);
				
				Acc_read_four();
				gfx_mono_draw_string("gic bottle Ma", 0,8, &sysfont);
				break;
			}

			case (UI_STATE_READY): {
				if (load_reset == true)
				{
					LoadDataVar.total = 0;
					load_reset = false;
				}
				ADCchip_Init();
				Get_Weight(final);
				LoadDataVar.weight = (int)((final[0] - cali) * 4.85);
				snprintf(help, 32, "final_int= %d\r\n",LoadDataVar.weight);
				SerialConsoleWriteString(help);
				/*gfx_mono_draw_string("Total:", 0,25, &sysfont);*/
				snprintf(help, 32, "Total:%3d", LoadDataVar.total);
				
				gfx_mono_draw_string(help, 0,25, &sysfont);
				is_dev = Acc_read(); //accelerate or not
				if (is_dev) {
					//start of dev
					if (!is_fluctuating) {
						is_fluctuating = true;
					}
					SerialConsoleWriteString("start\r\n");
				}
				else {
					if (is_fluctuating) {
						//end of dev
						LoadDataVar.diff = pre_fluctuation_value - LoadDataVar.weight;
						is_fluctuating = false;
						if (LoadDataVar.diff > 0)
						{
							LoadDataVar.total += LoadDataVar.diff;
						}
						SerialConsoleWriteString("end\r\n");
					} else {
					//not dev 
						pre_fluctuation_value = LoadDataVar.weight;
						SerialConsoleWriteString("normal\r\n");
					}
				}
				if (WifiAddWeightDataToQueue(&LoadDataVar)){
					SerialConsoleWriteString("send load ok\r\n");
				} else {
					SerialConsoleWriteString("send load fail\r\n");
				}
				//WifiAddwatervolumeDataToQueue(&LoadDataVar);
				
			
				break;
			}

			case (UI_STATE_dev): {
				break;
			}

			default:  // In case of unforseen error, it is always good to sent state
			// machine to an initial state.
			uiState = UI_STATE_START;
			break;
		}
	
	// After execution, you can put a thread to sleep for some time.
	
	
}
}

void reset_total(void)
{
	load_reset = true;
}


void cali_load(void)
{
	int final[2];
	ADCchip_Init();
	Get_Weight(final);
	cali = final[0];
}



/******************************************************************************
 * Functions
 ******************************************************************************/
void UiOrderShowMoves(struct GameDataPacket *packetIn)
{
    memcpy(&gamePacketIn, packetIn, sizeof(gamePacketIn));
    uiState = UI_STATE_SHOW_MOVES;
    playIsDone = false;  // Set play to false
}

bool UiPlayIsDone(void)
{
    return playIsDone;
}

struct GameDataPacket *UiGetGamePacketOut(void)
{
    return &gamePacketOut;
}

/**
 int UIChangeColors(uint8_t r, uint8_t g, uint8_t b);
 * @brief	Changes the LED colors
 * @param [in]
 * @return
 * @note

*/
void UIChangeColors(uint8_t r, uint8_t g, uint8_t b)
{
    red = r;
    green = g;
    blue = b;
}

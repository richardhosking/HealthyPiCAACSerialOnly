/***************************************************************
//   Basic Arduino program for HealthyPi V4 serial only 
//   Modified from Protocentral library
* 
//   Copyright(2022) Richard Hosking 
//   Bluetooth and Wifi not implemented 
//   
// 
//   Heartrate and respiration computation based on original code from Texas Instruments
//   and Renesas algorithm at 
//
//   This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,  INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR   OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
/////////////////////////////////////////////////////////////////////////////////////*/

// Includes 
#include <SPI.h>
#include <Wire.h>
#include <Update.h>

// Protocentral peripheral devices 
#include "ADS1292r.h"
#include "Protocentral_ecg_resp_signal_processing.h"

#include "myAFE4490_Oximeter.h"

#include "AFE4490_Oximeter.h"


// New library IR Thermometer
#include "MLX90614.h"

#include "arduinoFFT.h"

// Serial port data rate
#define SERIAL_BAUDRATE 115200
// Board switches 
#define PUSH_BUTTON 17
#define SLIDE_SWITCH 16

// Packet setup  
// defines start/stop and length Bytes
#define CES_CMDIF_PKT_START_1 0x0A
#define CES_CMDIF_PKT_START_2 0xFA
#define DATA_LENGTH 20
#define CES_CMDIF_DATA_LEN_LSB 20   // 20 bytes of actual data
#define CES_CMDIF_DATA_LEN_MSB 0
#define CES_CMDIF_TYPE_DATA 0x02
#define CES_CMDIF_PKT_STOP_1 0x00
#define CES_CMDIF_PKT_STOP_2 0x0B
char DataPacketHeader[] = {CES_CMDIF_PKT_START_1, CES_CMDIF_PKT_START_2, CES_CMDIF_DATA_LEN_LSB, CES_CMDIF_DATA_LEN_MSB, CES_CMDIF_TYPE_DATA};
char DataPacketFooter[] = {CES_CMDIF_PKT_STOP_1, CES_CMDIF_PKT_STOP_2};
// Actual data 
char DataPacket[DATA_LENGTH];

// Timing stuff
#define TEMP_READ_INTERVAL 1000

// Peripherals pins 
const int ADS1292_DRDY_PIN = 26;
const int ADS1292_CS_PIN = 13;
const int ADS1292_START_PIN = 14;
const int ADS1292_PWDN_PIN = 27;
const int AFE4490_CS_PIN = 21;
const int AFE4490_DRDY_PIN = 39;
const int AFE4490_PWDN_PIN = 4;
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;


// ECG variables 
// Data successfully loaded fro ADS1292
bool ECG_data_loaded = false;
// ECG leads not connected 
bool leadoff_detected = true;
// Not sure why we need both
uint8_t lead_flag = 0x04;
// 16 bit samples 
int16_t ecg_wave_sample, ecg_filterout;

// Variables for ECG Respiration algorithm 
#define RESP_BUFFER_SIZE 2048 //128*10 secs
int16_t res_wave_sample, resp_filterout;
int16_t resp_buffer[RESP_BUFFER_SIZE];
uint16_t resp_buffer_counter = 0;


volatile uint8_t global_HeartRate = 0;
volatile uint8_t global_HeartRate_prev = 0;
volatile uint8_t global_RespirationRate = 0;
volatile uint8_t global_RespirationRate_prev = 0;


// Instances of peripheral classes 
// ECG Frontend
ads1292r ADS1292R;

// Instance of respiration calculation algorithm    
ads1292r_processing ECG_RESPIRATION_ALGORITHM;

/** Instance of
 * data structure in header file Protocentral_ADS1292r.h
 * 
typedef struct ads1292r_Record{
  signed long raw_ecg;
  signed long raw_resp;
  uint32_t status_reg;
}ads1292r_data; 
* 
*  
*/
ads1292r_data ads1292r_raw_data;

// Oximeter stuff
// Instance
AFE4490 afe4490;
// spO2 calculation algorithm instance in AFE4490_Oximeter.cpp
//spo2_algorithm spo2;
//bool oximeter_initialized = false;
bool oximeter_data_loaded = false;
bool spo2_calc_done = false;

/** Instance of Struct containing Oximeter output data 
 * in Oximeter Header file Protocentral_AFE4490_Oximeter.h
 * 
 * typedef struct afe44xx_Record{
 * int8_t heart_rate;
 * int8_t spo2;
 * signed long IR_data;
 * signed long RED_data;
 * boolean buffer_count_overflow = false;
 * }afe44xx_data;
 * 
 */
afe44xx_data afe44xx_raw_data;
int spo2;

// Temperature stuff
// IR temp sensor instance
MLX90614 mlx90614;
// Temperature variables
double temp, temperature;
uint16_t tempint;
// Temperature timing
volatile long temp_read_timer = 0;
// Flag for presence of Temp sensor 
bool temperatureSensor = false;


void setup()
{
  // Wait for Serial port to open 
  // Sketch will hang if not working
  Serial.begin(SERIAL_BAUDRATE);
  while(!Serial){
      }
  // seems we have to wait a while for the serial port     
  delay(500);
  Serial.println("Setting up Healthy pI V4...");

  // initalize the  data ready and chip select pins:
  pinMode(ADS1292_DRDY_PIN, INPUT);
  pinMode(ADS1292_CS_PIN, OUTPUT);
  pinMode(ADS1292_START_PIN, OUTPUT);
  pinMode(ADS1292_PWDN_PIN, OUTPUT);
  pinMode(A15, OUTPUT);
  pinMode(A13, OUTPUT);
  pinMode(AFE4490_PWDN_PIN, OUTPUT);
  pinMode(AFE4490_CS_PIN, OUTPUT);  //Slave Select
  pinMode(AFE4490_DRDY_PIN, INPUT); // data ready
  
  //set up mode selection pins
  pinMode(SLIDE_SWITCH, OUTPUT);
  pinMode(PUSH_BUTTON, INPUT);
  int buttonState = digitalRead(SLIDE_SWITCH);
  Serial.print("Slide switch :");
  Serial.println(buttonState);
  
  // Interrupts if switches change state
  pinMode(PUSH_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON), push_button_intr_handler, FALLING);
  pinMode(SLIDE_SWITCH, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SLIDE_SWITCH), slideswitch_intr_handler, CHANGE);

  // Set up serial interface to oximeter
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  delay(10);

  // Setup Oximeter
  // Note needs SPI_MODE0
  // AFE4490_PWDN_PIN is in fact called reset pin in the header file 
  //  But connected electrically to AFE Power down
  // Reset in software 
  if (afe4490.afe44xxInit(AFE4490_CS_PIN, AFE4490_PWDN_PIN))
  {
      Serial.println("AFE4490 Oximeter initialized");
  }
  else
  {
      Serial.println("Could not initialize Oximeter!");
  }
    
  // And Initialize ECG frontend - needs SPI_MODE1
  SPI.setDataMode(SPI_MODE1); //Set SPI mode as 1
  delay(10);
  if (ADS1292R.ads1292_Init(ADS1292_CS_PIN, ADS1292_PWDN_PIN, ADS1292_START_PIN))
  {
      Serial.println("ADS1292 ECG Frontend initialized"); 
  }
  else
  {
      Serial.println("Failed to initialize ADS1292 ECG Frontend");
  }
  
  attachInterrupt(digitalPinToInterrupt(ADS1292_DRDY_PIN), ads1292r_interrupt_handler, FALLING); // Digital2 is attached to Data ready pin of AFE is interrupt0 in ARduino
  
  // Start I2C bus for temp sensor 
  Wire.begin(25, 22);
  
  // Check for Temp sensor
  Wire.beginTransmission(MLX90614_I2CADDR);  
  if(Wire.endTransmission () == 0){
    mlx90614.begin();
    Serial.println("Temperature Monitor MLX90614 active");
    temperatureSensor = true;
   }
   else{
       Serial.println("MLX90614 not found");
   }

  Serial.println("Initialization is complete");
  Serial.println("");
   
}

void loop()
{
    // Get raw data from ADS1292 - needs SPI_MODE1
    // Store in ads1292r_raw_data which is a struct in the ADS1292R header
    SPI.setDataMode(SPI_MODE1);
    //delay(10);
    ECG_data_loaded = ADS1292R.getAds1292r_Data_if_Available(ADS1292_DRDY_PIN, ADS1292_CS_PIN, &ads1292r_raw_data);

    if (ECG_data_loaded == true)
    {
        // Check to see if leads are connected 
        if (!((ads1292r_raw_data.status_reg & 0x1f) == 0))
        {
            leadoff_detected = true;
            lead_flag = 0x04;
            ecg_filterout = 0;
            resp_filterout = 0;
            DataPacket[14] = 0;
            DataPacket[16] = 0;
        }
        else
        {
            // If all is OK start data processing
            leadoff_detected = false;
            lead_flag = 0x06;
            
            // Extract data from struct and discard lowest 8 bits of 24
            ecg_wave_sample = (int16_t)(ads1292r_raw_data.raw_ecg >> 8); 
            res_wave_sample = (int16_t)(ads1292r_raw_data.raw_resp >> 8);
            
            // Now copy ECG data stream to the serial packet - try raw data
            // It appears that raw data is cleaner than the filtered sample! 
            memcpy(&DataPacket[0], &ecg_wave_sample, 2); //&ecg_filterout, 2);
            memcpy(&DataPacket[2], &res_wave_sample, 2); //&resp_filterout, 2);
            // Add resp data to resp data buffer 
            if (resp_buffer_counter < RESP_BUFFER_SIZE)
            {
                resp_buffer[resp_buffer_counter] = res_wave_sample;
                resp_buffer_counter++;
            }
            else
            {
                // If resp buffer full
                // (Do ECG/Resp algorithm here)
                //Serial.println("Resp buffer full");
                //delay(1000);
            }
        }
    }
    
        
    // Get Oximeter data needs SPI_MODE0
    SPI.setDataMode(SPI_MODE0);
    //delay(10);
    //oximeter_data_loaded = afe4490.get_AFE4490_Data(&afe44xx_raw_data, AFE4490_CS_PIN, AFE4490_DRDY_PIN);
    // Load data from Oximeter
    if (afe4490.get_AFE4490_Data(&afe44xx_raw_data, AFE4490_CS_PIN, AFE4490_DRDY_PIN))
    {
        // Copy Raw PPG data to packet 
        memcpy(&DataPacket[4], &afe44xx_raw_data.IR_data, sizeof(signed long));
        memcpy(&DataPacket[8], &afe44xx_raw_data.RED_data, sizeof(signed long)); 
        //Serial.println("Oximeter data loaded");
        // Heart rate and respiration rates algorithms are called from ECG/Oximeter Classes
        // Placeholders
        global_HeartRate = afe44xx_raw_data.heart_rate;
        global_RespirationRate = 45;
        spo2 = afe44xx_raw_data.spo2;
        DataPacket[14] = global_RespirationRate;
        DataPacket[15] = afe44xx_raw_data.spo2;
        // originally a 32 bit int - reduce to 8 
        //DataPacket[16] = (uint8_t)afe44xx_raw_data.heart_rate;
        spo2_calc_done = true;
        afe44xx_raw_data.buffer_count_overflow = false;   
        DataPacket[16] = global_HeartRate;
    }
    else
    {
        Serial.println("Oximeter data not available");
    }
    
    // Not Implemented at present 
    DataPacket[17] = 80;  //Blood Pressure Placeholder Diastolic
    DataPacket[18] = 120; //BP Systolic 
    DataPacket[19] = ads1292r_raw_data.status_reg;
    
    // read temperature for every 100ms
    temp = mlx90614.readObjectTempC(); 
    temperature = temp*100 +100;
    tempint = (uint16_t)temperature;
    DataPacket[12] = (uint8_t)tempint;
    DataPacket[13] = (uint8_t)(tempint >> 8);
    
    // Debugging 

    //printPacket();
    //printOximeterVariables(&afe44xx_raw_data);
    //printECGVariables(&ads1292r_raw_data);
    //printspO2variables(&afe4490.internal_data); 
    //Serial.print("Buffer counter ");
    //Serial.println(afe4490.dec_buffer_count); 
    //delay(5000);
    
    // Send completed packet    
    SPI.setDataMode(SPI_MODE0); 
    send_data_serial_port(); 

    printPacket();
    printOximeterVariables(&afe44xx_raw_data);
    printECGVariables(&ads1292r_raw_data);  
    delay(5000);
    
    // Send completed packet    
    SPI.setDataMode(SPI_MODE0); 
    //send_data_serial_port(); 


}


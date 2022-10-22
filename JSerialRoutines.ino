/***************************************************************
//   Basic Arduino program for HealthyPi V4 serial only 
//   Split main file into sections 
*    Arduino will compile all files with *.ino in the current directory together 
*    in alphabetical order 
*    This file contains miscellaneous routines 
* 
//   Copyright(2022) Richard Hosking 
//   Bluetooth and Wifi not implemented 
//   
/
//
/////////////////////////////////////////////////////////////////////////////////////*/

// Interrupt handlers
void push_button_intr_handler()
{
    // Delay to debounce
    delay(100);
    Serial.println("Push Button handler");
}

void slideswitch_intr_handler()
{
    // Delay to debounce
    delay(100);
    Serial.println("SlideSwitch handler");
  
}

// send data in packet over serial/USB 
// It seems quite hard to concatenate arrays in C++!
// so send each array individually

/**
 * Packet structure 
 * 8 bit bytes
 *  
 *  0 Start 0x0A
 *  1 Start 0xFA
 *  2 datalength LSB
 *  3 datalength MSB
 *  4 0x02
 *  These 4 bytes are 0x00 if lead off detected
 *  5 ECG LSB
 *  6 ECG MSB
 *  7 Resp LSB
 *  8 Resp MSB
 *  9-12 Oximeter IR data LSB first
 *  13-16 0ximeter Red data LSB first 
 *  Temp - Deg C in hundredths of a degree C
 *  17 Temperature LSB
 *  18 Temperature MSB
 *  19 Resp rate/min
 *  20 spO2 % saturation
 *  21 Heartrate BPM
 *  22 BP diastolic - not implemented = 80
 *  23 BP Systolic - not implemented
 *  24 spO2 probe status
 *  25 Stop 0x00
 *  26 Stop 0x0B
 */   
void send_data_serial_port()
{

  for (int i = 0; i < 5; i++)
  {
    Serial.write(DataPacketHeader[i]); 
  }

  for (int i = 0; i < DATA_LENGTH; i++)
  {
    Serial.write(DataPacket[i]); 
  }

  for (int i = 0; i < 2; i++)
  {
    Serial.write(DataPacketFooter[i]); 
  }
}

// Debugging - write out packet 

void printPacket()
{
  Serial.printf("Packet Header start1: %02x", DataPacketHeader[0]);
  Serial.println("");
  Serial.printf("Packet Header start2: %02x", DataPacketHeader[1]);
  Serial.println("");
  Serial.printf("Packet Header Datalength LSB: %02x", DataPacketHeader[2]);
  Serial.println("");
  Serial.printf("Packet Header Datalength MSB: %02x", DataPacketHeader[3]);
  Serial.println(""); 
  Serial.printf("Packet Header Byte 4: %02x", DataPacketHeader[4]);
  Serial.println("");
  Serial.println("");
  
  Serial.println("First 4 bytes are 0 if ECG leadoff detected ");    
  Serial.print("Data ECG LSB ");
  Serial.println(int(DataPacket[0])); 
  Serial.print("Data ECG MSB ");
  Serial.println(int(DataPacket[1]));  
  Serial.print("Data Resp LSB ");
  Serial.println(int(DataPacket[2])); 
  Serial.print("Data Resp MSB ");
  Serial.println(int(DataPacket[3]));
  Serial.println("");
    
  Serial.println("Next 4 bytes Oximeter IR data LSB first ");
  Serial.print("IR LSB ");
  Serial.println(int(DataPacket[4]));  
  Serial.println(int(DataPacket[5]));  
  Serial.println(int(DataPacket[6]));  
  Serial.println(int(DataPacket[7]));  
  Serial.println("");
    
  Serial.println("Next 4 bytes 0ximeter Red data LSB first ");
  Serial.print("Red LSB ");
  Serial.println(int(DataPacket[8])); 
  Serial.println(int(DataPacket[9]));  
  Serial.println(int(DataPacket[10]));
  Serial.println(int(DataPacket[11]));
  Serial.println("");
    
  Serial.println("Temp - Deg C in hundredths of a degree C ");
  Serial.print("Temp LSB ");
  Serial.println(int(DataPacket[12]));  
  Serial.print("Temp MSB ");
  Serial.println(int(DataPacket[13]));
  Serial.print("Resp rate/min ");
  Serial.println(int(DataPacket[14]));  
  Serial.print("spo2 % saturation 0 if invalid ");
  Serial.println(int(DataPacket[15]));  
  Serial.print("Heartrate BPM ");
  Serial.println(int(DataPacket[16]));
   
  Serial.print("BP Diastolic not implemented ");
  Serial.println(int(DataPacket[17]));  
  Serial.print("BP Systolic Not implemented ");
  Serial.println(int(DataPacket[18]));
  
  Serial.print("spO2 Probe status  ");
  Serial.println(int(DataPacket[19]));  
  
  Serial.println("");
  
  Serial.printf("Footer Byte 0 Stop1: %02x", DataPacketFooter[0]);
  Serial.println(""); 
  Serial.printf("Footer Byte 1 Stop2: %02x", DataPacketFooter[1]);
  Serial.println("");
  Serial.println("");   
}

// And write out structs for debugging
/**typedef struct afe44xx_Record{
  uint8_t heart_rate;
  uint8_t spo2;
  signed long IR_data;
  signed long RED_data;
  boolean buffer_count_overflow = false;
}afe44xx_data;*/

// Pass a pointer to the struct
void printOximeterVariables(afe44xx_data *data)
{
    Serial.println("ADE4490 Raw data struct");
    Serial.print("Heart Rate  ");   
    Serial.println(data->heart_rate);
    Serial.print("SpO2  ");   
    Serial.println(data->spo2);    
    Serial.print("IR_data  ");
    Serial.println(data->IR_data);
    Serial.printf("IR Data Hex : %x", data->IR_data);
    Serial.println("");          
    Serial.print("RED Data  ");   
    Serial.println(data->RED_data);  
    Serial.printf("RED Data Hex : %x", data->RED_data);
    Serial.println("");          
  
    Serial.print("Buffer count overflow ");   
    Serial.println(data->buffer_count_overflow);    
    Serial.println("");
    Serial.println("");
}

/**typedef struct ads1292r_Record{
  signed long raw_ecg;
  signed long raw_resp;
  uint32_t status_reg;
}ads1292r_data; */

void printECGVariables(ads1292r_data *data)
{
    Serial.println("ADS1292 Raw data struct");
    Serial.print("Raw ECG ");   
    Serial.println(data->raw_ecg);
    Serial.print("Raw Resp ");   
    Serial.println(data->raw_resp); 
    Serial.printf("Raw ECG Hex : %x", data->raw_ecg);
    Serial.println("");          
    //Serial.println(data->IR_data);
    Serial.printf("Raw Resp Hex : %x", data->raw_resp);
    Serial.println("");                 
    Serial.print("Status Register ");   
    Serial.println(data->status_reg);
    Serial.println("");
    Serial.println("");
}
/**
typedef struct afe44xx_Internal{
    uint8_t n_spo2;
    uint8_t n_heart_rate;
    uint8_t n_resp_rate;
    uint8_t buffer_length = BUFFER_FULL;
    bool ch_spo2_valid = false;
    bool ch_hr_valid = false;
    bool ch_resp_valid = false;    
}afe44xx_internal_data;
*/

void printspO2variables(afe44xx_Internal *data)
{
    Serial.println("Internal data for SpO2 algorithm");
    Serial.print("O2 Sats "); 
    Serial.println(data->n_spo2);
    
    Serial.print("Heart Rate ");   
    Serial.println(data->n_heart_rate);
     
    Serial.printf("BUFFER LENGTH ");
    Serial.println(data->buffer_length);
                   
    Serial.print("SpO2 valid ");   
    Serial.println(data->ch_spo2_valid);
    
    Serial.print("Heart Rate valid ");   
    Serial.println(data->ch_hr_valid);
    
    Serial.print("Resp valid ");   
    Serial.println(data->ch_resp_valid);
    
    Serial.println("");
    Serial.println("");
}

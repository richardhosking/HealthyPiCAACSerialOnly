//////////////////////////////////////////////////////////////////////////////////////////
//
//    Arduino library for the AFE4490 Pulse Oximeter Shield
//    from Protocentral code modified by Richard Hosking 2022
//
//    This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//   
/////////////////////////////////////////////////////////////////////////////////////////
#include "myAFE4490_Oximeter.h"
#include "myoximeter_algorithm.h"

// Instance of O2 calculation algorithm in oximeter_algorithm.h
spo2_algorithm Spo2;

/**
 * Main Oximeter function
 * Passes a pointer to typedef struct of Type afe44xx_data
 * afe44xx_data *afe44xx_raw_data
 * which holds HR and spO2
 * and pin variables
 * 
 */ 
 // Constructor 
AFE4490 :: AFE4490()
{
    // Initialize struct to hold internal data to pass to spO2/resp/HR routine
    internal_data.n_spo2 = 50;
    internal_data.n_heart_rate = 20;
    internal_data.n_resp_rate = 10;
    internal_data.buffer_length = BUFFER_FULL;
    internal_data.ch_spo2_valid = false;
    internal_data.ch_hr_valid = false;
    internal_data.ch_resp_valid = false; 
    dec = 0;
    dec_buffer_count = 0;
    
}
         

boolean AFE4490 :: get_AFE4490_Data (afe44xx_data *afe44xx_raw_data,const int chip_select,const int data_ready)
{
  // Enable SPI READ (Disabled on reset)
  afe44xxWrite(CONTROL0, 0x000001,chip_select);
  // Read IR and RED values
  // These will be 24 bits
  // 22 bits are valid - 2s complement format
  // IRTemp is a 32 bit unsigned long 
  IRtemp = afe44xxRead(LED1VAL,chip_select);
  
  // Do we need to do this before each SPI read?
  afe44xxWrite(CONTROL0, 0x000001,chip_select);  
  REDtemp = afe44xxRead(LED2VAL,chip_select);
  
  afe44xx_data_ready = true;
  
  // Discard top 10 bits (22 bits only from ADC)
  IRtemp = (unsigned long) (IRtemp << 10);
  IRtemp = (unsigned long) (IRtemp >> 10);
  afe44xx_raw_data->IR_data = (signed long) (IRtemp);
  
  REDtemp = (unsigned long) (REDtemp << 10);
  REDtemp = (unsigned long) (REDtemp >> 10);  
  afe44xx_raw_data->RED_data = (signed long) (REDtemp);
    
  // decimate data 
  // if dec = 20 and sample rate is 500 samples/sec
  // aun buffers will be updated at 25 samples/sec
  // Hence 127 samples is 5 sec worth  
  if (dec == DECIMATE)
  {
      // Decimate and trancate to 16 bits
      aun_ir_buffer[dec_buffer_count] = (uint16_t) ((afe44xx_raw_data->IR_data) >> 6);  // ? should this be >>6 22 bits to 16 
      aun_red_buffer[dec_buffer_count] = (uint16_t) ((afe44xx_raw_data->RED_data) >> 6);
      dec_buffer_count++;
      dec = 0;
  }
  dec++;
   // When Buffer has approx 5 seconds of data 
  if (dec_buffer_count > BUFFER_FULL)
  {
    // Call Routine to estimate spO2 and heart rate 
    // Pass to routine:
    // IR buffer, Buffer length = 127, Red buffer, pO2 sats, pO2 sats flag, Heart rate, heart rate flag, resp resp flag, BUFFER_FULL in struct
    // Note passing arrays dont pass address 
    // Passing struct - pass address??  
    Spo2.estimate_spo2(aun_ir_buffer, aun_red_buffer, &internal_data);
    
    // move data into the struct afe44xx_raw_data
    dec_buffer_count = 0;
    afe44xx_raw_data->buffer_count_overflow = true;
  }
  // move data into the struct afe44xx_raw_data
  afe44xx_raw_data->spo2 = internal_data.n_spo2;
  afe44xx_raw_data->heart_rate = internal_data.n_heart_rate;
  afe44xx_data_ready = false;
  return true;
}

bool AFE4490 :: afe44xxInit (const int chip_select,const int power_down_pin)
{
  // In fact this is a power down pin 
  // (Original code was calling it a reset)
  digitalWrite(power_down_pin, LOW);
  // Power down when pin low - chip enable
  digitalWrite(power_down_pin, HIGH);
  
  // Software reset pulse to CONTROL0
  // D3 High resets all internal registers 
  // D3 then self clears to 0
  afe44xxWrite(CONTROL0, 0x000000,chip_select);
  afe44xxWrite(CONTROL0, 0x000008,chip_select);
  delay(50);
  // Transimpedance amplifier Gain Register  
  // Gain stage 2 = 0dB 
  // Filter component for first stage Cf= 5pF
  // Feedback resistor sets Photodiode current RF = 500K
  // (Default settings)
  afe44xxWrite(TIAGAIN, 0x000000,chip_select);

  // Transimpedance Amplifier and Ambient Cancellation Stage Gain Register 
  // Ambient DAC 0uA Ambient Filter corner freq 500Hz
  // LED2 Stage 2 amp 0dB Filter Cf = 5pf Feedback resistor Rf = 250K
  afe44xxWrite(TIA_AMB_GAIN, 0x000001,chip_select);

  // LEDCNTRL register sets LED currents 
  // Full scale 150mA (default) 
  // LED1 and LED2 current (20/256)*150mA ie approx 11.7mA
  // Original comment LED_RANGE=100mA, LED=50mA ?? this is not correct by my reading
  afe44xxWrite(LEDCNTRL, 0x001414,chip_select);

  // CONTROL2 Various functions 
  // 0.75V reference voltage to ADC
  // No reset clock on external pin 
  // Internal ADC (not bypassed) 
  // LEDs H bridge driver
  // Bit D10 DIGOUT_TRISTATE: Digital output 3-state mode 0 default - not tristate 
  // this will load SPI bus if used by other devices 
  // Crystal oscillator enabled 
  // Fast Diagnostics mode
  // TX, RX and AFE powered up   
  afe44xxWrite(CONTROL2, 0x000000,chip_select);
  
  // CONTROL1
  // Timers ON, some Clocks to ALM pins, average 7 samples ADC  ?Should be 0x000F03 
  // Original comment said 3 samples  
  afe44xxWrite(CONTROL1, 0x000F07,chip_select); 
  
  // PRPCOUNT register sets up Pulse repetition rate which determines samples/sec
  // value here is 0x001F3F which is 7999
  // Change to 1F40 or 8000
  // equivalent to 500 reps/sec with a 4MHz clock ie 4000000/8000
  afe44xxWrite(PRPCOUNT, 0X001F40,chip_select);  
  
  // Timing of various actions - LEDs on and off, conversions etc
  // Can be set with these registers 
  afe44xxWrite(LED2STC, 0X001770,chip_select);
  afe44xxWrite(LED2ENDC, 0X001F3E,chip_select);
  afe44xxWrite(LED2LEDSTC, 0X001770,chip_select);
  afe44xxWrite(LED2LEDENDC, 0X001F3F,chip_select);
  afe44xxWrite(ALED2STC, 0X000000,chip_select);
  afe44xxWrite(ALED2ENDC, 0X0007CE,chip_select);
  afe44xxWrite(LED2CONVST, 0X000002,chip_select);
  afe44xxWrite(LED2CONVEND, 0X0007CF,chip_select);
  afe44xxWrite(ALED2CONVST, 0X0007D2,chip_select);
  afe44xxWrite(ALED2CONVEND, 0X000F9F,chip_select);
  afe44xxWrite(LED1STC, 0X0007D0,chip_select);
  afe44xxWrite(LED1ENDC, 0X000F9E,chip_select);
  afe44xxWrite(LED1LEDSTC, 0X0007D0,chip_select);
  afe44xxWrite(LED1LEDENDC, 0X000F9F,chip_select);
  afe44xxWrite(ALED1STC, 0X000FA0,chip_select);
  afe44xxWrite(ALED1ENDC, 0X00176E,chip_select);
  afe44xxWrite(LED1CONVST, 0X000FA2,chip_select);
  afe44xxWrite(LED1CONVEND, 0X00176F,chip_select);
  afe44xxWrite(ALED1CONVST, 0X001772,chip_select);
  afe44xxWrite(ALED1CONVEND, 0X001F3F,chip_select);
  afe44xxWrite(ADCRSTCNT0, 0X000000,chip_select);
  afe44xxWrite(ADCRSTENDCT0, 0X000000,chip_select);
  afe44xxWrite(ADCRSTCNT1, 0X0007D0,chip_select);
  afe44xxWrite(ADCRSTENDCT1, 0X0007D0,chip_select);
  afe44xxWrite(ADCRSTCNT2, 0X000FA0,chip_select);
  afe44xxWrite(ADCRSTENDCT2, 0X000FA0,chip_select);
  afe44xxWrite(ADCRSTCNT3, 0X001770,chip_select);
  afe44xxWrite(ADCRSTENDCT3, 0X001770,chip_select);
  delay(10);
  return true;
}

void AFE4490 :: afe44xxWrite (uint8_t address, uint32_t data,const int chip_select)
{
  digitalWrite (chip_select, LOW); // enable device for writing data
  SPI.transfer (address); // send address to device
  SPI.transfer ((data >> 16) & 0xFF); // write top 8 bits
  SPI.transfer ((data >> 8) & 0xFF); // write middle 8 bits
  SPI.transfer (data & 0xFF); // write bottom 8 bits
  digitalWrite (chip_select, HIGH); // disable device for writing 
}

unsigned long AFE4490 :: afe44xxRead (uint8_t address,const int chip_select)
{
  unsigned long data = 0;
  digitalWrite (chip_select, LOW); // enable device for transfer
  SPI.transfer (address); // send address to device
  data |= ((unsigned long)SPI.transfer (0) << 16); // read top 8 bits data
  data |= ((unsigned long)SPI.transfer (0) << 8); // read middle 8 bits  data
  data |= SPI.transfer (0); // read bottom 8 bits data
  digitalWrite (chip_select, HIGH); // disable device
  return data; // return with 24 bits of read data
}



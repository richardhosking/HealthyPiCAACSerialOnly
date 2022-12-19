
/*******************************************************
 * Calculate SpO2 Function - commented to understand it!
 * 
 * Variables passed to function 
 * uint16_t *pun_ir_buffer    pointer to IR buffer
 * uint16_t n_ir_buffer_length
 * uint16_t *pun_red_buffer   pointer to Red buffer 
 * uint16_t *pn_spo2           Pointer to O2 result
 * int8_t *pch_spo2_valid     pointer to Flag to indicate valid spO2
 * uint16_t *pn_heart_rate     pointer to heart rate
 * int8_t *pch_hr_valid       Pointer to HR valid flag
 * 
 * Algorithm
 * spO2 = 
 * sum buffer => average = DC offset
 * subtract DC offset and invert results 
 * 4 point moving average to remove noise 
 * calculate average of buffer again and use this as a threshold 
 * threshold
 * set threshold at max and in limits 30-60
 * 
 * an_ir_valley_locs
 * array of 15 32 bit nos to store locations of valleys (peaks)
 * 
*/

#include "myAFE4490_Oximeter.h"
#include "myoximeter_algorithm.h"
#include <math.h>

// Constructor
spo2_algorithm::spo2_algorithm()
{
    
}

// Main function 
void spo2_algorithm :: estimate_spo2(uint16_t *pun_ir_buffer, uint16_t *pun_red_buffer, afe44xx_internal_data *internal_data)
{

/** Internal data passed to spO2/resp/HR routine
 * as well as sample buffers
typedef struct afe44xx_Internal{
    uint8_t n_spo2;
    uint8_t n_heart_rate;
    uint8_t n_resp_rate;
    uint8_t buffer_length;
    bool ch_spo2_valid = false;
    bool ch_hr_valid = false;
    bool ch_resp_valid = false;
    // debugging 
    uint16_t test1 = 0;
    uint16_t test2 = 0;
    uint16_t test3 = 0;
    bool spO2_calc_done = false;    
}afe44xx_internal_data;
*/
    uint8_t pn_spo2 = internal_data->n_spo2;
    uint8_t pn_heart_rate = internal_data->n_heart_rate;
    
    // Have to be careful here with C++ interger arithmetic rules 
    int16_t buffer_length = internal_data->buffer_length;
    bool pch_spo2_valid = internal_data->ch_spo2_valid;
    bool pch_hr_valid = internal_data->ch_hr_valid;
    internal_data->test1 = 0;
    internal_data->test2 = 0;
    internal_data->test3 = 0;   
   
  // iterate through the sample and find the minimum and maximum of all points 
  sample_max = pun_ir_buffer[0];
  sample_min = pun_ir_buffer[0];
   
  for (k=0 ; k < buffer_length ; k++ )
  {
    if (sample_min > pun_ir_buffer[k])
    {
        sample_min = pun_ir_buffer[k];
    }
    
    if (sample_max < pun_ir_buffer[k])
    {
        sample_max = pun_ir_buffer[k];
    }
        
  }
  
  // remove DC - use sample_min as effective DC offset - add 1 to avoid divide by zero
  for (k=0 ; k <= buffer_length ; k++ )
  {
    an_x[k] = (pun_ir_buffer[k] - sample_min + 1);
  }
  
  // set threshold as halfway between sample_max and sample_ min
  // right shift to divide by 2 
  threshold = ((sample_max - sample_min)>>1);
  
  // Save to struct
  internal_data->sample_max = sample_max;
  internal_data->sample_min = sample_min;    
  internal_data->threshold = threshold;

  // 4 pt Moving Average - noise reduction
  // but doesnt deal with last 4 data points 
  // 4 points are eccentric - does this matter?
  // This routine does seem to improve output accuracy 
  for(k=0; k< BUFFER_LENGTH-3; k++)
  {
    // Use right shift to divide again
    an_x[k] = ((an_x[k]+an_x[k+1]+ an_x[k+2]+ an_x[k+3])>>2);
  }
  // deal with these last 4 points
  for(k=BUFFER_LENGTH; k>= BUFFER_LENGTH-3; k--)
  {
    // Use right shift to divide again
    an_x[k] = ((an_x[k]+an_x[k-1]+ an_x[k-2]+ an_x[k-3])>>2);
  }  

// Set all valley locations to 0 
  for ( k=0 ; k<15;k++)
  {
    an_ir_valley_locs[k]=0;
  }

  // my peak finding routine
  // returns number of peaks found 
  n_npks = my_find_peaks(an_x, an_ir_valley_locs, internal_data);

  // If enough peaks 
  // Calculate Heart Rate
  // Calculate the average distance between peaks n_peak_interval_sum/n_npks-1   
  n_peak_interval_sum = 0; 
  if (n_npks>3 && n_npks<15)
  {
    for (k=1; k<n_npks; k++)
    {
      n_peak_interval_sum += (an_ir_valley_locs[k] - an_ir_valley_locs[k-1]) ;
    }
    av_peak_interval = (float)n_peak_interval_sum/(n_npks-1);
    // SF_spO2 is the sampling frequency for oximeter #DEFINE in spo2_algorithm.h 
    // Set at 25 (?samples/sec)
    float HR =(float)((SF_spo2*60)/av_peak_interval);
    pn_heart_rate = int(HR);
    pch_hr_valid  = true;
  }
  else
  {
    pn_heart_rate = -999; // unable to calculate because # of peaks are too small
    pch_hr_valid  = false;
  }

  //  load raw value again for SPO2 calculation : RED(=y) and IR(=x)
  // Do we need to do this?
  
  for (k=0 ; k < buffer_length ; k++ )
  {
      an_x[k] =  pun_ir_buffer[k] ;
      an_y[k] =  pun_red_buffer[k] ;
  }

  // Choose a threshold crossing point and iterate forwards to the next crossing point
  // to find the minimum and maximum points for that cycle for the IR buffer
  local_maxIR = 0;
  local_minIR = sample_max;
  for (k = an_ir_valley_locs[1]; k < an_ir_valley_locs[2]; k++)
  {
      if (an_x[k] < local_minIR)
      {
          local_minIR = an_x[k];
          local_minRED = an_y[k];
      }
      if (an_x[k] > local_maxIR)
      {
          local_maxIR = an_x[k];
          local_maxRED = an_y[k];
      }
  }
  // Using a single interval - would probably be improved by using several and choosing the median 
   
  AC_IR = local_maxIR - local_minIR;
  DC_IR = local_minIR;
  AC_RED = local_maxRED - local_minRED;
  DC_RED = local_minRED;
  
  // Intermediate Variable - if any 0 then error 
  if (AC_IR != 0 && DC_IR !=0 && AC_RED != 0 && DC_RED !=0)
  {
      float red = float(AC_RED)/float(DC_RED);
      float ir = float(AC_IR)/float(DC_IR);
      
      // scale R to lookup table 100 seems to give a reasonable accuracy 
      R = (red/ir)*100;
      
      // Lookup table is based on this quadratic - see above  
      //R =  -45.060*R*R/10000 + 30.354 *R/100 + 94.845 ;
  }
  else 
  {
      // Invalid easurement 
      R = 200;
  }
  
  lookup = int(R);
  
  // Lookup table
   if(lookup > 1 && lookup <184)
  {
    n_spo2_calc= uch_spo2_table[lookup] ;
    pn_spo2 = n_spo2_calc ;
    pch_spo2_valid  = 1;
  }
  else
  {
    pn_spo2 =  0 ;   // do not use SPO2 since signal is out of range
    pch_spo2_valid  = 0;
  }
     
      // Save calculated data 
    internal_data->n_spo2 = pn_spo2;
    internal_data->n_heart_rate = pn_heart_rate;
    internal_data->ch_spo2_valid = pch_spo2_valid;
    internal_data->ch_hr_valid = pch_hr_valid;
    internal_data->spO2_calc_done = true;
    internal_data-> test1 = R;
    internal_data->test2 = AC_RED;
    internal_data->test3 = DC_RED;

  for (k=0 ; k <= 12 ; k++ )
  {
      internal_data->testbuffer[k] = 0;
      internal_data->testbuffer[k] = an_ir_valley_locs[k];
 
  }

 
}

// My (naive) peak finding routine seems to work quite well!
int spo2_algorithm :: my_find_peaks(uint16_t *AC_data_buffer, int16_t *peaks_buffer, afe44xx_internal_data *internal_data)
{
  int k = internal_data->buffer_length;
  int j=0;
  int threshold = internal_data->threshold;
  
  // Iterate over AC buffer to find points where data crosses threshold
  for (int i=1; i <= k; i++)
  {
      if (AC_data_buffer[i] >= threshold && AC_data_buffer[i-1] < threshold)
      {
          peaks_buffer[j] = i;
          j++;
      }
      // too many peaks 
      if (j>14)
      {
          j=14;
      }
  }
  // return number of peaks found 
  return j;
    
}

#include "AFE4490_Oximeter.h"
#include "oximeter_algorithm.h"

// Constructor
spo2_algorithm::spo2_algorithm()
{
    
}

/*******************************************************
 * Calculate SpO2 Function - commented to understand it!
 * 
 * Variables passed to function 
 * uint16_t *pun_ir_buffer    pointer to IR buffer
 * int32_t n_ir_buffer_length
 * uint16_t *pun_red_buffer   pointer to Red buffer 
 * int32_t *pn_spo2           Pointer to O2 result
 * int8_t *pch_spo2_valid     pointer to Flag to indicate valid spO2
 * int32_t *pn_heart_rate     pointer to heart rate
 * int8_t *pch_hr_valid       Pointer to HR valid flag
 * 
 * Algorithm
 * spO2 = 
 * sum buffer => average = DC offset
 * subtract DC offset and invert results 
 * 4 point moving average to remove noise 
 * calculate average of buffer again and use this as a threshold 
 * n_th1
 * set threshold at max and in limits 30-60
 * 
 * an_ir_valley_locs
 * array of 15 32 bit nos to store locations of valleys (peaks)
 * 
*/

void spo2_algorithm :: estimate_spo2(uint16_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint16_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate, int8_t *pch_hr_valid)
{
  // Place these variables in class definition
  /*uint32_t DC_ir_mean,un_only_once ;
  int32_t k, n_i_ratio_count;
  int32_t i, s, m, n_exact_ir_valley_locs_count, n_middle_idx;
  int32_t n_th1, n_npks, n_c_min;
  int32_t an_ir_valley_locs[15] ;
  int32_t n_peak_interval_sum;
  int32_t n_y_ac, n_x_ac;
  int32_t n_spo2_calc;
  int32_t n_y_dc_max, n_x_dc_max;
  int32_t n_y_dc_max_idx, n_x_dc_max_idx;
  int32_t an_ratio[5], n_ratio_average;
  int32_t n_nume, n_denom ;
*/
  // calculates DC mean and subtract DC from ir
  DC_ir_mean =0;

  for (k=0 ; k<n_ir_buffer_length ; k++ )
  {
    DC_ir_mean += pun_ir_buffer[k] ;
  }

  DC_ir_mean =DC_ir_mean/n_ir_buffer_length ;

  // remove DC and invert signal so that we can use peak detector as valley detector
  for (k=0 ; k<n_ir_buffer_length ; k++ )
  {
    an_x[k] = -1*(pun_ir_buffer[k] - DC_ir_mean) ;
  }

  // 4 pt Moving Average - noise reduction
  for(k=0; k< BUFFER_SIZE-MA4_SIZE; k++)
  {
    an_x[k]=( an_x[k]+an_x[k+1]+ an_x[k+2]+ an_x[k+3])/(int)4;
  }

  // calculate threshold
  n_th1=0;

  for ( k=0 ; k<BUFFER_SIZE ;k++)
  {
    // Sum whole buffer
    n_th1 +=  an_x[k];
  }
  // Threshold will be 50% of the average AC amplitude
  n_th1=  n_th1/ ( BUFFER_SIZE);

  if( n_th1<30)
  {
    n_th1=30; // min allowed
  }

  if( n_th1>60)
  {
    n_th1=60; // max allowed
  }

// Set all valley locations to 0 
  for ( k=0 ; k<15;k++)
  {
    an_ir_valley_locs[k]=0;
  }

  // since we flipped signal, we use peak detector as valley detector
  // Minimum peak height is n_th1 (threshold)
  // Min distance between peaks is 4 samples (how are samples timed?)
  // Max number of peaks for calculation is 15
  // find_peak is main function - it calls a number of subfunctions
  // n_pks is number of peaks found
  // an_ir_valley_locs is array of 15 32 bit nos to store locations of valleys (peaks)
  // pass to function:
  // array of valley locations
  // pointer to number of peaks
  // IR buffer 
  // BUFFER LENGTH
  // Threshold (peak height)
  // 4 peak distance minimum
  // 15 max number of peaks
  // Would be better to use a struct?
  find_peak(an_ir_valley_locs, &n_npks, an_x, BUFFER_SIZE, n_th1, 4, 15 );  
  n_peak_interval_sum =0;

  if (n_npks>=2)
  {
    // Calculate the average distance between peaks n_peak_interval_sum 
    for (k=1; k<n_npks; k++)
    {
      n_peak_interval_sum += (an_ir_valley_locs[k] -an_ir_valley_locs[k -1] ) ;
    }

    n_peak_interval_sum =n_peak_interval_sum/(n_npks-1);
    
    // Calculate Heart Rate from oximeter
    // SF_spO2 is the sampling frequency for oximeter #DEFINE in spo2_algorithm.h 
    // Set at 25 (?samples/sec)
    *pn_heart_rate =(int32_t)( (SF_spo2*60)/ n_peak_interval_sum );
    *pch_hr_valid  = 1;
  }
  else
  {
    *pn_heart_rate = -999; // unable to calculate because # of peaks are too small
    *pch_hr_valid  = 0;
  }

  //  load raw value again for SPO2 calculation : RED(=y) and IR(=X)
  for (k=0 ; k<n_ir_buffer_length ; k++ )
  {
      an_x[k] =  pun_ir_buffer[k] ;
      an_y[k] =  pun_red_buffer[k] ;
  }

  // find precise min near an_ir_valley_locs
  n_exact_ir_valley_locs_count =n_npks;

  //using exact_ir_valley_locs , find ir-red DC andir-red AC for SPO2 calibration an_ratio
  //finding AC/DC maximum of raw

  n_ratio_average =0;
  n_i_ratio_count = 0;

  for(k=0; k< 5; k++)
  {
    an_ratio[k]=0;
  }

  for (k=0; k< n_exact_ir_valley_locs_count; k++)
  {

    if (an_ir_valley_locs[k] > BUFFER_SIZE )
    {
      *pn_spo2 =  -999 ; // do not use SPO2 since valley loc is out of range
      *pch_spo2_valid  = 0;
      return;
    }

  }
  // find max between two valley locations
  // and use an_ratio betwen AC component of Ir & Red and DC compoent of Ir & Red for SPO2
  for (k=0; k< n_exact_ir_valley_locs_count-1; k++)
  {
    n_y_dc_max= -16777216 ;
    n_x_dc_max= -16777216;

    if (an_ir_valley_locs[k+1]-an_ir_valley_locs[k] >3)
    {

        for (i=an_ir_valley_locs[k]; i< an_ir_valley_locs[k+1]; i++)
        {

          if (an_x[i]> n_x_dc_max)
          {
            n_x_dc_max =an_x[i]; n_x_dc_max_idx=i;
          }

          if (an_y[i]> n_y_dc_max)
          {
            n_y_dc_max =an_y[i]; n_y_dc_max_idx=i;
          }

        }
        n_y_ac= (an_y[an_ir_valley_locs[k+1]] - an_y[an_ir_valley_locs[k] ] )*(n_y_dc_max_idx -an_ir_valley_locs[k]); //red
        n_y_ac=  an_y[an_ir_valley_locs[k]] + n_y_ac/ (an_ir_valley_locs[k+1] - an_ir_valley_locs[k])  ;
        n_y_ac=  an_y[n_y_dc_max_idx] - n_y_ac;    // subracting linear DC components from raw
        n_x_ac= (an_x[an_ir_valley_locs[k+1]] - an_x[an_ir_valley_locs[k] ] )*(n_x_dc_max_idx -an_ir_valley_locs[k]); // ir
        n_x_ac=  an_x[an_ir_valley_locs[k]] + n_x_ac/ (an_ir_valley_locs[k+1] - an_ir_valley_locs[k]);
        n_x_ac=  an_x[n_y_dc_max_idx] - n_x_ac;      // subracting linear DC compoenents from raw
        n_nume=( n_y_ac *n_x_dc_max)>>7 ; //prepare X100 to preserve floating value
        n_denom= ( n_x_ac *n_y_dc_max)>>7;

        if (n_denom>0  && n_i_ratio_count <5 &&  n_nume != 0)
        {
          an_ratio[n_i_ratio_count]= (n_nume*100)/n_denom ; //formula is ( n_y_ac *n_x_dc_max) / ( n_x_ac *n_y_dc_max) ;
          n_i_ratio_count++;
        }

    }

  }
  // choose median value since PPG signal may varies from beat to beat
  sort_ascend(an_ratio, n_i_ratio_count);
  n_middle_idx= n_i_ratio_count/2;

  if (n_middle_idx >1)
  {
    n_ratio_average =( an_ratio[n_middle_idx-1] +an_ratio[n_middle_idx])/2; // use median
  }
  else
  {
    n_ratio_average = an_ratio[n_middle_idx ];
  }

  if( n_ratio_average>2 && n_ratio_average <184)
  {
    n_spo2_calc= uch_spo2_table[n_ratio_average] ;
    *pn_spo2 = n_spo2_calc ;
    *pch_spo2_valid  = 1;//  float_SPO2 =  -45.060*n_ratio_average* n_ratio_average/10000 + 30.354 *n_ratio_average/100 + 94.845 ;  // for comparison with table
  }
  else
  {
    *pn_spo2 =  -999 ; // do not use SPO2 since signal an_ratio is out of range
    *pch_spo2_valid  = 0;
  }

}


void spo2_algorithm :: find_peak( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num )
/**
  \brief        Find peaks
  \variables    int32_t *pn_locs 
  \par          Details
                Find at most MAX_NUM peaks above MIN_HEIGHT separated by at least MIN_DISTANCE

  \retval       None
*/
{
  find_peak_above( pn_locs, n_npks, pn_x, n_size, n_min_height );
  remove_close_peaks( pn_locs, n_npks, pn_x, n_min_distance );
  *n_npks = min( *n_npks, n_max_num );
}

void spo2_algorithm :: find_peak_above( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height )
/**
  \brief        Find peaks above n_min_height
  \par          Details
                Find all peaks above MIN_HEIGHT

  \retval       None
*/
{
  int32_t i = 1, n_width;
  *n_npks = 0;

  while (i < n_size - 1)
  {

    if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i - 1])
    {   // find left edge of potential peaks
      n_width = 1;

      while (i + n_width < n_size && pn_x[i] == pn_x[i + n_width]) // find flat peaks
      {
        n_width++;
      }

      if (pn_x[i] > pn_x[i + n_width] && (*n_npks) < 15 )
      {   // find right edge of peaks
        pn_locs[(*n_npks)++] = i;
        // for flat peaks, peak location is left edge
        i += n_width + 1;
      }
      else
      {
        i += n_width;
      }

    }
    else
    {
      i++;
    }

  }

}

void spo2_algorithm :: remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance)
/**
  \brief        Remove peaks
  \par          Details
                Remove peaks separated by less than MIN_DISTANCE

  \retval       None
*/
{
  int32_t i, j, n_old_npks, n_dist; /* Order peaks from large to small */
  sort_indices_descend( pn_x, pn_locs, *pn_npks );

  for ( i = -1; i < *pn_npks; i++ )
  {
    n_old_npks = *pn_npks;
    *pn_npks = i + 1;

    for ( j = i + 1; j < n_old_npks; j++ )
    {
      n_dist =  pn_locs[j] - ( i == -1 ? -1 : pn_locs[i] ); // lag-zero peak of autocorr is at index -1

      if ( n_dist > n_min_distance || n_dist < -n_min_distance )
      {
        pn_locs[(*pn_npks)++] = pn_locs[j];
      }

    }

  }
  // Resort indices int32_to ascending order
  sort_ascend( pn_locs, *pn_npks );
}

void spo2_algorithm :: sort_ascend(int32_t  *pn_x, int32_t n_size)
/**
  \brief        Sort array
  \par          Details
                Sort array in ascending order (insertion sort algorithm)

  \retval       None
*/
{
  int32_t i, j, n_temp;

  for (i = 1; i < n_size; i++)
  {
    n_temp = pn_x[i];

    for (j = i; j > 0 && n_temp < pn_x[j - 1]; j--)
    {
      pn_x[j] = pn_x[j - 1];
    }
    pn_x[j] = n_temp;

  }

}

void spo2_algorithm :: sort_indices_descend(  int32_t  *pn_x, int32_t *pn_indx, int32_t n_size)
/**
  \brief        Sort indices
  \par          Details
                Sort indices according to descending order (insertion sort algorithm)

  \retval       None
*/
{
  int32_t i, j, n_temp;

  for (i = 1; i < n_size; i++)
  {
    n_temp = pn_indx[i];

    for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j - 1]]; j--)
    {
      pn_indx[j] = pn_indx[j - 1];
    }
    pn_indx[j] = n_temp;

  }

}

/***
 * Algorithm to calculate O2 sats %, Heart rate and Resp rate from Oximeter waveform
 * based on Protocentral library
 * and Maxim app note at  https://www.maximintegrated.com/en/design/technical-documents/app-notes/6/6845.html
 * modified by Richard Hosking 2022
 *  
 * spO2 (percentage O2 saturated Haemoglobin)
 * measurement is achieved by the following equation 
 * spO2 = aR^2 + bR + c
 * which is an approximate quadratic function with coefficients a,b,c derived experimentally
 * float_SPO2 =  -45.060*n_ratio_average* n_ratio_average/10000 + 30.354 *n_ratio_average/100 + 94.845 
 * 
 * Alternatively a lookup table can be used to derive spO2 from R
 * 
 * Intermediate variable R
 * R =  (ACred/DCred)/(ACir/DCir)
 * where
 * ACred is the AC amplitude of the red waveform signal
 * DCred is the DC offset of the red signal 
 * ACir is the AC amplitude of the ir (Infrared) signal 
 * DCir is the DC offset of the ir signal  
 */ 

#ifndef myoximeter_algorithm_h
#define myoximeter_algorithm_h

#define SF_spo2          25    //sampling frequency
#define BUFFER_LENGTH  255
#define MA4_SIZE         4     // 4 point moving average
#define min(x,y) ((x) < (y) ? (x) : (y)) // If x<y return x else return y

class spo2_algorithm
{
  public:
    spo2_algorithm();
	void estimate_spo2(uint16_t *pun_ir_buffer, uint16_t *pun_red_buffer, afe44xx_internal_data *internal_data);    
   
    // My peak finding routine
    int my_find_peaks(uint16_t *AC_data_buffer, int16_t *peaks_buffer, afe44xx_internal_data *internal_data);
    
  private:
    // IR buffer for intermediate calculations - will hold buffer - DC offset 
    uint16_t an_x[BUFFER_LENGTH];
    // Red buffer for intermediate calculations
    uint16_t an_y[BUFFER_LENGTH];
    
    // Lookup table for O2 sats %
    // See quadratic function above 
    const uint8_t uch_spo2_table[184]={ 95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
                                    99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
                                   100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
                                    97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
                                    90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
                                    80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
                                    66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
                                    49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
                                    28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5,
                                    3,   2,  1  };
                                    
    // Variables from main function 
    uint16_t sample_max, sample_min, threshold; 
    // Locations of valleys
    int16_t an_ir_valley_locs[15];
    // For HR calculation 
    int16_t n_peak_interval_sum;
    float av_peak_interval;
    int16_t n_npks;
    
    // For Sats calculation
    int local_minIR, local_maxIR, local_minRED, local_maxRED;
    int AC_IR, AC_RED, DC_IR, DC_RED;
    
    // intermediate variable for lookup table 
    float R = 0.0;
    int lookup;
    uint16_t n_spo2_calc;
    
    int i, j, k; // Loop counters - Care with C++ integer arithmetic
 
};

#endif

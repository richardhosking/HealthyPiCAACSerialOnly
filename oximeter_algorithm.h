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

#ifndef oximeter_algorithm_h
#define oximeter_algorithm_h

#define SF_spo2          25    //sampling frequency
#define BUFFER_SIZE  (SF_spo2*4)
#define MA4_SIZE         4     // 4 point moving average
#define min(x,y) ((x) < (y) ? (x) : (y)) // If x<y return x else return y

class spo2_algorithm
{
  public:
    spo2_algorithm();
	void estimate_spo2(uint16_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint16_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate, int8_t *pch_hr_valid);
    void find_peak( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num );
    void find_peak_above( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height);
    void remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance);
    void sort_ascend(int32_t  *pn_x, int32_t n_size);
    void sort_indices_descend(  int32_t  *pn_x, int32_t *pn_indx, int32_t n_size);
    
  private:
    // Red buffer
    // static variable available to all objects of this class 
    // one object only so seems irrelevant  
    static  int32_t an_x[ BUFFER_SIZE];
    // IR buffer
    static  int32_t an_y[ BUFFER_SIZE];
    // Lookup table 
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
    uint32_t DC_ir_mean,un_only_once;
    int32_t k, n_i_ratio_count;
    int32_t i, s, m, n_exact_ir_valley_locs_count, n_middle_idx;
    int32_t n_th1, n_npks, n_c_min;
    int32_t an_ir_valley_locs[15];
    int32_t n_peak_interval_sum;
    int32_t n_y_ac, n_x_ac;
    int32_t n_spo2_calc;
    int32_t n_y_dc_max, n_x_dc_max;
    int32_t n_y_dc_max_idx, n_x_dc_max_idx;
    int32_t an_ratio[5], n_ratio_average;
    int32_t n_nume, n_denom;
  
};

#endif

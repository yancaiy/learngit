#include <string.h>
#include "math.h"
#include "calterah_math.h"

#include "baseband.h"
#include "sensor_config.h"
#include "baseband_n2d.h"

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#ifndef M_PI
#define M_PI 3.1415926535f
#endif
#define RAD2DEG 57.295779513082323
/* DEG2RAD = pi/180 */
#define DEG2RAD 0.017453292519943295f

n2d_info_t n2d_info;

void cal_coe(complex_t *sv0, complex_t *sv1, complex_t *X, int elem_num, complex_t *coe_array)
{
        complex_t sv0X = dot_product(sv0, X, elem_num);
        complex_t sv1X = dot_product(sv1, X, elem_num);

        complex_t sHX[2] = {sv0X, sv1X};

        complex_t sv0sv1 = dot_product(sv0, sv1, elem_num);
        complex_t sv0sv1_conj = {.r = sv0sv1.r, .i = -1 * sv0sv1.i};

        complex_t c_elem_num = {.r = 1.0 * elem_num, .i = 0.0};

        complex_t minus_sv0sv1;
        crmult(&sv0sv1, -1.0, &minus_sv0sv1);
        complex_t minus_sv0sv1_conj;
        crmult(&sv0sv1_conj, -1.0, &minus_sv0sv1_conj);

        complex_t c0[2] = {c_elem_num, cconj(&minus_sv0sv1)};
        complex_t c1[2] = {cconj(&minus_sv0sv1_conj), c_elem_num};

        coe_array[0] = dot_product(c0, sHX, 2);
        coe_array[1] = dot_product(c1, sHX, 2);
}

bool is_sw_combined_mode(sensor_config_t* cfg,
                         doa_info_t *doa_info) {
        if (cfg->doa_mode != 1 || cfg->doa_max_obj_per_bin[0] == 1) {
                /* if is not configured as combined mode + multiple objects, return false to follow old procedure */
                return false;
        }

        if (!(doa_info->ang_vld_0 && doa_info->ang_vld_1)) {
                /* if DoA detects only one theta angle, use old procedure to handle */
                return false;
        }

        return true;
}

#if (RNE_ENA == 1)
bool is_sw_normal_mode(sensor_config_t* cfg,
                         doa_info_t *doa_info) {
        if (cfg->doa_mode != 0 || cfg->doa_max_obj_per_bin[0] == 1) {
                //if is not configured as normal mode + multiple objects, return false to follow old procedure 
                return false;
        }
        if (!(doa_info->ang_vld_0 && doa_info->ang_vld_1)) {
                // if DoA detects only one theta angle, use old procedure to handle 
                return false;
        }
#if TRK_CONF_3D
        //rne normal mode without ele doa, should take care of 
        return false;
#endif
        return true;
}
/* RNE function add end */
#endif

void sw_combined_mode(radar_sys_params_t* sys_params, sensor_config_t* cfg,
                      doa_info_t *doa_info,
                      volatile track_cdi_t *track_cdi,
                      int *j, int32_t rng_idx, int32_t vel_idx,
                      float vel_acc) {

        uint8_t azi_line_num = n2d_info.azi_line_num;

        uint32_t ang0_idx = doa_info->ang_idx_0 + ((cfg->doa_method == 2 && doa_info->ang_acc_0 != 0) ? 512 : 0);
        uint32_t ang1_idx = doa_info->ang_idx_1 + ((cfg->doa_method == 2 && doa_info->ang_acc_1 != 0) ? 512 : 0);

        complex_t x_total[32] = {0};
        baseband_hw_t * bb_hw = &((baseband_t *)cfg->bb)->bb_hw;

        uint8_t bpm_idx_min = 0;
        uint8_t bpm_idx_max = cfg->nvarray - 1;
        if (cfg->anti_velamb_en) {
                bpm_idx_min = 1;
                bpm_idx_max = cfg->nvarray;
        }
	
        uint32_t old1 = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_BUF);
        for (int bpm_index = bpm_idx_min; bpm_index <= bpm_idx_max; bpm_index++) {
                complex_t tw = {.r = 1, .i = 0};
                if (n2d_info.ph_comp_flg && bpm_index != bpm_idx_min) {
                        tw = expj(n2d_info.Tr_2_PI_Over_TDN * (bpm_index - bpm_idx_min) * vel_acc);
                }

                for (uint8_t ch_index = 0; ch_index < MAX_NUM_RX; ch_index++) {
                        uint32_t fft_mem = baseband_hw_get_fft_mem(bb_hw,
                                                                   ch_index,
                                                                   rng_idx, vel_idx,
                                                                   bpm_index);
                        complex_t complex_fft = cfl_to_complex(fft_mem, 14, 1, true, 4, false);

                        if (n2d_info.ph_comp_flg && (bpm_index != bpm_idx_min)) {
                                cmult(&complex_fft, &tw, &complex_fft);
                        }

                        x_total[ch_index + (bpm_index - bpm_idx_min) * MAX_NUM_RX] = complex_fft;
                }
        }
        baseband_switch_mem_access(bb_hw, old1);

        complex_t Y[2][4];

        float phi[2] = {0.0, 0.0};
        int y_elem_idx = 0;
        float factor_sum = 0;

        int last_line_idx = 0;

        for (int line_idx = 0; line_idx < azi_line_num; line_idx++) {
                int elem_num = n2d_info.azi_vector_size[line_idx];
                if (elem_num >= 3) {
                        last_line_idx = line_idx;
                        break;
                }
        }

        for (int line_idx = 0; line_idx < azi_line_num; line_idx++) {
                int elem_num = n2d_info.azi_vector_size[line_idx];

                complex_t x[32];
                complex_t sv0[32];
                complex_t sv1[32];

                for (int idx = 0; idx < elem_num; idx++) {
                        x[idx] = x_total[n2d_info.azi_logic_channel_idx[line_idx][idx]];
                }

                read_steering_vec_from_mem(bb_hw,
                                           elem_num,
                                           n2d_info.azi_sv_add[line_idx],
                                           ang0_idx, sv0);

                read_steering_vec_from_mem(bb_hw,
                                           elem_num,
                                           n2d_info.azi_sv_add[line_idx],
                                           ang1_idx, sv1);

                complex_t coe_array[2];
                cal_coe(sv0, sv1, x, elem_num, coe_array);

                Y[0][y_elem_idx] = coe_array[0];
                Y[1][y_elem_idx] = coe_array[1];

                if (y_elem_idx > 0) {
						factor_sum += 1 ;
                        complex_t y1y0;
                        float diff   = n2d_info.vertical_dis[line_idx] - n2d_info.vertical_dis[last_line_idx];
                        cmult_conj(&(Y[0][y_elem_idx]), &(Y[0][y_elem_idx - 1]), &y1y0);
						if ( diff >=0.01 || diff <= -0.01 ){
		                        phi[0] +=   asinf(atan2f(y1y0.i, y1y0.r) / (2 * M_PI * diff));							

                        cmult_conj(&(Y[1][y_elem_idx]), &(Y[1][y_elem_idx - 1]), &y1y0);
		                        phi[1] +=  asinf(atan2f(y1y0.i, y1y0.r) / (2 * M_PI * diff));
						}else{
								phi[0]=0;
								phi[1] =0;						
						}				
                }

                last_line_idx = line_idx;
                y_elem_idx++;
        }
		if (factor_sum ){
	        phi[0] /= factor_sum;
	        phi[1] /= factor_sum;
		}
		
		
		
        float phi0_rad = phi[0];
        float phi1_rad = phi[1];

        phi[0] *= RAD2DEG;
        phi[1] *= RAD2DEG;

        float sin_theta_0;
        float sin_theta_1;
#if (RNE_ENA == 1)
        /*power1 power2*/
        float cfar_noise = 10*log10f(track_cdi[*j].raw_z.noi);
#endif
        float sin_alpha_0;
        float sin_alpha_1;
		float Pow1_c;
		float Pow2_c;
        if (cfg->doa_method == 2) {
                sin_alpha_0 = sys_params->dml_sin_az_left + ang0_idx * sys_params->dml_sin_az_step;
                sin_alpha_1 = sys_params->dml_sin_az_left + ang1_idx * sys_params->dml_sin_az_step;
        } else {
                float dbf_alpha_0 = sys_params->bfm_az_left + ang0_idx * (sys_params->bfm_az_right - sys_params->bfm_az_left) / sys_params->doa_npoint[0];
                float dbf_alpha_1 = sys_params->bfm_az_left + ang1_idx * (sys_params->bfm_az_right - sys_params->bfm_az_left) / sys_params->doa_npoint[0];
                sin_alpha_0 = sinf(dbf_alpha_0*DEG2RAD);
                sin_alpha_1 = sinf(dbf_alpha_1*DEG2RAD);
        }

		int main_group_idx = 0;
		Pow1_c = Y[0][main_group_idx].r*Y[0][main_group_idx].r+Y[0][main_group_idx].i*Y[0][main_group_idx].i;
		Pow2_c = Y[1][main_group_idx].r*Y[1][main_group_idx].r+Y[1][main_group_idx].i*Y[1][main_group_idx].i;
#if (RNE_ENA == 1)
		float pow1_c_db = 10*log10f(Pow1_c);
		float pow2_c_db = 10*log10f(Pow2_c);
		float theta2,pow_temp;
#endif
        float theta0, theta1, ele0, ele1;
        sin_theta_0 = sin_alpha_0 / cosf(phi0_rad);
        sin_theta_1 = sin_alpha_1 / cosf(phi1_rad);

        if (sin_theta_0 <= 1 && sin_theta_0 >= -1) {
                theta0 = asinf(sin_theta_0) * RAD2DEG;
        } else {
                theta0 = asinf(sin_alpha_0) * RAD2DEG;
        }

        if (sin_theta_1 <= 1 && sin_theta_1 >= -1) {
                theta1 = asinf(sin_theta_1) * RAD2DEG;
        } else {
                theta1 = asinf(sin_alpha_1) * RAD2DEG;
        }
		
#if (RNE_ENA == 1)
	if (pow1_c_db > pow2_c_db) {
			ele0 =phi[0];
			ele1 =phi[1];
        } else {
			pow_temp = pow1_c_db;
			pow1_c_db = pow2_c_db;
			pow2_c_db = pow_temp;
			
			
			ele0 =phi[1];
		
			ele1 =phi[0];
			
			pow_temp = Pow1_c;
			Pow1_c= Pow2_c;
			Pow2_c = pow_temp;
			theta2 = theta0;
			theta0 = theta1;
			theta1 = theta2;		
        }
#else
			ele0 = phi[0];	
			ele1 = phi[1];
#endif
	#if TRK_CONF_3D
				ele0 = phi[0];	
				ele1 = phi[1];
		#else
				ele0 = 0;	
				ele1 = 0;
		#endif		
		
        bool theta0_exist = false;
#if (RNE_ENA == 1)
        bool theta1_exist = false;

        if(cfg->doa_max_obj_per_bin[0] == 0) {
                if( (pow1_c_db - pow2_c_db < (cfg-> SNR_Diff_Th)) && (pow2_c_db - cfar_noise > (cfg->Min_SNR_Th)) ){
                        theta0_exist = true;
                }
        }else if(cfg->doa_max_obj_per_bin[0] == 2){
                theta0_exist = true;
        }
#endif
		   
        if (   theta0 > sys_params->trk_fov_az_left && theta0 < sys_params->trk_fov_az_right
			   && ele0 > sys_params->trk_fov_ev_down && ele0 < sys_params->trk_fov_ev_up) {
                track_cdi[*j].raw_z.ang = theta0;
				   if (cfg->track_obj_snr_sel == 0) {						   
							   track_cdi[*j].raw_z.sig = Pow1_c;							   
					}
					else{							   
		                        uint32_t hw_sig = doa_info->sig_1;
		                        track_cdi[*j].raw_z.sig = fl_to_float(hw_sig, 15, 1, false, 5, false);
						   	   
					}
		 #if TRK_CONF_3D
						   track_cdi[*j].raw_z.ang_elv = ele0;
		 #endif

		 		
				   *j += 1;	
#if (RNE_ENA == 0)
				   theta0_exist = true;
#endif
		   }
#if (RNE_ENA == 1)
		   else{
			   theta1_exist = true;
			   theta0_exist = false;
		   }
#endif

#if (RNE_ENA == 1)
        if  (theta1_exist || theta0_exist){
#endif
		   /*  theta0 and ele0 must satisfy track fov range */
		   if (   theta1 > sys_params->trk_fov_az_left && theta1 < sys_params->trk_fov_az_right
			   && ele1 > sys_params->trk_fov_ev_down && ele1 < sys_params->trk_fov_ev_up) {

				   if(theta0_exist)
					   {
					 
					   track_cdi[*j].raw_z = track_cdi[*j - 1].raw_z;
					   track_cdi[*j].raw_z.ang = theta1;
							 
					   	#if TRK_CONF_3D
							   track_cdi[*j].raw_z.ang_elv = ele1;
			 			#endif							   
					   }
				   else 
					   {
					   
					   track_cdi[*j].raw_z.ang = theta1;
							   
			 			#if TRK_CONF_3D
					    		track_cdi[*j].raw_z.ang_elv = ele1;
			 			#endif
				       }
				   if (cfg->track_obj_snr_sel == 0) {						   
					   	track_cdi[*j].raw_z.sig = Pow2_c;							   
					}
				   else{
                        uint32_t hw_sig = doa_info->sig_1;
                        track_cdi[*j].raw_z.sig = fl_to_float(hw_sig, 15, 1, false, 5, false);
							   	   
				   }
				   *j += 1;
		  }  
#if (RNE_ENA == 1)
        }
#endif
}

#if (RNE_ENA == 1)
  /* doa normal mode, only azi doa, no ele doa */
  void sw_normal_mode(radar_sys_params_t* sys_params, sensor_config_t* cfg,
						doa_info_t *doa_info,
						volatile track_cdi_t *track_cdi,
						int *j, int32_t rng_idx, int32_t vel_idx,
						float vel_acc) {

  
		  uint32_t ang0_idx = doa_info->ang_idx_0 + ((cfg->doa_method == 2 && doa_info->ang_acc_0 != 0) ? 512 : 0);
		  uint32_t ang1_idx = doa_info->ang_idx_1 + ((cfg->doa_method == 2 && doa_info->ang_acc_1 != 0) ? 512 : 0);
  
		  complex_t x_total[32];
		  baseband_hw_t * bb_hw = &((baseband_t *)cfg->bb)->bb_hw;
  
		  uint8_t bpm_idx_min = 0;
		  uint8_t bpm_idx_max = cfg->nvarray - 1;
		 
		  float phi[2] = {0.0, 0.0};
        
		  float theta0, theta1,pow_temp,Pow1_c,Pow2_c;
		  float sin_alpha_0;
		  float sin_alpha_1;
		  float pow1_c_db, pow2_c_db, cfar_noise;
		  float dbf_alpha_0, dbf_alpha_1;
		  complex_t coe_array[2];
  
		  complex_t x[32];
		  complex_t sv0[32];
		  complex_t sv1[32];

		  if (cfg->anti_velamb_en) {
				  bpm_idx_min = 1;
				  bpm_idx_max = cfg->nvarray;
		  }
  
		  // should check steering vector address is right or not
		  uint32_t old1 = baseband_switch_mem_access(bb_hw, SYS_MEM_ACT_BUF);
		  for (int bpm_index = bpm_idx_min; bpm_index <= bpm_idx_max; bpm_index++) {
				  complex_t tw = {.r = 1, .i = 0};
				  if (n2d_info.ph_comp_flg && bpm_index != bpm_idx_min) {
						  tw = expj(n2d_info.Tr_2_PI_Over_TDN * (bpm_index - bpm_idx_min) * vel_acc);
				  }
  
				  for (uint8_t ch_index = 0; ch_index < MAX_NUM_RX; ch_index++) {
						  uint32_t fft_mem = baseband_hw_get_fft_mem(bb_hw,
																	 ch_index,
																	 rng_idx, vel_idx,
																	 bpm_index);
						  complex_t complex_fft = cfl_to_complex(fft_mem, 14, 1, true, 4, false);
  
						  if (n2d_info.ph_comp_flg && (bpm_index != bpm_idx_min)) {
								  cmult(&complex_fft, &tw, &complex_fft);
						  }
  
						  x_total[ch_index + (bpm_index - bpm_idx_min) * MAX_NUM_RX] = complex_fft;
				  }
		  }
		  baseband_switch_mem_access(bb_hw, old1);
  
  
		  for (int line_idx = 0; line_idx < 1; line_idx++) {
				  
				  int elem_num = n2d_info.azi_vector_size[line_idx];
  
				  for (int idx = 0; idx < elem_num; idx++) {
						  x[idx] = x_total[n2d_info.azi_logic_channel_idx[line_idx][idx]];
				  }
  
				 
				  read_steering_vec_from_mem(bb_hw,
											 elem_num,
											 n2d_info.azi_sv_add[line_idx],
											 ang0_idx, sv0);
  
				  read_steering_vec_from_mem(bb_hw,
											 elem_num,
											 n2d_info.azi_sv_add[line_idx],
											 ang1_idx, sv1);
  
  
				   cal_coe(sv0, sv1, x, elem_num, coe_array);

  
  
		  }
  
  
		  Pow1_c = coe_array[0].r*coe_array[0].r+coe_array[0].i*coe_array[0].i;
		  Pow2_c = coe_array[1].r*coe_array[1].r+coe_array[1].i*coe_array[1].i;
		  pow1_c_db = 10*log10f(Pow1_c);
		  pow2_c_db = 10*log10f(Pow2_c);
		  
		  float ele0, ele1;
		  

		  cfar_noise = 10*log10f(track_cdi[*j].raw_z.noi);
  
  
  
		   if (cfg->doa_method == 2) {
                sin_alpha_0 = sys_params->dml_sin_az_left + ang0_idx * sys_params->dml_sin_az_step;
                sin_alpha_1 = sys_params->dml_sin_az_left + ang1_idx * sys_params->dml_sin_az_step;
        } else {
                dbf_alpha_0 = sys_params->bfm_az_left + ang0_idx * (sys_params->bfm_az_right - sys_params->bfm_az_left) / sys_params->doa_npoint[0];
                dbf_alpha_1 = sys_params->bfm_az_left + ang1_idx * (sys_params->bfm_az_right - sys_params->bfm_az_left) / sys_params->doa_npoint[0];
                sin_alpha_0 = sinf(dbf_alpha_0*DEG2RAD);
                sin_alpha_1 = sinf(dbf_alpha_1*DEG2RAD);
        }

		/* must DML method */
		  if (pow1_c_db > pow2_c_db)
		  {
			  theta0=asinf(sin_alpha_0) * RAD2DEG;
		  	  theta1=asinf(sin_alpha_1) * RAD2DEG;
			  ele0 = 0;
			  ele1 = 0;
	
		  }
		  else
		  {
			  pow_temp = pow1_c_db;
			  pow1_c_db = pow2_c_db;
			  pow2_c_db=pow_temp;		  
			  theta0=asinf(sin_alpha_1) * RAD2DEG;	  
			  theta1=asinf(sin_alpha_0) * RAD2DEG;
			  ele0 = 0;
			  ele1 = 0;
		  }

		   // RNE of secondary
		   bool theta0_exist = false;
		   bool theta1_exist = false;
		   if(cfg->doa_max_obj_per_bin[0] == 0) {  				 
			   
					   if( (pow1_c_db - pow2_c_db < (cfg-> SNR_Diff_Th)) && (pow2_c_db - cfar_noise > (cfg->Min_SNR_Th)) ){
						   theta0_exist = true; 						   
						   
					   }
					   
					   
		   }else if(cfg->doa_max_obj_per_bin[0] == 2){
				   theta0_exist = true;

		   }

		   // theta0 and ele0 must satisfy track fov range 
		   if (   theta0 > sys_params->trk_fov_az_left && theta0 < sys_params->trk_fov_az_right){
			   
				   track_cdi[*j].raw_z.ang = theta0;				  
				 
				   if (cfg->track_obj_snr_sel == 0) {

						   track_cdi[*j].raw_z.sig = Pow1_c;
							  
				  }
				  else{
							uint32_t hw_sig = doa_info->sig_1;
							track_cdi[*j].raw_z.sig = fl_to_float(hw_sig, 15, 1, false, 5, false);

							   		   
				 }
		 #if TRK_CONF_3D
						   track_cdi[*j].raw_z.ang_elv = ele0;
		
		 #endif

				   *j += 1;	   
		   }else{		
				   theta1_exist = true;
				   theta0_exist = false ;				   
		   }
					   
		  
		  
			if  (theta1_exist || theta0_exist){
		   // theta0 and ele0 must satisfy track fov range 
				   if (   theta1 > sys_params->trk_fov_az_left && theta1 < sys_params->trk_fov_az_right) {

						   if(theta0_exist)
							   {
							   track_cdi[*j].raw_z = track_cdi[*j - 1].raw_z;
							   track_cdi[*j].raw_z.ang = theta1;
							 
							   	#if TRK_CONF_3D
									   track_cdi[*j].raw_z.ang_elv = ele1;
					 			#endif

							   }
						   else 
							   {
							   track_cdi[*j].raw_z.ang = theta1;
							   
					 #if TRK_CONF_3D
							    track_cdi[*j].raw_z.ang_elv = ele1;
					 #endif
				  
						   }
						   
						   if (cfg->track_obj_snr_sel == 0) {

							   track_cdi[*j].raw_z.sig = Pow2_c;							  
							}
						   else{							   
		                        uint32_t hw_sig = doa_info->sig_1;
		                        track_cdi[*j].raw_z.sig = fl_to_float(hw_sig, 15, 1, false, 5, false);

							   		   
						   }
						   *j += 1;
				  }   
			}
}
#endif

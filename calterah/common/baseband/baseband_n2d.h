#ifndef BASEBAND_N2D_H
#define BASEBAND_N2D_H

typedef struct N2D_INFO {
        uint8_t azi_line_num;
        uint8_t azi_vector_size[4];
        uint8_t azi_logic_channel_idx[4][32];
        uint32_t azi_sv_add[4];
        float   vertical_dis[4];

        /* fft2d data in fft-memory needs to be
           phase compensated when velamb is enabled */
        bool  ph_comp_flg;
        float Tr_2_PI_Over_TDN;
} n2d_info_t;
extern n2d_info_t n2d_info;

void sw_combined_mode(radar_sys_params_t* sys_params, sensor_config_t* cfg,
                      doa_info_t *doa_info,
                      volatile track_cdi_t *track_cdi,
                      int *j, int32_t rng_idx, int32_t vel_idx,
                      float vel_acc);

bool is_sw_combined_mode(sensor_config_t* cfg,
                         doa_info_t *doa_info);

#if (RNE_ENA == 1)
void sw_normal_mode(radar_sys_params_t* sys_params, sensor_config_t* cfg,
                      doa_info_t *doa_info,
                      volatile track_cdi_t *track_cdi,
                      int *j, int32_t rng_idx, int32_t vel_idx,
                      float vel_acc);

bool is_sw_normal_mode(sensor_config_t* cfg,
                         doa_info_t *doa_info);
#endif

#endif

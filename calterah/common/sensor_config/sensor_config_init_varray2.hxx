
/*
 This file is automatically generated by sesnor_code_gen.py. Please do not check in any manual changes!
 */
        .fmcw_startfreq                = 78.5,
        .fmcw_bandwidth                = 400,
        .fmcw_chirp_rampup             = 20,
        .fmcw_chirp_down               = 3,
        .fmcw_chirp_period             = 28,
        .nchirp                        = 256,
        .adc_freq                      = 40,
        .dec_factor                    = 1,
        .adc_sample_start              = 2,
        .adc_sample_end                = 18,
        .tx_groups                     = {0x0001, 0x0000, 0x0000, 0x0010},
        .rng_win                       = "cheb",
        .vel_win                       = "cheb",
        .rng_win_params                = {80,0, 0},
        .vel_win_params                = {80,0,0},
        .rng_nfft                      = 512,
        .vel_nfft                      = 128,
        .rng_fft_scalar                = 255,
        .vel_fft_scalar                = 63,
        .fft_nve_bypass                = false,
        .fft_nve_shift                 = 0,
        .fft_nve_ch_mask               = 65535,
        .fft_nve_default_value         = 0.0001,
        .fft_nve_without_interference  = 0.0001,
        .fft_nve_threash               = 50,
        .fft_nve_rangeindex_start      = 50,
        .fft_nve_rangeindex_end        = 70,
        .cfar_pk_en                    = 0xff,
        .cfar_pk_win_size1             = {3,3,3,3,3,3,3,3},
        .cfar_pk_win_size2             = {3,3,3,3,3,3,3,3},
        .cfar_pk_threshold             = {31,31,31,31,31,31,31,31},
        .cfar_sliding_win              = 0,
        .cfar_recwin_decimate          = 0,
        .cfar_recwin_msk               = {0x7FF,0x401,0x401,0x401,0x401,0x401,0x401,0x401,0x401,0x401,0x7FF,0x7FF,0x7FF,0x7FF,0x7FF,0x7FF,0x7FF,0x7FF,0x7FF,0x7FF,0x7FF,0x7FF,0x3FE,0x202,0x202,0x202,0x202,0x202,0x202,0x202,0x202,0x202,0x3FE,0x3FE,0x3FE,0x3FE,0x3FE,0x3FE,0x3FE,0x3FE,0x3FE,0x3FE,0x3FE,0x3FE,0x1FC,0x104,0x104,0x104,0x104,0x104,0x104,0x104,0x104,0x104,0x104,0x1FC,0x1FC,0x1FC,0x1FC,0x1FC,0x1FC,0x1FC,0x1FC,0x1FC,0x1FC,0x1FC,0x7FF,0x202,0x202,0x202,0x202,0x202,0x202,0x202,0x202,0x202,0x7FF,0x000,0x3FE,0x202,0x202,0x202,0x202,0x202,0x202,0x202,0x3FE,0x000},
        .cfar_region_algo_type         = 0,
        .cfar_os_rnk_ratio             = {0.4444,0.4444,0.4444,0.4444,0.4444,0.4444,0.4444,0.4444},
        .cfar_os_rnk_sel               = {0,0,0,0,0,0,0,0},
        .cfar_os_tdec                  = {4,4,4,4,4,4,4,4},
        .cfar_os_alpha                 = {4.22,4.22,4.22,4.22,4.22,4.22,4.22,4.22},
        .cfar_combine_dirs             = 0,
        .cfar_combine_thetas           = {0, 5, -5, 10, -10, 15, -15, 20, -20, 25, -25, 40, -40, 0, 0, 0},
        .cfar_combine_phis             = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        .cfar_crswin_rng_size          = 5,
        .cfar_crswin_rng_skip          = 1,
        .cfar_crswin_vel_size          = 5,
        .cfar_crswin_vel_skip          = 1,
        .cfar_mimo_win                 = "cheb",
        .cfar_mimo_win_params          = {30, 0, 0},
        .cfar_noise_type               = 0xff,
        .cfar_nr_alpha                 = {10,10,10,10,10,10,10,10},
        .cfar_nr_beta1                 = {0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25},
        .cfar_nr_beta2                 = {4,4,4,4,4,4,4,4},
        .cfar_nr_rnk_ratio             = {0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25},
        .cfar_nr_rnk_sel               = {0,0,0,0,0,0,0,0},
        .cfar_nr_scheme_sel            = {0,0,0,0,0,0,0,0},
        .cfar_nr_tdec                  = {4,4,4,4,4,4,4,4},
        .cfar_region_sep_rng           = {1023, 1023, 1023},
        .cfar_region_sep_vel           = {1023, 1023, 1023,1023, 1023, 1023,1023, 1023},
        .cfar_sogo_alpha               = {15,15,15,15,15,15,15,15},
        .cfar_sogo_i                   = {0,0,0,0,0,0,0,0},
        .cfar_sogo_mask                = {0xF,0xF,0xF,0xF,0xF,0xF,0xF,0xF},
        .cfar_ca_alpha                 = {25,25,25,25,25,25,25,25},
        .cfar_ca_n                     = {2,2,2,2,2,2,2,2},
        .doa_mode                      = 0,
        .doa_num_groups                = 1,
        .doa_fft_mux                   = {0x000000FF, 0x00000124, 0x00008421},
        .combined_doa_fft_mux          = {0x0000000F, 0x000000F0, 0x00000F00, 0x0000F000},
        .doa_method                    = 0,
        .doa_npoint                    = {360,360,360},
        .doa_samp_space                = 't',
        .doa_max_obj_per_bin           = {1,1,1},
        .bfm_peak_scalar               = {0.1, 0.1, 0.1},
        .bfm_noise_level_scalar        = {30, 30, 30},
        .bfm_snr_thres                 = {15,30},
        .bfm_az_left                   = -60,
        .bfm_az_right                  = 60,
        .bfm_ev_up                     = 10,
        .bfm_ev_down                   = -10,
        .doa_win                       = "cheb",
        .doa_win_params                = {30, 0, 0},
        .bfm_raw_search_step           = 1,
        .bfm_fine_search_range         = 20,
        .bfm_iter_search               = false,
        .bfm_mode                      = 0,
        .bfm_group_idx                 = 0,
        .ant_info_from_flash           = true,
        .ant_info_flash_addr           = 0x10000,
        .ant_pos                       = {{0, 0}, {.5, 0}, {1, 0}, {1.5, 0}, {2, 0}, {2.5, 0}, {3.0, 0}, {3.5, 0}, {4, 0}, {4.5, 0}, {5, 0}, {5.5, 0}, {6, 0}, {6.5, 0}, {7.0, 0}, {7.5, 0}},
        .ant_comps                     = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        .bpm_mode                      = false,
        .phase_scramble_on             = 0,
        .phase_scramble_init_state     = 0xdeadbeaf,
        .phase_scramble_tap            = 0xc0000401,
        .phase_scramble_comp           = {180,180,180,180},
        .phase_scramble_comp_amplitude = {1,1,1,1},
        .freq_hopping_on               = 0,
        .freq_hopping_init_state       = 0xbabedead,
        .freq_hopping_tap              = 0xc0000401,
        .freq_hopping_deltaf           = 6,
        .chirp_shifting_on             = 0,
        .chirp_shifting_init_state     = 0x87654321,
        .chirp_shifting_init_tap       = 0xc0000401,
        .chirp_shifting_delay          = 1,
        .fsm_on                        = false,
        .agc_mode                      = 0,
        .agc_code                      = {0x125, 0x91, 0x83, 0x3, 0x88, 0x81, 0x11, 0x81, 0x81, 0x3, 0x1, 0x1, 0x3},
        .agc_tia_thres                 = 0.5,
        .agc_vga1_thres                = 0.8,
        .agc_vga2_thres                = 0.8,
        .agc_align_en                  = false,
        .adc_comp_en                   = false,
        .rf_tia_gain                   = 2,
        .rf_vga1_gain                  = 5,
        .rf_vga2_gain                  = 3,
        .rf_hpf1                       = 2,
        .rf_hpf2                       = 1,
        .de_vel_amb                    = false,
        .track_fps                     = 20,
        .track_fov_az_left             = -60,
        .track_fov_az_right            = 60,
        .track_fov_ev_down             = -8,
        .track_fov_ev_up               = 8,
        .track_near_field_thres        = 10,
        .track_capture_delay           = 0.15,
        .track_drop_delay              = 0.5,
        .track_vel_pos_ind_portion     = 0.5,
        .track_obj_snr_sel             = 0,
        .tx_phase_value                = {45, 45, 45, 45},
        .spk_en                        = false,
        .spk_buf_len                   = 4,
        .spk_set_zero                  = false,
        .spk_ovr_num                   = 0,
        .spk_thres_dbl                 = false,
        .spk_min_max_sel               = false,
        .zero_doppler_cancel           = false,
        .anti_velamb_en                = false,
        .anti_velamb_delay             = 10,
        .anti_velamb_qmin              = -2,
        .high_vel_comp_en              = true,
        .high_vel_comp_method          = 0,
        .vel_comp_usr                  = 0,
        .rng_comp_usr                  = 0,
        .dml_2dsch_start               = {0, 0},
        .dml_2dsch_step                = {16, 8},
        .dml_2dsch_end                 = {359, 359},
        .dml_extra_1d_en               = {true, true},
        .dml_p1p2_en                   = {true, true},
        .dml_respwr_coef               = {0, 0, 0.1, -1, 0, 0, 0, 0.1, -1, 0},
        .acc_rng_hw                    = true,
        .acc_vel_hw                    = true,
        .acc_angle_hw                  = false,
        .cas_obj_merg_typ              = 2,
        .sv_read_from_flash            = false,
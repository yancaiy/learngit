
/*
 This file is automatically generated by sesnor_code_gen.py. Please do not check in any manual changes!
 */
    .fmcw_startfreq = 76.05,
    .fmcw_bandwidth = 650,
    .fmcw_chirp_rampup = 45,
    .fmcw_chirp_down = 2,
    .fmcw_chirp_period = 49,
    .tx_groups = {0x0000, 0x0001, 0x0010, 0x0100},
    .nchirp = 192,
    .adc_freq = 25,
    .dec_factor = 2,
    .adc_sample_start = 3,
    .adc_sample_end = 43,
    .rng_win = "hanning",
    .vel_win = "hanning",
    .rng_win_params = {80, 0, 0},
    .vel_win_params = {80, 0, 0},
    .rng_nfft = 1024,
    .vel_nfft = 64,
    .rng_fft_scalar = 0xff,
    .vel_fft_scalar = 0x3f,
    .fft_nve_bypass = false,
    .fft_nve_shift = 0,
    .fft_nve_ch_mask = 65535,
    .fft_nve_default_value = 0.0001,
    .cfar_pk_en = 0xff,
    .cfar_pk_win_size1 = {3, 3, 3, 3, 3, 3, 3, 3},
    .cfar_pk_win_size2 = {3, 3, 3, 3, 3, 3, 3, 3},
    .cfar_pk_threshold = {31, 31, 31, 31, 31, 31, 31, 31},
    .cfar_recwin_decimate = 0,
    .cfar_recwin_msk = {0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF,
                        0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF,
                        0x707, 0x707, 0x707, 0x707, 0x707, 0x707, 0x707, 0x707, 0x707, 0x707, 0x707,
                        0x707, 0x707, 0x707, 0x707, 0x707, 0x707, 0x707, 0x707, 0x707, 0x707, 0x707,
                        0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF,
                        0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF,
                        0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF,
                        0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF, 0x7FF},
    .cfar_region_algo_type = 0x5555,
    .cfar_os_rnk_ratio = {0.278,  0.3, 0.278, 0.3, 0.4, 0.4, 0.4, 0.4},
    .cfar_os_rnk_sel = {1, 1, 1, 1, 1, 1, 1, 1},
    .cfar_os_tdec = {2, 5, 5, 5, 5, 5, 5, 5},
    .cfar_combine_thetas = {0, 5, -5, 10, -10, 15, -15, 20, -20, 25, -25, 40, -40, 0, 0, 0},
    .cfar_crswin_rng_size = 7,
    .cfar_crswin_rng_skip = 3,
    .cfar_crswin_vel_size = 9,
    .cfar_crswin_vel_skip = 3,
    .cfar_combine_phis = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    .cfar_combine_dirs = 0,
    .cfar_mimo_win = "cheb",
    .cfar_mimo_win_params = {30, 0, 0},
    .cfar_noise_type = 0xff,
    .cfar_nr_alpha = {10, 10, 10, 10, 10, 10, 10, 10},
    .cfar_nr_beta1 = {0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25},
    .cfar_nr_beta2 = {4, 4, 4, 4, 4, 4, 4, 4},
    .cfar_nr_rnk_ratio = {0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25},
    .cfar_nr_rnk_sel = {0, 0, 0, 0, 0, 0, 0, 0},
    .cfar_nr_scheme_sel = {0, 0, 0, 0, 0, 0, 0, 0},
    .cfar_nr_tdec = {4, 4, 4, 4, 4, 4, 4, 4},
    .cfar_os_alpha  =   {80.00, 25.00, 12.0, 25.0, 20.0, 15.0, 16.0, 12.00},
    .cfar_region_sep_rng = {10, 50, 230},
    .cfar_region_sep_vel = {1, 63, 1, 63, 1, 63, 1, 63},
    .cfar_sliding_win = 1,
    .cfar_sogo_alpha = {20, 15, 15, 15, 15, 15, 15, 15},
    .cfar_sogo_i = {0, 0, 0, 0, 0, 0, 0, 0},
    .cfar_sogo_mask = {0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF},
    .cfar_ca_alpha = {40, 20, 15, 15, 15, 15, 15, 15},
    .cfar_ca_n = {2, 2, 2, 2, 2, 2, 2, 2},
    .doa_mode = 1,
    .doa_num_groups = 2,
    .doa_fft_mux = {0x0000DD0, 0x0000248, 0x00008421},
    .combined_doa_fft_mux = {0x0000DD0, 0x00000D, 0x0000220, 0x00000000},
    .doa_method = 0,
    .doa_npoint = {151, 25, 0},
    .doa_samp_space = 't',
    .doa_max_obj_per_bin = {1, 1, 1},
    .bfm_peak_scalar = {0.1, 0.1, 0.1},
    .bfm_noise_level_scalar = {30, 30, 30},
    .bfm_snr_thres = {15, 30},
    .bfm_az_left = -75,
    .bfm_az_right = 76,
    .bfm_ev_up = 13,
    .bfm_ev_down = -12,
    .doa_win = "rect",
    .doa_win_params = {30, 0, 0},
    .bfm_raw_search_step = 1,
    .bfm_fine_search_range = 20,
    .bfm_iter_search = false,
    .bfm_mode = 0,
    .bfm_group_idx = 0,
    .ant_info_from_flash = false,
    .ant_info_flash_addr = 0x10000,
    //.ant_pos = {{3.5, 1.5}, {5, 1.5}, {6, 1.5}, {9, 1.5}, {1.5, 0}, {3, 0}, {4, 0}, {7, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 1.5}, {1.5, 1.5}, {2.5, 1.5}, {5.5, 1.5}},
    //.ant_comps = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0},
    //.ant_pos = {{0,0},{1.51,-0.36},{2.56,-0.16},{5.56,-0.44},{-2.03,-1.35},{-0.51,-1.65},{0.54,-1.47},{3.51,-1.77},{0,0},{0,0},{0,0},{0,0},{-3.6,0.14},{-2.1,-0.18},{-0.96,-0.02},{1.95,-0.29}},
    //.ant_comps = {0,-111.86,-221.99,-137.6,-180.61,-296.51,-46.28,-320.84,0,0,0,0,-62.19,-170.92,-283.2,-197.06},
    .ant_pos = {{0.00,0.00},{-0.50,-0.68},{-1.93,-0.03},{-2.99,0.03},{-1.42,0.67},{-1.93,-0.04},{-3.36,0.62},{-4.42,0.68},{-2.49,-0.16},{-2.99,-0.83},{-4.42,-0.19},{-5.49,-0.13},{-3.93,0.02},{-4.43,-0.66},{-5.86,-0.02},{-6.92,0.04}},
    .ant_comps = {0.00,159.47,222.25,130.78,298.77,95.65,162.21,77.85,193.20,350.57,57.40,330.15,352.32,150.33,214.90,127.79},
    .bpm_mode = false,
    .phase_scramble_on = false,
    .phase_scramble_init_state = 0xdeadbeaf,
    .phase_scramble_tap = 0xc0000401,
    .phase_scramble_comp = {180, 180, 180, 180},
    .freq_hopping_on = false,
    .freq_hopping_init_state = 0xbabedead,
    .freq_hopping_tap = 0xc0000401,
    .freq_hopping_deltaf = 6,
    .chirp_shifting_on = false,
    .chirp_shifting_init_state = 0x87654321,
    .chirp_shifting_init_tap = 0x00000409,
    .chirp_shifting_delay = 1,
    .fsm_on = false,
    .agc_mode = 0,
    .agc_code = {0x125, 0x91, 0x83, 0x3, 0x88, 0x81, 0x11, 0x81, 0x81, 0x3, 0x1, 0x1, 0x3},
    .agc_tia_thres = 0.5,
    .agc_vga1_thres = 0.8,
    .agc_vga2_thres = 0.8,
    .agc_align_en = false,
    .adc_comp_en = false,
    .rf_tia_gain = 2,
    .rf_vga1_gain = 5,
    .rf_vga2_gain = 3,
    .rf_hpf1 = 2,
    .rf_hpf2 = 1,
    .de_vel_amb = false,
    .track_fps = 20,
    .track_fov_az_left = -75,
    .track_fov_az_right = 76,
    .track_fov_ev_down = -12,
    .track_fov_ev_up = 13,
    .track_near_field_thres = 10,
    .track_capture_delay = 0.15,
    .track_drop_delay = 0.5,
    .track_vel_pos_ind_portion = 0.5,
    .tx_phase_value = {45, 45, 45, 45},
    .spk_en = false,
    .spk_buf_len = 4,
    .spk_set_zero = false,
    .spk_ovr_num = 0,
    .spk_thres_dbl = false,
    .spk_min_max_sel = false,
    .zero_doppler_cancel = false,
    .anti_velamb_en = false,
    .anti_velamb_delay = 10,
    .anti_velamb_qmin = -2,
    .dml_2dsch_start = {0, 0},
    .dml_2dsch_step = {16, 8},
    .dml_2dsch_end = {359, 359},
    .dml_extra_1d_en = {true, true},
    .dml_p1p2_en = {true, true},
    .dml_respwr_coef = {0, 0, 0.1, -1, 0, 0, 0, 0.1, -1, 0},
    .acc_rng_hw = true,
    .acc_vel_hw = true,
    .acc_angle_hw = false,
    .sv_read_from_flash = false,

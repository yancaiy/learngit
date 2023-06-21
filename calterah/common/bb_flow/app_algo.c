#include "bb_flow.h"
#include "option_print.h"

#if DDM_EN

/* Description of Doppler Division Multiplexing(DDM) MIMO implementation on ALPS MP:
 *
 * For example, if choose TX2 and TX3 out of TX0~3 on Lyra board to implement DDM MIMO,
 * with imposing no chirp phase shift on TX2 and imposing phase shift sequence [45deg, 135deg, 225deg, 315deg, 45deg,
 * 135deg, 225deg, 315deg, ...] (repetition period is 4) on TX3's chirps,
 * then following STEPs are needed:
 *
 =========================    STEP 1 : configure chirp phase shifting sequence  ========================
 In bb_flow.h, set:

 #define DDM_EN                            true    // Enable DDM MIMO
 #define DDM_CHRP_PHAS_SEQ_PRD             4       // Repetition period of chirp phase shift sequence

 In bb_flow.c:

 static uint8_t ddm_tx_vel_pos[MAX_NUM_TX] = {0, 0, 1, 2};  // Choose phase shift sequence for TX2 and TX3

  =========================    STEP 2 : configure baseband of 2 type of frames  ========================


 * Frame type 0:   // Generate DDM MIMO ADC and FFT2D data

        .nchirp                    = 256,
        .tx_groups                 = {0x0000, 0x0000, 0x0001, 0x0001},  // Choose Lyra TXs to form DDM MIMO
        .rng_nfft                  = 256,
        .vel_nfft                  = 256,
        .doa_npoint                = {36,36,36},  // Squeeze size of frame type 0 steering vectors to save MEM_COE space

 * Frame type 1:  // Use the same configurations as in Frame type 0, except:
 *                // CFAR and DOA run in period-4 TDM mode
 *                // but only choose first 2 MIMO channels, since only 2 TXs works

        .nchirp                    = 256,
        .tx_groups                 = {0x0100, 0x1000, 0x0001, 0x0010},  // set tx2 and tx3 on in first 2 time slots in period-4 TDM mode
                                                                        // number of non-zeros values in array 'tx_groups' must equal to DDM_CHRP_PHAS_SEQ_PRD

        .rng_nfft                  = 256,
        .vel_nfft                  = 64,    // virtual size is 4, velocity FFT size decreased to 1/4
        .doa_fft_mux               = {0x000000FF, 0x00001111, 0x00008421},  // Choose RX FFTs generated in first 2 time slots in period-4 TDM mode for DOA
        .doa_npoint                = {360,360,360},

  The baseband flow configuration is shown in bb_flow_datproc_cfg.c.



 * NOTE:
 1. The velocity difference between 2 velocity FFT bins is denoted as 'vel_delta', and
    vel_delta = k*sys_params->vel_nfft * sys_params->chirp_period * cfg->nvarray (k is a constant coefficient)
    Therefore, 'vel_delta''s values of frame type 0 and frame type 1 are the same.
 2. DDM MIMO implemented on ALPS MP is not compatible with chirp-delay anti-velocity ambiguity
 3. DDM MIMO implemented on ALPS MP is not compatible with TDM MIMO in chirp generation
 4. DDM MIMO implemented on ALPS MP is only compatible with CA CFAR, since the Doppler domain FFT size for NVE(in FFT module) and CFAR/DOA are different.
 5. DDM MIMO implemented on ALPS MP is not compatible with CASCADE temporarily.


 >>>> UPDATE

 2021/06/25 : Update DDM with implementation in one bank, the sensor_config_init.hxx file should be modified according to ADC/FFT parameters.
 And also support different TXs on/off states in different frame types, while all TXs' phase shifters must be working following the same
 preset phase sequences in different frame types.

 One example setting:

 In bb_flow.h, set:
 #define DDM_EN                            true    // Enable DDM MIMO
 #define DDM_CHRP_PHAS_SEQ_PRD             4       // Repetition period of chirp phase shift sequence

 In bb_flow.c: ddm_tx_vel_pos[MAX_NUM_TX] = {1, 2, 3, 4};  // All phase shifters are on and values in array determine the phase sequences

 In sensor_config_init0.hxx : tx_group = {1, 0, 0, 1}, doa_fft_mux = {0x0000F00F, ...}   // TX0 and TX3 working in DDM MIMO mode, and forming signal vector for DoA
 In sensor_config_init1.hxx : tx_group = {0, 0, 1, 1}, doa_fft_mux = {0x0000FF00, ...}   // TX2 and TX3 working in DDM MIMO mode, and forming signal vector for DoA

 */



#endif


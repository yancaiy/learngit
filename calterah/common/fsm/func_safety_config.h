#ifndef func_safety_config_h
#define func_safety_config_h

#define FEATURE_ON          1
#define FEATURE_OFF         0

#define FEATURE_DEFAULT     FEATURE_OFF
/* FUSA CLI switch*/
#ifndef FUNC_SAFETY_CLI
#define FUNC_SAFETY_CLI  FEATURE_DEFAULT
#endif

/* FUSA items switch*/
#ifndef SAFETY_FEATURE_SM1
#define SAFETY_FEATURE_SM1  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM2
#define SAFETY_FEATURE_SM2  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM3
#define SAFETY_FEATURE_SM3  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM4
#define SAFETY_FEATURE_SM4  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM5
#define SAFETY_FEATURE_SM5  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM6
#define SAFETY_FEATURE_SM6  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM8
#define SAFETY_FEATURE_SM8  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM11
#define SAFETY_FEATURE_SM11 FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM12
#define SAFETY_FEATURE_SM12  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM13
#define SAFETY_FEATURE_SM13  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM14
#define SAFETY_FEATURE_SM14  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM101
#define SAFETY_FEATURE_SM101  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM102
#define SAFETY_FEATURE_SM102  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM103
#define SAFETY_FEATURE_SM103  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM104
#define SAFETY_FEATURE_SM104  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM105
#define SAFETY_FEATURE_SM105  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM106
#define SAFETY_FEATURE_SM106  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM107
#define SAFETY_FEATURE_SM107  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM108
#define SAFETY_FEATURE_SM108  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM109
#define SAFETY_FEATURE_SM109  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM120
#define SAFETY_FEATURE_SM120  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM121
#define SAFETY_FEATURE_SM121  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM122
#define SAFETY_FEATURE_SM122  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM123
#define SAFETY_FEATURE_SM123  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM124
#define SAFETY_FEATURE_SM124  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM125
#define SAFETY_FEATURE_SM125  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM126
#define SAFETY_FEATURE_SM126  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM129
#define SAFETY_FEATURE_SM129  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM130
#define SAFETY_FEATURE_SM130  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM133
#define SAFETY_FEATURE_SM133  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM201
#define SAFETY_FEATURE_SM201  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM202
#define SAFETY_FEATURE_SM202  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM206
#define SAFETY_FEATURE_SM206  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM805
#define SAFETY_FEATURE_SM805  FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM901
#define SAFETY_FEATURE_SM901 FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM902
#define SAFETY_FEATURE_SM902 FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM904
#define SAFETY_FEATURE_SM904 FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM905
#define SAFETY_FEATURE_SM905 FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM906
#define SAFETY_FEATURE_SM906 FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM907
#define SAFETY_FEATURE_SM907 FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM908
#define SAFETY_FEATURE_SM908 FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM910
#define SAFETY_FEATURE_SM910 FEATURE_DEFAULT
#endif
#ifndef SAFETY_FEATURE_SM911
#define SAFETY_FEATURE_SM911 FEATURE_DEFAULT
#endif

/******************************************************************************************/
/******************************************************************************************/
#if SAFETY_FEATURE_SM109 == FEATURE_ON
#ifndef SAFETY_FEATURE_CAN0
#define SAFETY_FEATURE_CAN0  FEATURE_ON
#endif
#ifndef SAFETY_FEATURE_CAN1
#define SAFETY_FEATURE_CAN1  FEATURE_ON
#endif
#endif
/******************************************************************************************/
/******************************************************************************************/
#define SAFETY_MECHANISM_POWER_ON_CHECK_EN      ((SAFETY_FEATURE_SM901 == FEATURE_ON) || \
                                                 (SAFETY_FEATURE_SM902 == FEATURE_ON) || \
                                                 (SAFETY_FEATURE_SM904 == FEATURE_ON) || \
                                                 (SAFETY_FEATURE_SM905 == FEATURE_ON) || \
                                                 (SAFETY_FEATURE_SM906 == FEATURE_ON) || \
                                                 (SAFETY_FEATURE_SM907 == FEATURE_ON) || \
                                                 (SAFETY_FEATURE_SM908 == FEATURE_ON) || \
                                                 (SAFETY_FEATURE_SM911 == FEATURE_ON) || \
                                                 (SAFETY_FEATURE_SM922 == FEATURE_ON))

#define SAFETY_PERIODIC_RUN_ITEMS_EN            ((SAFETY_FEATURE_SM1   == FEATURE_ON) || \
                                                 (SAFETY_FEATURE_SM6   == FEATURE_ON) || \
                                                 (SAFETY_FEATURE_SM11  == FEATURE_ON) || \
                                                 (SAFETY_FEATURE_SM12  == FEATURE_ON) || \
                                                 (SAFETY_FEATURE_SM13  == FEATURE_ON) || \
                                                 (SAFETY_FEATURE_SM14  == FEATURE_ON) || \
                                                 (SAFETY_FEATURE_SM101 == FEATURE_ON) || \
                                                 (SAFETY_FEATURE_SM201 == FEATURE_ON))

#endif


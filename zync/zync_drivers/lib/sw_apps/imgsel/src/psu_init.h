/******************************************************************************
* Copyright (c) 2020 Xilinx, Inc. All rights reserved.
* SPDX-License-Identifier: MIT
******************************************************************************/

/****************************************************************************/
/**
*
* @file psu_init.h
*
* This file is automatically generated
*
*****************************************************************************/

/*
* Divisor value for this clock.
*/
#undef CRL_APB_IOPLL_CFG_OFFSET
#define CRL_APB_IOPLL_CFG_OFFSET                                                   (0XFF5E0024U)
#undef CRL_APB_IOPLL_CTRL_OFFSET
#define CRL_APB_IOPLL_CTRL_OFFSET                                                  (0XFF5E0020U)
#undef CRF_APB_APLL_CFG_OFFSET
#define CRF_APB_APLL_CFG_OFFSET                                                    (0XFD1A0024U)
#undef CRF_APB_APLL_CTRL_OFFSET
#define CRF_APB_APLL_CTRL_OFFSET                                                   (0XFD1A0020U)
#undef CRL_APB_I2C1_REF_CTRL_OFFSET
#define CRL_APB_I2C1_REF_CTRL_OFFSET                                               (0XFF5E0124U)
#undef CRL_APB_UART0_REF_CTRL_OFFSET
#define CRL_APB_UART0_REF_CTRL_OFFSET                                              (0XFF5E0074U)
#undef CRL_APB_QSPI_REF_CTRL_OFFSET
#define CRL_APB_QSPI_REF_CTRL_OFFSET                                               (0XFF5E0068U)

#undef IOU_SLCR_MIO_PIN_0_OFFSET
#define IOU_SLCR_MIO_PIN_0_OFFSET                                                  (0XFF180000U)
#undef IOU_SLCR_MIO_PIN_1_OFFSET
#define IOU_SLCR_MIO_PIN_1_OFFSET                                                  (0XFF180004U)
#undef IOU_SLCR_MIO_PIN_2_OFFSET
#define IOU_SLCR_MIO_PIN_2_OFFSET                                                  (0XFF180008U)
#undef IOU_SLCR_MIO_PIN_3_OFFSET
#define IOU_SLCR_MIO_PIN_3_OFFSET                                                  (0XFF18000CU)
#undef IOU_SLCR_MIO_PIN_4_OFFSET
#define IOU_SLCR_MIO_PIN_4_OFFSET                                                  (0XFF180010U)
#undef IOU_SLCR_MIO_PIN_5_OFFSET
#define IOU_SLCR_MIO_PIN_5_OFFSET                                                  (0XFF180014U)
#undef IOU_SLCR_MIO_PIN_6_OFFSET
#define IOU_SLCR_MIO_PIN_6_OFFSET                                                  (0XFF180018U)
#undef IOU_SLCR_MIO_PIN_7_OFFSET
#define IOU_SLCR_MIO_PIN_7_OFFSET                                                  (0XFF18001CU)
#undef IOU_SLCR_MIO_PIN_8_OFFSET
#define IOU_SLCR_MIO_PIN_8_OFFSET                                                  (0XFF180020U)
#undef IOU_SLCR_MIO_PIN_9_OFFSET
#define IOU_SLCR_MIO_PIN_9_OFFSET                                                  (0XFF180024U)
#undef IOU_SLCR_MIO_PIN_10_OFFSET
#define IOU_SLCR_MIO_PIN_10_OFFSET                                                 (0XFF180028U)
#undef IOU_SLCR_MIO_PIN_11_OFFSET
#define IOU_SLCR_MIO_PIN_11_OFFSET                                                 (0XFF18002CU)
#undef IOU_SLCR_MIO_PIN_12_OFFSET
#define IOU_SLCR_MIO_PIN_12_OFFSET                                                 (0XFF180030U)
#undef IOU_SLCR_MIO_PIN_16_OFFSET
#define IOU_SLCR_MIO_PIN_16_OFFSET                                                 (0XFF180040U)
#undef IOU_SLCR_MIO_PIN_17_OFFSET
#define IOU_SLCR_MIO_PIN_17_OFFSET                                                 (0XFF180044U)
#undef IOU_SLCR_MIO_PIN_18_OFFSET
#define IOU_SLCR_MIO_PIN_18_OFFSET                                                 (0XFF180048U)
#undef IOU_SLCR_MIO_PIN_19_OFFSET
#define IOU_SLCR_MIO_PIN_19_OFFSET                                                 (0XFF18004CU)
#undef IOU_SLCR_MIO_MST_TRI0_OFFSET
#define IOU_SLCR_MIO_MST_TRI0_OFFSET                                               (0XFF180204U)

#undef IOU_SLCR_IOU_TAPDLY_BYPASS_OFFSET
#define IOU_SLCR_IOU_TAPDLY_BYPASS_OFFSET                                          (0XFF180390U)
#undef IOU_SECURE_SLCR_IOU_AXI_WPRTCN_OFFSET
#define IOU_SECURE_SLCR_IOU_AXI_WPRTCN_OFFSET									   (0XFF240000U)

#undef CRL_APB_RST_LPD_IOU2_OFFSET
#define CRL_APB_RST_LPD_IOU2_OFFSET                                                (0XFF5E0238U)
#define CRL_APB_PLL_STATUS_OFFSET                                                  (0XFF5E0040U)
#define CRF_APB_PLL_STATUS_OFFSET												   (0XFD1A0044U)
#define PSU_MASK_POLL_TIME							   							   (1100000U)

#ifdef __cplusplus
extern "C" {
#endif

int Psu_Init ();

#ifdef __cplusplus
}
#endif
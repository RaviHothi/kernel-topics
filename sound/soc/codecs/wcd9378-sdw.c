// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.

#include <linux/component.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/soundwire/sdw.h>
#include <linux/soundwire/sdw_registers.h>
#include <linux/soundwire/sdw_type.h>
#include <sound/soc-dapm.h>
#include <sound/soc.h>
#include "wcd9378.h"
#include "wcd-common.h"

/*
 * WCD9378 RX channel info table.
 * Maps logical channel IDs to SoundWire port numbers and channel masks.
 * 5 RX ports: HPH (stereo), CLSH, COMP (stereo), LO, DSD (stereo).
 */
static struct wcd_sdw_ch_info wcd9378_sdw_rx_ch_info[] = {
	WCD_SDW_CH(WCD9378_HPH_L,  WCD9378_HPH_PORT,  BIT(0)),
	WCD_SDW_CH(WCD9378_HPH_R,  WCD9378_HPH_PORT,  BIT(1)),
	WCD_SDW_CH(WCD9378_CLSH,   WCD9378_CLSH_PORT, BIT(0)),
	WCD_SDW_CH(WCD9378_COMP_L, WCD9378_COMP_PORT, BIT(0)),
	WCD_SDW_CH(WCD9378_COMP_R, WCD9378_COMP_PORT, BIT(1)),
	WCD_SDW_CH(WCD9378_LO,     WCD9378_LO_PORT,   BIT(0)),
	WCD_SDW_CH(WCD9378_DSD_L,  WCD9378_DSD_PORT,  BIT(0)),
	WCD_SDW_CH(WCD9378_DSD_R,  WCD9378_DSD_PORT,  BIT(1)),
};

/*
 * WCD9378 TX channel info table.
 * Maps logical channel IDs to SoundWire port numbers and channel masks.
 * 5 TX ports across 2 physical TX SoundWire lanes.
 */
static struct wcd_sdw_ch_info wcd9378_sdw_tx_ch_info[] = {
	WCD_SDW_CH(WCD9378_ADC1,  WCD9378_ADC_1_PORT,         BIT(0)),
	WCD_SDW_CH(WCD9378_ADC2,  WCD9378_ADC_2_3_PORT,       BIT(0)),
	WCD_SDW_CH(WCD9378_ADC3,  WCD9378_ADC_2_3_PORT,       BIT(1)),
	WCD_SDW_CH(WCD9378_DMIC0, WCD9378_DMIC_0_3_MBHC_PORT, BIT(0)),
	WCD_SDW_CH(WCD9378_DMIC1, WCD9378_DMIC_0_3_MBHC_PORT, BIT(1)),
	WCD_SDW_CH(WCD9378_MBHC,  WCD9378_DMIC_0_3_MBHC_PORT, BIT(2)),
	WCD_SDW_CH(WCD9378_DMIC2, WCD9378_DMIC_0_3_MBHC_PORT, BIT(2)),
	WCD_SDW_CH(WCD9378_DMIC3, WCD9378_DMIC_0_3_MBHC_PORT, BIT(3)),
	WCD_SDW_CH(WCD9378_DMIC4, WCD9378_DMIC_4_7_PORT,      BIT(0)),
	WCD_SDW_CH(WCD9378_DMIC5, WCD9378_DMIC_4_7_PORT,      BIT(1)),
	WCD_SDW_CH(WCD9378_DMIC6, WCD9378_DMIC_4_7_PORT,      BIT(2)),
	WCD_SDW_CH(WCD9378_DMIC7, WCD9378_DMIC_4_7_PORT,      BIT(3)),
};

/*
 * Data port properties for WCD9378.
 * All 5 ports use SDW_DPN_SIMPLE type with BPP packing.
 */
static struct sdw_dpn_prop wcd9378_dpn_prop[WCD9378_MAX_SWR_PORTS] = {
	{
		.num = 1,
		.type = SDW_DPN_SIMPLE,
		.min_ch = 1,
		.max_ch = 8,
		.simple_ch_prep_sm = true,
	}, {
		.num = 2,
		.type = SDW_DPN_SIMPLE,
		.min_ch = 1,
		.max_ch = 4,
		.simple_ch_prep_sm = true,
	}, {
		.num = 3,
		.type = SDW_DPN_SIMPLE,
		.min_ch = 1,
		.max_ch = 4,
		.simple_ch_prep_sm = true,
	}, {
		.num = 4,
		.type = SDW_DPN_SIMPLE,
		.min_ch = 1,
		.max_ch = 4,
		.simple_ch_prep_sm = true,
	}, {
		.num = 5,
		.type = SDW_DPN_SIMPLE,
		.min_ch = 1,
		.max_ch = 4,
		.simple_ch_prep_sm = true,
	},
};

/**
 * wcd9378_sdw_hw_params - Configure SoundWire stream for hw_params
 * @wcd:	SDW private data
 * @substream:	PCM substream
 * @params:	Hardware parameters
 * @dai:	DAI pointer
 *
 * Iterates active port configs, counts channels, and adds this slave
 * to the SoundWire stream runtime.
 */
int wcd9378_sdw_hw_params(struct wcd9378_sdw_priv *wcd,
			  struct snd_pcm_substream *substream,
			  struct snd_pcm_hw_params *params,
			  struct snd_soc_dai *dai)
{
	struct sdw_port_config port_config[WCD9378_MAX_SWR_PORTS];
	unsigned long ch_mask;
	int i, j;

	wcd->sconfig.ch_count = 1;
	wcd->active_ports = 0;

	for (i = 0; i < WCD9378_MAX_SWR_PORTS; i++) {
		ch_mask = wcd->port_config[i].ch_mask;
		if (!ch_mask)
			continue;

		for_each_set_bit(j, &ch_mask, 4)
			wcd->sconfig.ch_count++;

		port_config[wcd->active_ports] = wcd->port_config[i];
		wcd->active_ports++;
	}

	wcd->sconfig.bps = 1;
	wcd->sconfig.frame_rate = params_rate(params);
	wcd->sconfig.direction = wcd->is_tx ? SDW_DATA_DIR_TX : SDW_DATA_DIR_RX;
	wcd->sconfig.type = SDW_STREAM_PCM;

	return sdw_stream_add_slave(wcd->sdev, &wcd->sconfig,
				    &port_config[0], wcd->active_ports,
				    wcd->sruntime);
}
EXPORT_SYMBOL_GPL(wcd9378_sdw_hw_params);

/**
 * wcd9378_sdw_free - Remove slave from SoundWire stream
 * @wcd:	SDW private data
 * @substream:	PCM substream
 * @dai:	DAI pointer
 */
int wcd9378_sdw_free(struct wcd9378_sdw_priv *wcd,
		     struct snd_pcm_substream *substream,
		     struct snd_soc_dai *dai)
{
	return sdw_stream_remove_slave(wcd->sdev, wcd->sruntime);
}
EXPORT_SYMBOL_GPL(wcd9378_sdw_free);

/**
 * wcd9378_sdw_set_sdw_stream - Attach a SoundWire stream runtime
 * @wcd:	SDW private data
 * @dai:	DAI pointer
 * @stream:	SoundWire stream runtime pointer
 * @direction:	SNDRV_PCM_STREAM_PLAYBACK or SNDRV_PCM_STREAM_CAPTURE
 */
int wcd9378_sdw_set_sdw_stream(struct wcd9378_sdw_priv *wcd,
			       struct snd_soc_dai *dai,
			       void *stream, int direction)
{
	wcd->sruntime = stream;
	return 0;
}
EXPORT_SYMBOL_GPL(wcd9378_sdw_set_sdw_stream);

/*
 * Handle WCD9378 SDCA out-of-band interrupt.
 * The SDCA interrupt status is in SWRS_SCP_SDCA_INTSTAT_1/2/3.
 * Trigger the first IRQ of the slave_irq domain so regmap_irq handles it.
 */
static int wcd9378_interrupt_callback(struct sdw_slave *slave,
				      struct sdw_slave_intr_status *status)
{
	struct wcd9378_sdw_priv *wcd = dev_get_drvdata(&slave->dev);

	return wcd_interrupt_callback(slave, wcd->slave_irq,
				      SWRS_SCP_SDCA_INTSTAT_1,
				      SWRS_SCP_SDCA_INTSTAT_2,
				      SWRS_SCP_SDCA_INTSTAT_3);
}

/*
 * WCD9378 register defaults.
 * Only non-volatile, non-read-only registers are listed here.
 * SDCA interrupt status/mask/type registers are handled separately.
 */
static const struct reg_default wcd9378_defaults[] = {
	{ SWRS_SCP_SDCA_INTMASK_1,			0x00 },
	{ SWRS_SCP_SDCA_INTMASK_2,			0x00 },
	{ SWRS_SCP_SDCA_INTMASK_3,			0x00 },
	{ SWRS_SCP_SDCA_INTRTYPE_1,			0x00 },
	{ SWRS_SCP_SDCA_INTRTYPE_2,			0x00 },
	{ SWRS_SCP_SDCA_INTRTYPE_3,			0x00 },
	{ WCD9378_ANA_BIAS,				0x00 },
	{ WCD9378_ANA_RX_SUPPLIES,			0x00 },
	{ WCD9378_ANA_HPH,				0x0c },
	{ WCD9378_ANA_EAR,				0x00 },
	{ WCD9378_ANA_EAR_COMPANDER_CTL,		0x02 },
	{ WCD9378_ANA_TX_CH1,				0x20 },
	{ WCD9378_ANA_TX_CH2,				0x00 },
	{ WCD9378_ANA_TX_CH3,				0x20 },
	{ WCD9378_ANA_TX_CH3_HPF,			0x00 },
	{ WCD9378_ANA_MICB1_MICB2_DSP_EN_LOGIC,		0x00 },
	{ WCD9378_ANA_MICB3_DSP_EN_LOGIC,		0x00 },
	{ WCD9378_ANA_MBHC_MECH,			0x39 },
	{ WCD9378_ANA_MBHC_ELECT,			0x08 },
	{ WCD9378_ANA_MBHC_ZDET,			0x00 },
	{ WCD9378_ANA_MBHC_BTN0,			0x00 },
	{ WCD9378_ANA_MBHC_BTN1,			0x10 },
	{ WCD9378_ANA_MBHC_BTN2,			0x20 },
	{ WCD9378_ANA_MBHC_BTN3,			0x30 },
	{ WCD9378_ANA_MBHC_BTN4,			0x40 },
	{ WCD9378_ANA_MBHC_BTN5,			0x50 },
	{ WCD9378_ANA_MBHC_BTN6,			0x60 },
	{ WCD9378_ANA_MBHC_BTN7,			0x70 },
	{ WCD9378_ANA_MICB1,				0x10 },
	{ WCD9378_ANA_MICB2,				0x10 },
	{ WCD9378_ANA_MICB2_RAMP,			0x00 },
	{ WCD9378_ANA_MICB3,				0x00 },
	{ WCD9378_BIAS_CTL,				0x2a },
	{ WCD9378_BIAS_VBG_FINE_ADJ,			0x55 },
	{ WCD9378_LDOL_VDDCX_ADJUST,			0x01 },
	{ WCD9378_LDOL_DISABLE_LDOL,			0x00 },
	{ WCD9378_MBHC_CTL_CLK,			0x00 },
	{ WCD9378_MBHC_CTL_ANA,			0x00 },
	{ WCD9378_MBHC_CTL_SPARE_1,			0x02 },
	{ WCD9378_MBHC_CTL_SPARE_2,			0x00 },
	{ WCD9378_MBHC_CTL_BCS,			0x00 },
	{ WCD9378_MBHC_TEST_CTL,			0x00 },
	{ WCD9378_LDOH_MODE,				0x2b },
	{ WCD9378_LDOH_BIAS,				0x68 },
	{ WCD9378_LDOH_STB_LOADS,			0x00 },
	{ WCD9378_LDOH_SLOWRAMP,			0x50 },
	{ WCD9378_MICB1_TEST_CTL_1,			0x1a },
	{ WCD9378_MICB1_TEST_CTL_2,			0x00 },
	{ WCD9378_MICB1_TEST_CTL_3,			0xa4 },
	{ WCD9378_MICB2_TEST_CTL_1,			0x1a },
	{ WCD9378_MICB2_TEST_CTL_2,			0x00 },
	{ WCD9378_MICB2_TEST_CTL_3,			0x24 },
	{ WCD9378_MICB3_TEST_CTL_1,			0x9a },
	{ WCD9378_MICB3_TEST_CTL_2,			0x80 },
	{ WCD9378_MICB3_TEST_CTL_3,			0x24 },
	{ WCD9378_TX_COM_ADC_VCM,			0x39 },
	{ WCD9378_TX_COM_BIAS_ATEST,			0xe0 },
	{ WCD9378_TX_COM_TXFE_DIV_CTL,			0x22 },
	{ WCD9378_TX_COM_TXFE_DIV_START,		0x00 },
	{ WCD9378_TX_1_2_TEST_EN,			0xcc },
	{ WCD9378_TX_1_2_ADC_IB,			0xe9 },
	{ WCD9378_TX_1_2_ATEST_REFCTL,			0x0b },
	{ WCD9378_TX_1_2_TEST_CTL,			0x38 },
	{ WCD9378_TX_1_2_TEST_BLK_EN1,			0xff },
	{ WCD9378_TX_1_2_TXFE1_CLKDIV,			0x00 },
	{ WCD9378_TX_3_TEST_EN,				0xcc },
	{ WCD9378_TX_3_ADC_IB,				0xe9 },
	{ WCD9378_TX_3_ATEST_REFCTL,			0x0b },
	{ WCD9378_TX_3_TEST_CTL,			0x38 },
	{ WCD9378_TX_3_TEST_BLK_EN3,			0xff },
	{ WCD9378_TX_3_TXFE3_CLKDIV,			0x00 },
	{ WCD9378_TX_3_TEST_BLK_EN2,			0xfb },
	{ WCD9378_TX_3_TXFE2_CLKDIV,			0x00 },
	{ WCD9378_RX_AUX_SW_CTL,			0x00 },
	{ WCD9378_RX_PA_AUX_IN_CONN,			0x00 },
	{ WCD9378_RX_TIMER_DIV,				0x32 },
	{ WCD9378_RX_OCP_CTL,				0x1f },
	{ WCD9378_RX_OCP_COUNT,				0x77 },
	{ WCD9378_RX_BIAS_EAR_DAC,			0xa0 },
	{ WCD9378_RX_BIAS_EAR_AMP,			0xaa },
	{ WCD9378_RX_BIAS_HPH_LDO,			0xa9 },
	{ WCD9378_RX_BIAS_HPH_PA,			0xaa },
	{ WCD9378_RX_BIAS_HPH_RDACBUFF_CNP2,		0x8a },
	{ WCD9378_RX_BIAS_HPH_RDAC_LDO,		0x88 },
	{ WCD9378_RX_BIAS_HPH_CNP1,			0x82 },
	{ WCD9378_RX_BIAS_HPH_LOWPOWER,			0x82 },
	{ WCD9378_RX_BIAS_AUX_DAC,			0xa0 },
	{ WCD9378_RX_BIAS_AUX_AMP,			0xaa },
	{ WCD9378_HPH_CNP_EN,				0x80 },
	{ WCD9378_HPH_CNP_WG_CTL,			0x9a },
	{ WCD9378_HPH_CNP_WG_TIME,			0x14 },
	{ WCD9378_HPH_OCP_CTL,				0x28 },
	{ WCD9378_HPH_AUTO_CHOP,			0x16 },
	{ WCD9378_HPH_CHOP_CTL,				0x83 },
	{ WCD9378_HPH_PA_CTL1,				0x46 },
	{ WCD9378_HPH_PA_CTL2,				0x50 },
	{ WCD9378_HPH_L_EN,				0x80 },
	{ WCD9378_HPH_L_TEST,				0xe0 },
	{ WCD9378_HPH_L_ATEST,				0x50 },
	{ WCD9378_HPH_R_EN,				0x80 },
	{ WCD9378_HPH_R_TEST,				0xe0 },
	{ WCD9378_HPH_R_ATEST,				0x54 },
	{ WCD9378_HPH_RDAC_CLK_CTL1,			0x99 },
	{ WCD9378_HPH_RDAC_CLK_CTL2,			0x9b },
	{ WCD9378_HPH_RDAC_LDO_CTL,			0x33 },
	{ WCD9378_HPH_RDAC_CHOP_CLK_LP_CTL,		0x00 },
	{ WCD9378_HPH_REFBUFF_UHQA_CTL,			0xa8 },
	{ WCD9378_HPH_REFBUFF_LP_CTL,			0x0e },
	{ WCD9378_HPH_L_DAC_CTL,			0x20 },
	{ WCD9378_HPH_R_DAC_CTL,			0x20 },
	{ WCD9378_HPH_SURGE_HPHLR_SURGE_COMP_SEL,	0x55 },
	{ WCD9378_HPH_SURGE_HPHLR_SURGE_EN,		0x19 },
	{ WCD9378_HPH_SURGE_HPHLR_SURGE_MISC1,		0xa0 },
	{ WCD9378_EAR_EAR_EN_REG,			0x22 },
	{ WCD9378_EAR_EAR_PA_CON,			0x44 },
	{ WCD9378_EAR_EAR_SP_CON,			0xdb },
	{ WCD9378_EAR_EAR_DAC_CON,			0x80 },
	{ WCD9378_EAR_EAR_CNP_FSM_CON,			0xb2 },
	{ WCD9378_EAR_TEST_CTL,				0x00 },
	{ WCD9378_HPH_NEW_ANA_HPH2,			0x00 },
	{ WCD9378_HPH_NEW_ANA_HPH3,			0x00 },
	{ WCD9378_SLEEP_CTL,				0x16 },
	{ WCD9378_SLEEP_WATCHDOG_CTL,			0x00 },
	{ WCD9378_MBHC_NEW_ELECT_REM_CLAMP_CTL,		0x00 },
	{ WCD9378_MBHC_NEW_CTL_1,			0x0e },
	{ WCD9378_MBHC_NEW_CTL_2,			0x05 },
	{ WCD9378_MBHC_NEW_PLUG_DETECT_CTL,		0xe9 },
	{ WCD9378_MBHC_NEW_ZDET_ANA_CTL,		0x0f },
	{ WCD9378_MBHC_NEW_ZDET_RAMP_CTL,		0x00 },
	{ WCD9378_AUX_AUXPA,				0x00 },
	{ WCD9378_DIE_CRACK_DIE_CRK_DET_EN,		0x00 },
	{ WCD9378_TX_NEW_TX_CH12_MUX,			0x11 },
	{ WCD9378_TX_NEW_TX_CH34_MUX,			0x23 },
	{ WCD9378_HPH_NEW_INT_RDAC_GAIN_CTL,		0x40 },
	{ WCD9378_HPH_NEW_INT_RDAC_HD2_CTL_L,		0x81 },
	{ WCD9378_HPH_NEW_INT_RDAC_VREF_CTL,		0x10 },
	{ WCD9378_HPH_NEW_INT_RDAC_OVERRIDE_CTL,	0x00 },
	{ WCD9378_HPH_NEW_INT_RDAC_HD2_CTL_R,		0x81 },
	{ WCD9378_HPH_NEW_INT_PA_MISC1,			0x22 },
	{ WCD9378_HPH_NEW_INT_PA_MISC2,			0x00 },
	{ WCD9378_HPH_NEW_INT_PA_RDAC_MISC,		0x01 },
	{ WCD9378_HPH_NEW_INT_HPH_TIMER1,		0xfe },
	{ WCD9378_HPH_NEW_INT_HPH_TIMER2,		0x02 },
	{ WCD9378_HPH_NEW_INT_HPH_TIMER3,		0x4e },
	{ WCD9378_HPH_NEW_INT_HPH_TIMER4,		0x54 },
	{ WCD9378_HPH_NEW_INT_PA_RDAC_MISC2,		0x00 },
	{ WCD9378_HPH_NEW_INT_PA_RDAC_MISC3,		0x00 },
	{ WCD9378_RX_NEW_INT_HPH_RDAC_BIAS_LOHIFI,	0x62 },
	{ WCD9378_RX_NEW_INT_HPH_RDAC_BIAS_ULP,		0x01 },
	{ WCD9378_RX_NEW_INT_HPH_RDAC_LDO_LP,		0x11 },
	{ WCD9378_CP_CLASSG_CP_CTRL_0,			0x00 },
	{ WCD9378_CP_CLASSG_CP_CTRL_1,			0x00 },
	{ WCD9378_CP_CLASSG_CP_CTRL_2,			0x23 },
	{ WCD9378_CP_CLASSG_CP_CTRL_3,			0x03 },
	{ WCD9378_CP_CLASSG_CP_CTRL_4,			0x00 },
	{ WCD9378_CP_CLASSG_CP_CTRL_5,			0x0a },
	{ WCD9378_CP_CLASSG_CP_CTRL_6,			0x00 },
	{ WCD9378_CP_CLASSG_CP_CTRL_7,			0x00 },
	{ WCD9378_CP_VNEGDAC_CTRL_0,			0x23 },
	{ WCD9378_CP_VNEGDAC_CTRL_1,			0x00 },
	{ WCD9378_CP_VNEGDAC_CTRL_2,			0x00 },
	{ WCD9378_CP_VNEGDAC_CTRL_3,			0x00 },
	{ WCD9378_CP_CP_DTOP_CTRL_0,			0x00 },
	{ WCD9378_CP_CP_DTOP_CTRL_1,			0x1b },
	{ WCD9378_CP_CP_DTOP_CTRL_9,			0x63 },
	{ WCD9378_MBHC_NEW_INT_MOISTURE_DET_DC_CTRL,	0x57 },
	{ WCD9378_MBHC_NEW_INT_MOISTURE_DET_POLLING_CTRL, 0x01 },
	{ WCD9378_MBHC_NEW_INT_MECH_DET_CURRENT,	0x00 },
	{ WCD9378_MBHC_NEW_INT_SPARE_2,			0x00 },
	{ WCD9378_EAR_INT_NEW_EAR_CHOPPER_CON,		0xa8 },
	{ WCD9378_EAR_INT_NEW_CNP_VCM_CON1,		0x42 },
	{ WCD9378_EAR_INT_NEW_CNP_VCM_CON2,		0x22 },
	{ WCD9378_EAR_INT_NEW_EAR_DYNAMIC_BIAS,		0x00 },
	{ WCD9378_AUX_INT_EN_REG,			0x00 },
	{ WCD9378_AUX_INT_PA_CTRL,			0x06 },
	{ WCD9378_AUX_INT_SP_CTRL,			0xd2 },
	{ WCD9378_AUX_INT_DAC_CTRL,			0x80 },
	{ WCD9378_AUX_INT_CLK_CTRL,			0x50 },
	{ WCD9378_AUX_INT_TEST_CTRL,			0x00 },
	{ WCD9378_AUX_INT_MISC,				0x00 },
	/* SDCA function registers */
	{ WCD9378_XU22_BYP,				0x01 },
	{ WCD9378_PDE22_REQ_PS,				0x03 },
	{ WCD9378_FU23_MUTE,				0x01 },
	{ WCD9378_PDE23_REQ_PS,				0x03 },
	{ WCD9378_IT41_USAGE,				0x03 },
	{ WCD9378_XU42_BYP,				0x01 },
	{ WCD9378_PDE42_REQ_PS,				0x03 },
	{ WCD9378_FU42_MUTE_CH1,			0x01 },
	{ WCD9378_FU42_MUTE_CH2,			0x01 },
	{ WCD9378_FU42_MUTE_CH1_CN,			0x01 },
	{ WCD9378_FU42_MUTE_CH2_CN,			0x01 },
	{ WCD9378_SU43_SELECTOR,			0x01 },
	{ WCD9378_SU45_SELECTOR,			0x01 },
	{ WCD9378_PDE47_REQ_PS,				0x03 },
	{ WCD9378_GE35_SEL_MODE,			0x00 },
	{ WCD9378_GE35_DET_MODE,			0x00 },
	{ WCD9378_IT31_MICB,				0x00 },
	{ WCD9378_IT31_USAGE,				0x03 },
	{ WCD9378_PDE34_REQ_PS,				0x03 },
	{ WCD9378_SU45_TX_SELECTOR,			0x01 },
	{ WCD9378_XU36_BYP,				0x01 },
	{ WCD9378_PDE36_REQ_PS,				0x03 },
	{ WCD9378_OT36_USAGE,				0x03 },
	{ WCD9378_IT11_MICB,				0x00 },
	{ WCD9378_IT11_USAGE,				0x03 },
	{ WCD9378_PDE11_REQ_PS,				0x03 },
	{ WCD9378_OT10_USAGE,				0x03 },
	{ WCD9378_SMP_MIC_CTRL1_IT11_MICB,		0x00 },
	{ WCD9378_SMP_MIC_CTRL1_IT11_USAGE,		0x03 },
	{ WCD9378_SMP_MIC_CTRL1_PDE11_REQ_PS,		0x03 },
	{ WCD9378_SMP_MIC_CTRL2_IT11_MICB,		0x00 },
	{ WCD9378_SMP_MIC_CTRL2_IT11_USAGE,		0x03 },
	{ WCD9378_SMP_MIC_CTRL2_PDE11_REQ_PS,		0x03 },
	{ WCD9378_REPORT_ID,				0x01 },
	{ WCD9378_MESSAGE0,				0x00 },
	{ WCD9378_MESSAGE1,				0x00 },
	{ WCD9378_MESSAGE2,				0x00 },
};

static bool wcd9378_rdwr_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case WCD9378_ANA_BIAS:
	case WCD9378_ANA_RX_SUPPLIES:
	case WCD9378_ANA_HPH:
	case WCD9378_ANA_EAR:
	case WCD9378_ANA_EAR_COMPANDER_CTL:
	case WCD9378_ANA_TX_CH1:
	case WCD9378_ANA_TX_CH2:
	case WCD9378_ANA_TX_CH3:
	case WCD9378_ANA_TX_CH3_HPF:
	case WCD9378_ANA_MICB1_MICB2_DSP_EN_LOGIC:
	case WCD9378_ANA_MICB3_DSP_EN_LOGIC:
	case WCD9378_ANA_MBHC_MECH:
	case WCD9378_ANA_MBHC_ELECT:
	case WCD9378_ANA_MBHC_ZDET:
	case WCD9378_ANA_MBHC_BTN0 ... WCD9378_ANA_MBHC_BTN7:
	case WCD9378_ANA_MICB1:
	case WCD9378_ANA_MICB2:
	case WCD9378_ANA_MICB2_RAMP:
	case WCD9378_ANA_MICB3:
	case WCD9378_BIAS_CTL:
	case WCD9378_BIAS_VBG_FINE_ADJ:
	case WCD9378_LDOL_VDDCX_ADJUST:
	case WCD9378_LDOL_DISABLE_LDOL:
	case WCD9378_MBHC_CTL_CLK:
	case WCD9378_MBHC_CTL_ANA:
	case WCD9378_MBHC_CTL_SPARE_1:
	case WCD9378_MBHC_CTL_SPARE_2:
	case WCD9378_MBHC_CTL_BCS:
	case WCD9378_MBHC_TEST_CTL:
	case WCD9378_LDOH_MODE:
	case WCD9378_LDOH_BIAS:
	case WCD9378_LDOH_STB_LOADS:
	case WCD9378_LDOH_SLOWRAMP:
	case WCD9378_MICB1_TEST_CTL_1 ... WCD9378_MICB3_TEST_CTL_3:
	case WCD9378_TX_COM_ADC_VCM:
	case WCD9378_TX_COM_BIAS_ATEST:
	case WCD9378_TX_COM_TXFE_DIV_CTL:
	case WCD9378_TX_COM_TXFE_DIV_START:
	case WCD9378_TX_1_2_TEST_EN:
	case WCD9378_TX_1_2_ADC_IB:
	case WCD9378_TX_1_2_ATEST_REFCTL:
	case WCD9378_TX_1_2_TEST_CTL:
	case WCD9378_TX_1_2_TEST_BLK_EN1:
	case WCD9378_TX_1_2_TXFE1_CLKDIV:
	case WCD9378_TX_3_TEST_EN:
	case WCD9378_TX_3_ADC_IB:
	case WCD9378_TX_3_ATEST_REFCTL:
	case WCD9378_TX_3_TEST_CTL:
	case WCD9378_TX_3_TEST_BLK_EN3:
	case WCD9378_TX_3_TXFE3_CLKDIV:
	case WCD9378_TX_3_TEST_BLK_EN2:
	case WCD9378_TX_3_TXFE2_CLKDIV:
	case WCD9378_RX_AUX_SW_CTL:
	case WCD9378_RX_PA_AUX_IN_CONN:
	case WCD9378_RX_TIMER_DIV:
	case WCD9378_RX_OCP_CTL:
	case WCD9378_RX_OCP_COUNT:
	case WCD9378_RX_BIAS_EAR_DAC:
	case WCD9378_RX_BIAS_EAR_AMP:
	case WCD9378_RX_BIAS_HPH_LDO:
	case WCD9378_RX_BIAS_HPH_PA:
	case WCD9378_RX_BIAS_HPH_RDACBUFF_CNP2:
	case WCD9378_RX_BIAS_HPH_RDAC_LDO:
	case WCD9378_RX_BIAS_HPH_CNP1:
	case WCD9378_RX_BIAS_HPH_LOWPOWER:
	case WCD9378_RX_BIAS_AUX_DAC:
	case WCD9378_RX_BIAS_AUX_AMP:
	case WCD9378_HPH_CNP_EN:
	case WCD9378_HPH_CNP_WG_CTL:
	case WCD9378_HPH_CNP_WG_TIME:
	case WCD9378_HPH_OCP_CTL:
	case WCD9378_HPH_AUTO_CHOP:
	case WCD9378_HPH_CHOP_CTL:
	case WCD9378_HPH_PA_CTL1:
	case WCD9378_HPH_PA_CTL2:
	case WCD9378_HPH_L_EN:
	case WCD9378_HPH_L_TEST:
	case WCD9378_HPH_L_ATEST:
	case WCD9378_HPH_R_EN:
	case WCD9378_HPH_R_TEST:
	case WCD9378_HPH_R_ATEST:
	case WCD9378_HPH_RDAC_CLK_CTL1:
	case WCD9378_HPH_RDAC_CLK_CTL2:
	case WCD9378_HPH_RDAC_LDO_CTL:
	case WCD9378_HPH_RDAC_CHOP_CLK_LP_CTL:
	case WCD9378_HPH_REFBUFF_UHQA_CTL:
	case WCD9378_HPH_REFBUFF_LP_CTL:
	case WCD9378_HPH_L_DAC_CTL:
	case WCD9378_HPH_R_DAC_CTL:
	case WCD9378_HPH_SURGE_HPHLR_SURGE_COMP_SEL:
	case WCD9378_HPH_SURGE_HPHLR_SURGE_EN:
	case WCD9378_HPH_SURGE_HPHLR_SURGE_MISC1:
	case WCD9378_EAR_EAR_EN_REG:
	case WCD9378_EAR_EAR_PA_CON:
	case WCD9378_EAR_EAR_SP_CON:
	case WCD9378_EAR_EAR_DAC_CON:
	case WCD9378_EAR_EAR_CNP_FSM_CON:
	case WCD9378_EAR_TEST_CTL:
	case WCD9378_HPH_NEW_ANA_HPH2:
	case WCD9378_HPH_NEW_ANA_HPH3:
	case WCD9378_SLEEP_CTL:
	case WCD9378_SLEEP_WATCHDOG_CTL:
	case WCD9378_MBHC_NEW_ELECT_REM_CLAMP_CTL:
	case WCD9378_MBHC_NEW_CTL_1:
	case WCD9378_MBHC_NEW_CTL_2:
	case WCD9378_MBHC_NEW_PLUG_DETECT_CTL:
	case WCD9378_MBHC_NEW_ZDET_ANA_CTL:
	case WCD9378_MBHC_NEW_ZDET_RAMP_CTL:
	case WCD9378_AUX_AUXPA:
	case WCD9378_DIE_CRACK_DIE_CRK_DET_EN:
	case WCD9378_TX_NEW_TX_CH12_MUX:
	case WCD9378_TX_NEW_TX_CH34_MUX:
	case WCD9378_HPH_NEW_INT_RDAC_GAIN_CTL:
	case WCD9378_HPH_NEW_INT_RDAC_HD2_CTL_L:
	case WCD9378_HPH_NEW_INT_RDAC_VREF_CTL:
	case WCD9378_HPH_NEW_INT_RDAC_OVERRIDE_CTL:
	case WCD9378_HPH_NEW_INT_RDAC_HD2_CTL_R:
	case WCD9378_HPH_NEW_INT_PA_MISC1:
	case WCD9378_HPH_NEW_INT_PA_MISC2:
	case WCD9378_HPH_NEW_INT_PA_RDAC_MISC:
	case WCD9378_HPH_NEW_INT_HPH_TIMER1:
	case WCD9378_HPH_NEW_INT_HPH_TIMER2:
	case WCD9378_HPH_NEW_INT_HPH_TIMER3:
	case WCD9378_HPH_NEW_INT_HPH_TIMER4:
	case WCD9378_HPH_NEW_INT_PA_RDAC_MISC2:
	case WCD9378_HPH_NEW_INT_PA_RDAC_MISC3:
	case WCD9378_RX_NEW_INT_HPH_RDAC_BIAS_LOHIFI:
	case WCD9378_RX_NEW_INT_HPH_RDAC_BIAS_ULP:
	case WCD9378_RX_NEW_INT_HPH_RDAC_LDO_LP:
	case WCD9378_CP_CLASSG_CP_CTRL_0 ... WCD9378_CP_CLASSG_CP_CTRL_7:
	case WCD9378_CP_VNEGDAC_CTRL_0 ... WCD9378_CP_VNEGDAC_CTRL_3:
	case WCD9378_CP_CP_DTOP_CTRL_0:
	case WCD9378_CP_CP_DTOP_CTRL_1:
	case WCD9378_CP_CP_DTOP_CTRL_9:
	case WCD9378_MBHC_NEW_INT_MOISTURE_DET_DC_CTRL:
	case WCD9378_MBHC_NEW_INT_MOISTURE_DET_POLLING_CTRL:
	case WCD9378_MBHC_NEW_INT_MECH_DET_CURRENT:
	case WCD9378_MBHC_NEW_INT_SPARE_2:
	case WCD9378_EAR_INT_NEW_EAR_CHOPPER_CON:
	case WCD9378_EAR_INT_NEW_CNP_VCM_CON1:
	case WCD9378_EAR_INT_NEW_CNP_VCM_CON2:
	case WCD9378_EAR_INT_NEW_EAR_DYNAMIC_BIAS:
	case WCD9378_AUX_INT_EN_REG:
	case WCD9378_AUX_INT_PA_CTRL:
	case WCD9378_AUX_INT_SP_CTRL:
	case WCD9378_AUX_INT_DAC_CTRL:
	case WCD9378_AUX_INT_CLK_CTRL:
	case WCD9378_AUX_INT_TEST_CTRL:
	case WCD9378_AUX_INT_MISC:
	/* SDCA function registers */
	case WCD9378_XU22_BYP:
	case WCD9378_PDE22_REQ_PS:
	case WCD9378_FU23_MUTE:
	case WCD9378_PDE23_REQ_PS:
	case WCD9378_IT41_USAGE:
	case WCD9378_XU42_BYP:
	case WCD9378_PDE42_REQ_PS:
	case WCD9378_FU42_MUTE_CH1:
	case WCD9378_FU42_MUTE_CH2:
	case WCD9378_FU42_MUTE_CH1_CN:
	case WCD9378_FU42_MUTE_CH2_CN:
	case WCD9378_FU42_CH_VOL_CH1:
	case WCD9378_FU42_CH_VOL_CH1_MSB:
	case WCD9378_FU42_CH_VOL_CH1_LSB:
	case WCD9378_FU42_CH_VOL_CH2:
	case WCD9378_FU42_CH_VOL_CH2_MSB:
	case WCD9378_FU42_CH_VOL_CH2_LSB:
	case WCD9378_SU43_SELECTOR:
	case WCD9378_SU45_SELECTOR:
	case WCD9378_PDE47_REQ_PS:
	case WCD9378_GE35_SEL_MODE:
	case WCD9378_GE35_DET_MODE:
	case WCD9378_IT31_MICB:
	case WCD9378_IT31_USAGE:
	case WCD9378_PDE34_REQ_PS:
	case WCD9378_SU45_TX_SELECTOR:
	case WCD9378_XU36_BYP:
	case WCD9378_PDE36_REQ_PS:
	case WCD9378_OT36_USAGE:
	case WCD9378_IT11_MICB:
	case WCD9378_IT11_USAGE:
	case WCD9378_PDE11_REQ_PS:
	case WCD9378_OT10_USAGE:
	case WCD9378_SMP_MIC_CTRL1_IT11_MICB:
	case WCD9378_SMP_MIC_CTRL1_IT11_USAGE:
	case WCD9378_SMP_MIC_CTRL1_PDE11_REQ_PS:
	case WCD9378_SMP_MIC_CTRL2_IT11_MICB:
	case WCD9378_SMP_MIC_CTRL2_IT11_USAGE:
	case WCD9378_SMP_MIC_CTRL2_PDE11_REQ_PS:
	case WCD9378_REPORT_ID:
	case WCD9378_MESSAGE0 ... WCD9378_MESSAGE2:
		return true;
	}
	return false;
}

static bool wcd9378_readable_register(struct device *dev, unsigned int reg)
{
	/* Volatile/status registers that are read-only */
	switch (reg) {
	case SWRS_SCP_SDCA_INTSTAT_1:
	case SWRS_SCP_SDCA_INTSTAT_2:
	case SWRS_SCP_SDCA_INTSTAT_3:
	case WCD9378_ANA_MBHC_RESULT_1:
	case WCD9378_ANA_MBHC_RESULT_2:
	case WCD9378_ANA_MBHC_RESULT_3:
	case WCD9378_MBHC_MOISTURE_DET_FSM_STATUS:
	case WCD9378_TX_1_2_SAR2_ERR:
	case WCD9378_TX_1_2_SAR1_ERR:
	case WCD9378_TX_3_SAR4_ERR:
	case WCD9378_TX_3_SAR3_ERR:
	case WCD9378_HPH_L_STATUS:
	case WCD9378_HPH_R_STATUS:
	case WCD9378_HPH_SURGE_HPHLR_SURGE_STATUS:
	case WCD9378_EAR_STATUS_REG_1:
	case WCD9378_EAR_STATUS_REG_2:
	case WCD9378_MBHC_NEW_FSM_STATUS:
	case WCD9378_MBHC_NEW_ADC_RESULT:
	case WCD9378_DIE_CRACK_DIE_CRK_DET_OUT:
	case WCD9378_AUX_INT_STATUS_REG:
	case WCD9378_PDE22_ACT_PS:
	case WCD9378_SAPU29_PROT_MODE:
	case WCD9378_SAPU29_PROT_STAT:
	case WCD9378_PDE23_ACT_PS:
	case WCD9378_SMP_JACK_FUNC_STAT:
	case WCD9378_SMP_JACK_FUNC_ACT:
	case WCD9378_PDE42_ACT_PS:
	case WCD9378_PDE47_ACT_PS:
	case WCD9378_PDE34_ACT_PS:
	case WCD9378_PDE36_ACT_PS:
	case WCD9378_SMP_MIC_CTRL0_FUNC_STAT:
	case WCD9378_SMP_MIC_CTRL0_FUNC_ACT:
	case WCD9378_PDE11_ACT_PS:
	case WCD9378_SMP_MIC_CTRL1_FUNC_STAT:
	case WCD9378_SMP_MIC_CTRL1_FUNC_ACT:
	case WCD9378_SMP_MIC_CTRL1_PDE11_ACT_PS:
	case WCD9378_SMP_MIC_CTRL2_FUNC_STAT:
	case WCD9378_SMP_MIC_CTRL2_FUNC_ACT:
	case WCD9378_SMP_MIC_CTRL2_PDE11_ACT_PS:
		return true;
	}
	return wcd9378_rdwr_register(dev, reg);
}

static bool wcd9378_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SWRS_SCP_SDCA_INTSTAT_1:
	case SWRS_SCP_SDCA_INTSTAT_2:
	case SWRS_SCP_SDCA_INTSTAT_3:
	case WCD9378_ANA_MBHC_RESULT_1:
	case WCD9378_ANA_MBHC_RESULT_2:
	case WCD9378_ANA_MBHC_RESULT_3:
	case WCD9378_MBHC_MOISTURE_DET_FSM_STATUS:
	case WCD9378_TX_1_2_SAR2_ERR:
	case WCD9378_TX_1_2_SAR1_ERR:
	case WCD9378_TX_3_SAR4_ERR:
	case WCD9378_TX_3_SAR3_ERR:
	case WCD9378_HPH_L_STATUS:
	case WCD9378_HPH_R_STATUS:
	case WCD9378_HPH_SURGE_HPHLR_SURGE_STATUS:
	case WCD9378_EAR_STATUS_REG_1:
	case WCD9378_EAR_STATUS_REG_2:
	case WCD9378_MBHC_NEW_FSM_STATUS:
	case WCD9378_MBHC_NEW_ADC_RESULT:
	case WCD9378_DIE_CRACK_DIE_CRK_DET_OUT:
	case WCD9378_AUX_INT_STATUS_REG:
	case WCD9378_PDE22_ACT_PS:
	case WCD9378_SAPU29_PROT_STAT:
	case WCD9378_PDE23_ACT_PS:
	case WCD9378_PDE42_ACT_PS:
	case WCD9378_PDE47_ACT_PS:
	case WCD9378_PDE34_ACT_PS:
	case WCD9378_PDE36_ACT_PS:
	case WCD9378_PDE11_ACT_PS:
	case WCD9378_SMP_MIC_CTRL1_PDE11_ACT_PS:
	case WCD9378_SMP_MIC_CTRL2_PDE11_ACT_PS:
		return true;
	}
	return false;
}

static const struct regmap_config wcd9378_regmap_config = {
	.name = "wcd9378_sdca",
	.reg_bits = 32,
	.val_bits = 8,
	.cache_type = REGCACHE_MAPLE,
	.reg_defaults = wcd9378_defaults,
	.num_reg_defaults = ARRAY_SIZE(wcd9378_defaults),
	.max_register = WCD9378_MAX_REGISTER,
	.readable_reg = wcd9378_readable_register,
	.writeable_reg = wcd9378_rdwr_register,
	.volatile_reg = wcd9378_volatile_register,
	.reg_format_endian = REGMAP_ENDIAN_NATIVE,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.can_multi_write = true,
	.use_single_read = true,
};

static const struct sdw_slave_ops wcd9378_slave_ops = {
	.update_status = wcd_update_status,
	.interrupt_callback = wcd9378_interrupt_callback,
};

static int wcd9378_sdw_probe(struct sdw_slave *pdev,
			     const struct sdw_device_id *id)
{
	struct device *dev = &pdev->dev;
	struct wcd9378_sdw_priv *wcd;
	u8 master_ch_mask[WCD9378_MAX_SWR_CH_IDS];
	int master_ch_mask_size = 0;
	int ret, i;

	wcd = devm_kzalloc(dev, sizeof(*wcd), GFP_KERNEL);
	if (!wcd)
		return -ENOMEM;

	/*
	 * Port map index starts at 0; data ports start at index 1.
	 * Read TX or RX port mapping from DT.
	 */
	if (of_property_present(dev->of_node, "qcom,tx-port-mapping")) {
		wcd->is_tx = true;
		ret = of_property_read_u32_array(dev->of_node,
						 "qcom,tx-port-mapping",
						 &pdev->m_port_map[1],
						 WCD9378_MAX_TX_SWR_PORTS);
	} else {
		ret = of_property_read_u32_array(dev->of_node,
						 "qcom,rx-port-mapping",
						 &pdev->m_port_map[1],
						 WCD9378_MAX_SWR_PORTS);
	}
	if (ret < 0)
		dev_info(dev, "Error getting static port mapping for %s (%d)\n",
			 wcd->is_tx ? "TX" : "RX", ret);

	wcd->sdev = pdev;
	dev_set_drvdata(dev, wcd);

	pdev->prop.scp_int1_mask = SDW_SCP_INT1_IMPL_DEF |
				   SDW_SCP_INT1_BUS_CLASH |
				   SDW_SCP_INT1_PARITY;
	pdev->prop.lane_control_support = true;
	pdev->prop.simple_clk_stop_capable = true;

	memset(master_ch_mask, 0, sizeof(master_ch_mask));

	if (wcd->is_tx) {
		master_ch_mask_size =
			of_property_count_u8_elems(dev->of_node,
						   "qcom,tx-channel-mapping");
		if (master_ch_mask_size > 0)
			ret = of_property_read_u8_array(dev->of_node,
							"qcom,tx-channel-mapping",
							master_ch_mask,
							master_ch_mask_size);
	} else {
		master_ch_mask_size =
			of_property_count_u8_elems(dev->of_node,
						   "qcom,rx-channel-mapping");
		if (master_ch_mask_size > 0)
			ret = of_property_read_u8_array(dev->of_node,
							"qcom,rx-channel-mapping",
							master_ch_mask,
							master_ch_mask_size);
	}

	if (ret < 0)
		dev_info(dev, "Static channel mapping not specified\n");

	if (wcd->is_tx) {
		pdev->prop.source_ports = GENMASK(WCD9378_MAX_TX_SWR_PORTS, 0);
		pdev->prop.src_dpn_prop = wcd9378_dpn_prop;
		wcd->ch_info = &wcd9378_sdw_tx_ch_info[0];

		for (i = 0; i < master_ch_mask_size; i++)
			wcd->ch_info[i].master_ch_mask =
				WCD9378_SWRM_CH_MASK(master_ch_mask[i]);

		pdev->prop.wake_capable = true;

		wcd->regmap = devm_regmap_init_sdw(pdev, &wcd9378_regmap_config);
		if (IS_ERR(wcd->regmap))
			return dev_err_probe(dev, PTR_ERR(wcd->regmap),
					     "Regmap init failed\n");

		/* Start in cache-only until device is enumerated */
		regcache_cache_only(wcd->regmap, true);
	} else {
		pdev->prop.sink_ports = GENMASK(WCD9378_MAX_SWR_PORTS - 1, 0);
		pdev->prop.sink_dpn_prop = wcd9378_dpn_prop;
		wcd->ch_info = &wcd9378_sdw_rx_ch_info[0];

		for (i = 0; i < master_ch_mask_size; i++)
			wcd->ch_info[i].master_ch_mask =
				WCD9378_SWRM_CH_MASK(master_ch_mask[i]);
	}

	ret = component_add(dev, &wcd_sdw_component_ops);
	if (ret)
		return ret;

	/* Set suspended until aggregate device is bound */
	pm_runtime_set_suspended(dev);

	return 0;
}

static void wcd9378_sdw_remove(struct sdw_slave *pdev)
{
	component_del(&pdev->dev, &wcd_sdw_component_ops);
}

static const struct sdw_device_id wcd9378_slave_id[] = {
	SDW_SLAVE_ENTRY(0x0217, 0x10a, 0), /* WCD9378 RX/TX Device ID */
	{ },
};
MODULE_DEVICE_TABLE(sdw, wcd9378_slave_id);

static int wcd9378_sdw_runtime_suspend(struct device *dev)
{
	struct wcd9378_sdw_priv *wcd = dev_get_drvdata(dev);

	if (wcd->regmap) {
		regcache_cache_only(wcd->regmap, true);
		regcache_mark_dirty(wcd->regmap);
	}

	return 0;
}

static int wcd9378_sdw_runtime_resume(struct device *dev)
{
	struct wcd9378_sdw_priv *wcd = dev_get_drvdata(dev);

	if (wcd->regmap) {
		regcache_cache_only(wcd->regmap, false);
		regcache_sync(wcd->regmap);
	}

	return 0;
}

static const struct dev_pm_ops wcd9378_sdw_pm_ops = {
	RUNTIME_PM_OPS(wcd9378_sdw_runtime_suspend,
		       wcd9378_sdw_runtime_resume, NULL)
};

static struct sdw_driver wcd9378_codec_driver = {
	.probe = wcd9378_sdw_probe,
	.remove = wcd9378_sdw_remove,
	.ops = &wcd9378_slave_ops,
	.id_table = wcd9378_slave_id,
	.driver = {
		.name = "wcd9378-codec",
		.pm = pm_ptr(&wcd9378_sdw_pm_ops),
	},
};
module_sdw_driver(wcd9378_codec_driver);

MODULE_DESCRIPTION("WCD9378 SoundWire codec driver");
MODULE_LICENSE("GPL");

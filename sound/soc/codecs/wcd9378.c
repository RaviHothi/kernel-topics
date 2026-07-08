// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.

#include <linux/component.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/soundwire/sdw_registers.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/pcm.h>
#include <sound/soc-dapm.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "wcd-clsh-v2.h"
#include "wcd-common.h"
#include "wcd-mbhc-v2.h"
#include "wcd9378.h"

/* Z value defined in milliohm */
#define WCD9378_ZDET_VAL_32		(32000)
#define WCD9378_ZDET_VAL_400		(400000)
#define WCD9378_ZDET_VAL_1200		(1200000)
#define WCD9378_ZDET_VAL_100K		(100000000)
/* Z floating defined in ohms */
#define WCD9378_ZDET_FLOATING_IMPEDANCE	(0x0FFFFFFE)
#define WCD9378_ZDET_NUM_MEASUREMENTS	(900)
#define WCD9378_MBHC_GET_C1(c)		(((c) & 0xC000) >> 14)
#define WCD9378_MBHC_GET_X1(x)		((x) & 0x3FFF)
/* Z value compared in milliOhm */
#define WCD9378_MBHC_IS_SECOND_RAMP_REQUIRED(z) \
				(((z) > WCD9378_ZDET_VAL_400) || ((z) < WCD9378_ZDET_VAL_32))
#define WCD9378_MBHC_ZDET_CONST		(86 * 16384)
#define WCD9378_MBHC_MOISTURE_RREF	R_24_KOHM
#define WCD_MBHC_HS_V_MAX		1600
#define WCD9378_MBHC_MAX_BUTTONS	8
#define WCD9378_SDW_COMMIT_VALUE	0x02

#define WCD9378_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
		       SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |\
		       SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000 |\
		       SNDRV_PCM_RATE_384000)

/* Fractional Rates */
#define WCD9378_FRAC_RATES (SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_88200 |\
			    SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800)

#define WCD9378_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |\
			 SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE)

enum {
	ALLOW_BUCK_DISABLE,
	HPH_COMP_DELAY,
	HPH_PA_DELAY,
	AMIC2_BCS_ENABLE,
	HPH_SEQ_TX_PM,
};

enum {
	AIF1_PB = 0,
	AIF1_CAP,
	NUM_CODEC_DAIS,
};

struct wcd9378_mbhc_zdet_param {
	u16 ldo_ctl;
	u16 noff;
	u16 nshift;
	u16 btn5;
	u16 btn6;
	u16 btn7;
};

struct wcd9378_priv {
	struct sdw_slave *tx_sdw_dev;
	struct wcd9378_sdw_priv *sdw_priv[NUM_CODEC_DAIS];
	struct device *txdev;
	struct device *rxdev;
	struct device_node *rxnode;
	struct device_node *txnode;
	struct regmap *regmap;
	/* micb setup lock */
	struct mutex micb_lock;
	/* mbhc module */
	struct wcd_mbhc *wcd_mbhc;
	struct wcd_mbhc_config mbhc_cfg;
	struct wcd_mbhc_intr intr_ids;
	struct wcd_clsh_ctrl *clsh_info;
	struct wcd_common common;
	struct irq_domain *virq;
	struct regmap_irq_chip_data *irq_chip;
	struct snd_soc_jack *jack;
	unsigned long status_mask;
	s32 micb_ref[WCD9378_MAX_MICBIAS];
	s32 pullup_ref[WCD9378_MAX_MICBIAS];
	u32 hph_mode;
	int hphr_pdm_wd_int;
	int hphl_pdm_wd_int;
	int aux_pdm_wd_int;
	bool comp1_enable;
	bool comp2_enable;
	struct gpio_desc *us_euro_gpio;
	struct gpio_desc *reset_gpio;
	atomic_t rx_clk_cnt;
	atomic_t ana_clk_count;
};

static const char * const wcd9378_supplies[] = {
	"vdd-rxtx", "vdd-px", "vdd-mic-bias", "vdd-buck",
};

static const SNDRV_CTL_TLVD_DECLARE_DB_MINMAX(ear_pa_gain, 600, -1800);
static const DECLARE_TLV_DB_SCALE(line_gain, 0, 7, 1);
static const DECLARE_TLV_DB_SCALE(analog_gain, 0, 25, 1);

/* ------------------------------------------------------------------ */
/* MBHC field table — maps MBHC register fields to WCD9378 registers  */
/* ------------------------------------------------------------------ */
static const struct wcd_mbhc_field wcd_mbhc_fields[WCD_MBHC_REG_FUNC_MAX] = {
	WCD_MBHC_FIELD(WCD_MBHC_L_DET_EN,
		       WCD9378_ANA_MBHC_MECH, 0x80),
	WCD_MBHC_FIELD(WCD_MBHC_GND_DET_EN,
		       WCD9378_ANA_MBHC_MECH, 0x40),
	WCD_MBHC_FIELD(WCD_MBHC_MECH_DETECTION_TYPE,
		       WCD9378_ANA_MBHC_MECH, 0x20),
	WCD_MBHC_FIELD(WCD_MBHC_MIC_CLAMP_CTL,
		       WCD9378_MBHC_NEW_PLUG_DETECT_CTL, 0x30),
	WCD_MBHC_FIELD(WCD_MBHC_ELECT_DETECTION_TYPE,
		       WCD9378_ANA_MBHC_ELECT, 0x08),
	WCD_MBHC_FIELD(WCD_MBHC_HS_L_DET_PULL_UP_CTRL,
		       WCD9378_MBHC_NEW_INT_MECH_DET_CURRENT, 0x1F),
	WCD_MBHC_FIELD(WCD_MBHC_HS_L_DET_PULL_UP_COMP_CTRL,
		       WCD9378_ANA_MBHC_MECH, 0x04),
	WCD_MBHC_FIELD(WCD_MBHC_HPHL_PLUG_TYPE,
		       WCD9378_ANA_MBHC_MECH, 0x10),
	WCD_MBHC_FIELD(WCD_MBHC_GND_PLUG_TYPE,
		       WCD9378_ANA_MBHC_MECH, 0x08),
	WCD_MBHC_FIELD(WCD_MBHC_SW_HPH_LP_100K_TO_GND,
		       WCD9378_ANA_MBHC_MECH, 0x01),
	WCD_MBHC_FIELD(WCD_MBHC_ELECT_SCHMT_ISRC,
		       WCD9378_ANA_MBHC_ELECT, 0x06),
	WCD_MBHC_FIELD(WCD_MBHC_FSM_EN,
		       WCD9378_ANA_MBHC_ELECT, 0x80),
	WCD_MBHC_FIELD(WCD_MBHC_INSREM_DBNC,
		       WCD9378_MBHC_NEW_PLUG_DETECT_CTL, 0x0F),
	WCD_MBHC_FIELD(WCD_MBHC_BTN_DBNC,
		       WCD9378_MBHC_NEW_CTL_1, 0x03),
	WCD_MBHC_FIELD(WCD_MBHC_HS_VREF,
		       WCD9378_MBHC_NEW_CTL_2, 0x03),
	WCD_MBHC_FIELD(WCD_MBHC_HS_COMP_RESULT,
		       WCD9378_ANA_MBHC_RESULT_3, 0x08),
	WCD_MBHC_FIELD(WCD_MBHC_IN2P_CLAMP_STATE,
		       WCD9378_ANA_MBHC_RESULT_3, 0x10),
	WCD_MBHC_FIELD(WCD_MBHC_MIC_SCHMT_RESULT,
		       WCD9378_ANA_MBHC_RESULT_3, 0x20),
	WCD_MBHC_FIELD(WCD_MBHC_HPHL_SCHMT_RESULT,
		       WCD9378_ANA_MBHC_RESULT_3, 0x80),
	WCD_MBHC_FIELD(WCD_MBHC_HPHR_SCHMT_RESULT,
		       WCD9378_ANA_MBHC_RESULT_3, 0x40),
	WCD_MBHC_FIELD(WCD_MBHC_OCP_FSM_EN,
		       WCD9378_HPH_OCP_CTL, 0x10),
	WCD_MBHC_FIELD(WCD_MBHC_BTN_RESULT,
		       WCD9378_ANA_MBHC_RESULT_3, 0x07),
	WCD_MBHC_FIELD(WCD_MBHC_BTN_ISRC_CTL,
		       WCD9378_ANA_MBHC_ELECT, 0x70),
	WCD_MBHC_FIELD(WCD_MBHC_ELECT_RESULT,
		       WCD9378_ANA_MBHC_RESULT_3, 0xFF),
	WCD_MBHC_FIELD(WCD_MBHC_MICB_CTRL,
		       WCD9378_ANA_MICB2, 0xC0),
	WCD_MBHC_FIELD(WCD_MBHC_HPH_CNP_WG_TIME,
		       WCD9378_HPH_CNP_WG_TIME, 0xFF),
	WCD_MBHC_FIELD(WCD_MBHC_HPHR_PA_EN,
		       WCD9378_ANA_HPH, 0x40),
	WCD_MBHC_FIELD(WCD_MBHC_HPHL_PA_EN,
		       WCD9378_ANA_HPH, 0x80),
	WCD_MBHC_FIELD(WCD_MBHC_HPH_PA_EN,
		       WCD9378_ANA_HPH, 0xC0),
	WCD_MBHC_FIELD(WCD_MBHC_SWCH_LEVEL_REMOVE,
		       WCD9378_ANA_MBHC_RESULT_3, 0x10),
	WCD_MBHC_FIELD(WCD_MBHC_ANC_DET_EN,
		       WCD9378_MBHC_CTL_BCS, 0x02),
	WCD_MBHC_FIELD(WCD_MBHC_FSM_STATUS,
		       WCD9378_MBHC_NEW_FSM_STATUS, 0x01),
	WCD_MBHC_FIELD(WCD_MBHC_MUX_CTL,
		       WCD9378_MBHC_NEW_CTL_2, 0x70),
	WCD_MBHC_FIELD(WCD_MBHC_MOISTURE_STATUS,
		       WCD9378_MBHC_NEW_FSM_STATUS, 0x20),
	WCD_MBHC_FIELD(WCD_MBHC_HPHR_GND,
		       WCD9378_HPH_PA_CTL2, 0x40),
	WCD_MBHC_FIELD(WCD_MBHC_HPHL_GND,
		       WCD9378_HPH_PA_CTL2, 0x10),
	WCD_MBHC_FIELD(WCD_MBHC_HPHL_OCP_DET_EN,
		       WCD9378_HPH_L_TEST, 0x01),
	WCD_MBHC_FIELD(WCD_MBHC_HPHR_OCP_DET_EN,
		       WCD9378_HPH_R_TEST, 0x01),
	WCD_MBHC_FIELD(WCD_MBHC_HPHL_OCP_STATUS,
		       SWRS_SCP_SDCA_INTSTAT_1, 0x80),
	WCD_MBHC_FIELD(WCD_MBHC_HPHR_OCP_STATUS,
		       SWRS_SCP_SDCA_INTSTAT_1, 0x20),
	WCD_MBHC_FIELD(WCD_MBHC_ADC_EN,
		       WCD9378_MBHC_NEW_CTL_1, 0x08),
	WCD_MBHC_FIELD(WCD_MBHC_ADC_COMPLETE,
		       WCD9378_MBHC_NEW_FSM_STATUS, 0x40),
	WCD_MBHC_FIELD(WCD_MBHC_ADC_TIMEOUT,
		       WCD9378_MBHC_NEW_FSM_STATUS, 0x80),
	WCD_MBHC_FIELD(WCD_MBHC_ADC_RESULT,
		       WCD9378_MBHC_NEW_ADC_RESULT, 0xFF),
	WCD_MBHC_FIELD(WCD_MBHC_MICB2_VOUT,
		       WCD9378_ANA_MICB2, 0x3F),
	WCD_MBHC_FIELD(WCD_MBHC_ADC_MODE,
		       WCD9378_MBHC_NEW_CTL_1, 0x10),
	WCD_MBHC_FIELD(WCD_MBHC_DETECTION_DONE,
		       WCD9378_MBHC_NEW_CTL_1, 0x04),
	WCD_MBHC_FIELD(WCD_MBHC_ELECT_ISRC_EN,
		       WCD9378_ANA_MBHC_ZDET, 0x02),
};

/* ------------------------------------------------------------------ */
/* IRQ chip — SDCA interrupt status registers                          */
/* ------------------------------------------------------------------ */
static const struct regmap_irq wcd9378_irqs[WCD9378_NUM_IRQS] = {
	REGMAP_IRQ_REG(WCD9378_IRQ_MBHC_BUTTON_PRESS_DET,    0, BIT(0)),
	REGMAP_IRQ_REG(WCD9378_IRQ_MBHC_BUTTON_RELEASE_DET,  0, BIT(1)),
	REGMAP_IRQ_REG(WCD9378_IRQ_MBHC_ELECT_INS_REM_DET,   0, BIT(2)),
	REGMAP_IRQ_REG(WCD9378_IRQ_MBHC_ELECT_INS_REM_LEG_DET, 0, BIT(3)),
	REGMAP_IRQ_REG(WCD9378_IRQ_MBHC_SW_DET,              0, BIT(4)),
	REGMAP_IRQ_REG(WCD9378_IRQ_HPHR_OCP_INT,             0, BIT(5)),
	REGMAP_IRQ_REG(WCD9378_IRQ_HPHR_CNP_INT,             0, BIT(6)),
	REGMAP_IRQ_REG(WCD9378_IRQ_HPHL_OCP_INT,             0, BIT(7)),
	REGMAP_IRQ_REG(WCD9378_IRQ_HPHL_CNP_INT,             1, BIT(0)),
	REGMAP_IRQ_REG(WCD9378_IRQ_EAR_CNP_INT,              1, BIT(1)),
	REGMAP_IRQ_REG(WCD9378_IRQ_EAR_SCD_INT,              1, BIT(2)),
	REGMAP_IRQ_REG(WCD9378_IRQ_AUX_CNP_INT,              1, BIT(3)),
	REGMAP_IRQ_REG(WCD9378_IRQ_AUX_SCD_INT,              1, BIT(4)),
	REGMAP_IRQ_REG(WCD9378_IRQ_HPHL_PDM_WD_INT,          1, BIT(5)),
	REGMAP_IRQ_REG(WCD9378_IRQ_HPHR_PDM_WD_INT,          1, BIT(6)),
	REGMAP_IRQ_REG(WCD9378_IRQ_AUX_PDM_WD_INT,           1, BIT(7)),
	REGMAP_IRQ_REG(WCD9378_IRQ_LDORT_SCD_INT,            2, BIT(0)),
	REGMAP_IRQ_REG(WCD9378_IRQ_MBHC_MOISTURE_INT,        2, BIT(1)),
	REGMAP_IRQ_REG(WCD9378_IRQ_HPHL_SURGE_DET_INT,       2, BIT(2)),
	REGMAP_IRQ_REG(WCD9378_IRQ_HPHR_SURGE_DET_INT,       2, BIT(3)),
	REGMAP_IRQ_REG(WCD9378_IRQ_SAPU_PROT_MODE_CHG,       2, BIT(6)),
};

static int wcd9378_handle_post_irq(void *data)
{
	struct wcd9378_priv *wcd9378;

	if (data)
		wcd9378 = (struct wcd9378_priv *)data;
	else
		return IRQ_HANDLED;

	/* Clear SDCA interrupt status registers */
	regmap_write(wcd9378->regmap, SWRS_SCP_SDCA_INTSTAT_1, 0xff);
	regmap_write(wcd9378->regmap, SWRS_SCP_SDCA_INTSTAT_2, 0xff);
	regmap_write(wcd9378->regmap, SWRS_SCP_SDCA_INTSTAT_3, 0xff);

	return IRQ_HANDLED;
}

static const struct regmap_irq_chip wcd9378_regmap_irq_chip = {
	.name = "wcd9378",
	.irqs = wcd9378_irqs,
	.num_irqs = ARRAY_SIZE(wcd9378_irqs),
	.num_regs = 3,
	.status_base = SWRS_SCP_SDCA_INTSTAT_1,
	.mask_base = SWRS_SCP_SDCA_INTMASK_1,
	.ack_base = SWRS_SCP_SDCA_INTSTAT_1,
	.use_ack = 1,
	.clear_ack = 1,
	.runtime_pm = true,
	.handle_post_irq = wcd9378_handle_post_irq,
	.irq_drv_data = NULL,
};

/* ------------------------------------------------------------------ */
/* Reset / IO init                                                     */
/* ------------------------------------------------------------------ */
static void wcd9378_reset(struct wcd9378_priv *wcd9378)
{
	gpiod_set_value(wcd9378->reset_gpio, 1);
	usleep_range(20, 30);
	gpiod_set_value(wcd9378->reset_gpio, 0);
	usleep_range(20, 30);
}

static void wcd9378_class_load(struct regmap *regmap)
{
	/* SMP_AMP class loading */
	regmap_update_bits(regmap, WCD9378_FUNC_ACT, 0x01, 0x01);
	usleep_range(20000, 20010);
	regmap_update_bits(regmap, WCD9378_SMP_AMP_FUNC_STAT, 0xFF, 0xFF);

	/* SMP_JACK class loading */
	regmap_update_bits(regmap, WCD9378_SMP_JACK_FUNC_ACT, 0x01, 0x01);
	usleep_range(30000, 30010);
	regmap_update_bits(regmap, WCD9378_CMT_GRP_MASK_REG, 0x02, 0x02);
	regmap_update_bits(regmap, WCD9378_SMP_JACK_FUNC_STAT, 0xFF, 0xFF);
}

static void wcd9378_io_init(struct regmap *regmap)
{
	/*
	 * Ensure the chip is not in software reset state.
	 * CDC_RST_CTL bits: ANA_SW_RST_N (bit1) and DIG_SW_RST_N (bit0)
	 * Both must be 1 (active-low reset, so 1 = not in reset).
	 * Hardware reset value may be 0x00 (both in reset), which would
	 * prevent the SWR interface from functioning correctly.
	 */
	regmap_write(regmap, WCD9378_CDC_RST_CTL, WCD9378_CDC_RST_CTL_NO_RESET);

	/*
	 * WCD9378 uses a hardware-driven power sequencer.
	 * Software enables bandgap and bias only; the HW sequencer
	 * handles Class-G CP bring-up — no explicit SW sequences needed.
	 */
	regmap_update_bits(regmap, WCD9378_SLEEP_CTL, 0x0e, 0x0e);
	regmap_update_bits(regmap, WCD9378_SLEEP_CTL, 0x80, 0x80);
	usleep_range(1000, 1010);
	regmap_update_bits(regmap, WCD9378_SLEEP_CTL, 0x40, 0x40);
	usleep_range(1000, 1010);

	regmap_update_bits(regmap, WCD9378_BIAS_VBG_FINE_ADJ, 0xf0, BIT(7));
	regmap_update_bits(regmap, WCD9378_ANA_BIAS, BIT(7), BIT(7));
	regmap_update_bits(regmap, WCD9378_ANA_BIAS, BIT(6), BIT(6));
	usleep_range(10000, 10010);
	regmap_update_bits(regmap, WCD9378_ANA_BIAS, BIT(6), 0x00);

	/* TX analog clock enable (ANA_TXSCBIAS_CLK_EN) */
	regmap_update_bits(regmap, WCD9378_CDC_ANA_TX_CLK_CTL, 0x01, 0x01);

	/* TX frontend sequencer bypass */
	regmap_update_bits(regmap, WCD9378_TX_COM_TXFE_DIV_CTL, 0x80, 0x80);

	/* PDM watchdog timeout: 160 cycles */
	regmap_update_bits(regmap, WCD9378_PDM_WD_CTL0, 0x10, 0x10);
	regmap_update_bits(regmap, WCD9378_PDM_WD_CTL1, 0x10, 0x10);

	regmap_update_bits(regmap, WCD9378_MICB1_TEST_CTL_1, 0xff, 0xfa);
	regmap_update_bits(regmap, WCD9378_MICB2_TEST_CTL_1, 0xff, 0xfa);
	regmap_update_bits(regmap, WCD9378_MICB3_TEST_CTL_1, 0xff, 0xfa);
	regmap_update_bits(regmap, WCD9378_MICB1_TEST_CTL_2, 0x38, 0x00);
	regmap_update_bits(regmap, WCD9378_MICB2_TEST_CTL_2, 0x38, 0x00);
	regmap_update_bits(regmap, WCD9378_MICB3_TEST_CTL_2, 0x38, 0x00);

	/* RDAC HD2 tuning for WCD9378 silicon */
	regmap_update_bits(regmap, WCD9378_HPH_NEW_INT_RDAC_HD2_CTL_L, 0x1F, 0x04);
	regmap_update_bits(regmap, WCD9378_HPH_NEW_INT_RDAC_HD2_CTL_R, 0x1F, 0x04);
	regmap_update_bits(regmap, WCD9378_BIAS_VBG_FINE_ADJ, 0xF0, 0xB0);
	regmap_update_bits(regmap, WCD9378_HPH_NEW_INT_RDAC_GAIN_CTL, 0xF0, 0x50);

	/* Class-G CP: disable TWAIT */
	regmap_update_bits(regmap, WCD9378_CP_CP_DTOP_CTRL_9, 0x08, 0x08);

	/*
	 * Restore RDAC clock control registers to their default values.
	 * HPH power sequencer timing and state machine micbias selection.
	 */
	regmap_update_bits(regmap, WCD9378_HPH_UP_T0, 0xff, 0x05);
	regmap_update_bits(regmap, WCD9378_HPH_UP_T9, 0xff, 0x05);
	regmap_update_bits(regmap, WCD9378_HPH_DN_T0, 0xff, 0x06);
	regmap_update_bits(regmap, WCD9378_SM0_MB_SEL, 0xff, 0x01);
	regmap_update_bits(regmap, WCD9378_SM1_MB_SEL, 0xff, 0x02);
	regmap_update_bits(regmap, WCD9378_SM2_MB_SEL, 0xff, 0x03);
	regmap_update_bits(regmap, WCD9378_SYS_USAGE_CTRL, 0x0f, 0x00);

	/*
	 * Set mobile mode (0x01): hardware uses RX SWR clock for valid-config
	 * detection. Without this, hardware defaults to compute mode and checks
	 * TX SWR clock, causing HPH_FUNC_FAULTY and blocking the sequencer.
	 */
	regmap_write(regmap, WCD9378_PLATFORM_CTL, WCD9378_PLATFORM_CTL_MOBILE);

	/*
	 * Disable SWR slave auto-reset on sync loss (SWR_RST_EN = 0x00).
	 *
	 * The hardware default is 0x1f which enables auto-reset for both RX
	 * and TX slaves on: sync loss, SWR bus reset, and SWR register reset.
	 * When the LPASS audio subsystem starts up, a brief SWR sync loss
	 * occurs. With the default 0x1f, this causes BOTH RX and TX slaves to
	 * reset themselves and drop off the bus simultaneously (observed as
	 * mcp_slv=0x0 while link_sts=0x2081 - clock still running).
	 *
	 * Setting SWR_RST_EN=0x00 prevents the slaves from auto-resetting on
	 * sync loss. The slaves will stay attached and re-sync automatically
	 * after the brief glitch, eliminating the need for a chip reset on
	 * every playback start.
	 *
	 * LA downstream uses 0x1f but doesn't have this issue because the LA
	 * SWR framework doesn't trigger the LPASS sync loss event.
	 */
	regmap_write(regmap, WCD9378_SWR_RST_EN, 0x00);

	/*
	 * Set SWR TX clock rate register for both banks to 9.6MHz (val=0).
	 * The hardware valid-config detection circuit reads this register to
	 * determine if the SWR clock rate is valid for the HPH sequencer.
	 * After a chip reset this register reverts to its hardware default;
	 * without an explicit write the sequencer fires HPH_FUNC_FAULTY.
	 * Encoding: nibble[3:0]=bank0, nibble[7:4]=bank1; 0=9.6MHz.
	 */
	regmap_write(regmap, WCD9378_SWR_TX_CLK_RATE, 0x00);

	/*
	 * Override HPH sequencer enable and valid-config signals.
	 * Required in the upstream SWR framework — the valid-config
	 * detection circuit fires HPH_FUNC_FAULTY without these.
	 * Root cause under investigation (LA does not need them).
	 */
	regmap_update_bits(regmap, WCD9378_SEQ_OVRRIDE_CTL0, 0x01, 0x01);
	regmap_update_bits(regmap, WCD9378_SEQ_OVRRIDE_CTL2, 0x04, 0x04);
	regmap_update_bits(regmap, WCD9378_BYP_EN_CTL2, WCD9378_HPH_VALID_CFG_BYP_EN,
			   WCD9378_HPH_VALID_CFG_BYP_EN);

	/*
	 * After a chip reset, WCD9378_HPH_RDAC_CLK_CTL1 bit7 is cleared
	 * (RDAC clock disabled), which silences the HPH output even though
	 * the HPH PA is powered on. Explicitly restore the defaults here.
	 */
	regmap_write(regmap, WCD9378_HPH_RDAC_CLK_CTL1, 0x99);
	regmap_write(regmap, WCD9378_HPH_RDAC_CLK_CTL2, 0x9b);

	/* Load HPH control classes (SMP_AMP + SMP_JACK). */
	wcd9378_class_load(regmap);

	/* Mute HPH channels initially */
	regmap_write(regmap, WCD9378_FU42_MUTE_CH1_CN, 0x01);
	regmap_write(regmap, WCD9378_FU42_MUTE_CH2_CN, 0x01);

	/*
	 * Mask HPHL/R surge detection interrupts (SDCA_INTMASK_3 bits 2,3).
	 * These fire when HPH surge protection is enabled with no HPH load,
	 * setting SDCA_CASCADE in SDW_DP0_INT → continuous ALERT loop.
	 * Unmask when MBHC is re-enabled and surge detection is handled.
	 */
	regmap_write(regmap, SWRS_SCP_SDCA_INTMASK_3, 0x0c);

	/*
	 * Mask all MBHC SDCA interrupts (SDCA_INTMASK_1 = 0xff) while MBHC
	 * is disabled. Without this, MBHC events fire sub-IRQs that have no
	 * handler registered, causing 'irq: nobody cared' warnings and the
	 * kernel disabling the IRQ. Unmask when MBHC is re-enabled.
	 */
	regmap_write(regmap, SWRS_SCP_SDCA_INTMASK_1, 0xff);
}

static u8 wcd9378_get_hph_pwr_level(int hph_mode)
{
	switch (hph_mode) {
	case CLS_H_LOHIFI:
	case CLS_AB_LOHIFI:
		return 0x00;
	case CLS_H_LP:
	case CLS_AB_LP:
		return 0x01;
	case CLS_H_HIFI:
	case CLS_AB_HIFI:
		return 0x02;
	case CLS_H_ULP:
	case CLS_AB:
	case CLS_H_NORMAL:
	default:
		return 0x03;
	}
}

static int wcd9378_pde_act_ps_check(struct snd_soc_component *component,
				    unsigned int reg, u8 expected_ps)
{
	int retries = 40;
	unsigned int val = 0;

	while (retries--) {
		val = snd_soc_component_read(component, reg) & 0xff;
		if (val == expected_ps)
			return 0;

		usleep_range(1000, 1100);
	}

	return -ETIMEDOUT;
}

static void wcd9378_sdw_commit(struct snd_soc_component *component,
			       struct wcd9378_priv *wcd9378)
{
	struct sdw_slave *rx_sdw = NULL;
	int ret;

	if (wcd9378->rxdev)
		rx_sdw = dev_to_sdw_dev(wcd9378->rxdev);

	if (!wcd9378->tx_sdw_dev && !rx_sdw)
		return;

	if (wcd9378->tx_sdw_dev) {
		ret = sdw_write(wcd9378->tx_sdw_dev, SDW_SCP_COMMIT,
				WCD9378_SDW_COMMIT_VALUE);
		if (ret < 0)
			dev_err_ratelimited(component->dev,
					    "SDW commit (tx) failed: %d\n", ret);
	}

	if (rx_sdw) {
		ret = sdw_write(rx_sdw, SDW_SCP_COMMIT, WCD9378_SDW_COMMIT_VALUE);
		if (ret < 0)
			dev_err_ratelimited(component->dev,
					    "SDW commit (rx) failed: %d\n", ret);
	}
}

static int wcd9378_hph_sequencer_enable(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *kcontrol,
					int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);
	int hph_mode = wcd9378->hph_mode;
	int ret = 0;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:

		if (wcd9378->txdev) {
			ret = pm_runtime_resume_and_get(wcd9378->txdev);
			if (ret < 0)
				dev_warn(component->dev,
					 "txdev pm_get failed: %d\n", ret);
			else
				set_bit(HPH_SEQ_TX_PM, &wcd9378->status_mask);
		}

		snd_soc_component_write(component, WCD9378_CMT_GRP_MASK_REG, 0x02);
		/*
		 * SYS_USAGE_CTRL must be set to a value that has RX0_RX1_HPH_EN
		 * (bit 12) set in the sys_usage[] table, otherwise the HPH
		 * sequencer is disabled and PDE47 writes have no effect.
		 * Index 1 (0x12a7) is the minimal HPH-only entry with
		 * RX0_RX1_HPH_EN + TX1_FOR_JACK set.
		 */
		snd_soc_component_update_bits(component, WCD9378_SYS_USAGE_CTRL, 0x0f, WCD9378_SYS_USAGE_HPH_ONLY);

		if (!wcd9378->comp1_enable || !wcd9378->comp2_enable) {
			snd_soc_component_update_bits(component, WCD9378_HPH_UP_T7, 0x07, 0x07);
			snd_soc_component_update_bits(component, WCD9378_HPH_DN_T1, 0x07, 0x07);
		}

		if (hph_mode == CLS_AB || hph_mode == CLS_AB_HIFI ||
		    hph_mode == CLS_AB_LP || hph_mode == CLS_AB_LOHIFI)
			snd_soc_component_update_bits(component, WCD9378_CP_CP_DTOP_CTRL_14,
						      BIT(7), BIT(7));

		snd_soc_component_update_bits(component, WCD9378_IT41_USAGE, 0x03,
					      wcd9378_get_hph_pwr_level(hph_mode));




		snd_soc_component_update_bits(component, WCD9378_PDE47_REQ_PS, 0xff,
					      WCD9378_PDE_PS0);


		if (!wcd9378->comp1_enable || !wcd9378->comp2_enable)
			usleep_range(26000, 26100);
		else
			usleep_range(15000, 15100);

		snd_soc_component_write(component, WCD9378_FU42_MUTE_CH1_CN, 0x00);
		snd_soc_component_write(component, WCD9378_FU42_MUTE_CH2_CN, 0x00);
		wcd9378_sdw_commit(component, wcd9378);

		ret = wcd9378_pde_act_ps_check(component, WCD9378_PDE47_ACT_PS,
					       WCD9378_PDE_PS0);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component, WCD9378_PDE47_REQ_PS, 0xff,
					      WCD9378_PDE_PS3);

		if (!wcd9378->comp1_enable || !wcd9378->comp2_enable)
			usleep_range(30000, 30100);
		else
			usleep_range(17000, 17100);

		ret = wcd9378_pde_act_ps_check(component, WCD9378_PDE47_ACT_PS,
					       WCD9378_PDE_PS3);
		snd_soc_component_write(component, WCD9378_FU42_MUTE_CH1_CN, 0x01);
		snd_soc_component_write(component, WCD9378_FU42_MUTE_CH2_CN, 0x01);
		wcd9378_sdw_commit(component, wcd9378);
		snd_soc_component_update_bits(component, WCD9378_SYS_USAGE_CTRL, 0x0f, 0x00);


		if (wcd9378->txdev &&
		    test_bit(HPH_SEQ_TX_PM, &wcd9378->status_mask)) {
			pm_runtime_put_autosuspend(wcd9378->txdev);
			clear_bit(HPH_SEQ_TX_PM, &wcd9378->status_mask);
		}
		break;
	default:
		break;
	}

	return ret;
}

/* ------------------------------------------------------------------ */
/* RX clock management                                                 */
/* ------------------------------------------------------------------ */
static int wcd9378_rx_clk_enable(struct snd_soc_component *component)
{
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);

	if (atomic_read(&wcd9378->rx_clk_cnt))
		return 0;

	snd_soc_component_update_bits(component,
				      WCD9378_ANA_RX_SUPPLIES, BIT(0), BIT(0));
	atomic_inc(&wcd9378->rx_clk_cnt);

	return 0;
}

static int wcd9378_rx_clk_disable(struct snd_soc_component *component)
{
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);

	if (!atomic_read(&wcd9378->rx_clk_cnt)) {
		dev_err(component->dev, "clk already disabled\n");
		return 0;
	}

	atomic_dec(&wcd9378->rx_clk_cnt);
	snd_soc_component_update_bits(component,
				      WCD9378_ANA_RX_SUPPLIES, BIT(0), 0x00);

	return 0;
}

/* ------------------------------------------------------------------ */
/* Micbias management                                                  */
/* ------------------------------------------------------------------ */
static int wcd9378_micbias_control(struct snd_soc_component *component,
				   int micb_num, int req, bool is_dapm)
{
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);
	unsigned int micb_reg;

	switch (micb_num) {
	case MIC_BIAS_1:
		micb_reg = WCD9378_ANA_MICB1;
		break;
	case MIC_BIAS_2:
		micb_reg = WCD9378_ANA_MICB2;
		break;
	case MIC_BIAS_3:
		micb_reg = WCD9378_ANA_MICB3;
		break;
	default:
		dev_err(component->dev, "Invalid micbias number: %d\n", micb_num);
		return -EINVAL;
	}

	mutex_lock(&wcd9378->micb_lock);

	switch (req) {
	case MICB_PULLUP_ENABLE:
		wcd9378->pullup_ref[micb_num - 1]++;
		if (wcd9378->pullup_ref[micb_num - 1] == 1 &&
		    wcd9378->micb_ref[micb_num - 1] == 0)
			snd_soc_component_update_bits(component, micb_reg,
						      WCD9378_MICB_EN_MASK,
						      WCD9378_MICB_PULL_UP << 6);
		break;
	case MICB_PULLUP_DISABLE:
		if (wcd9378->pullup_ref[micb_num - 1] > 0)
			wcd9378->pullup_ref[micb_num - 1]--;
		if (wcd9378->pullup_ref[micb_num - 1] == 0 &&
		    wcd9378->micb_ref[micb_num - 1] == 0)
			snd_soc_component_update_bits(component, micb_reg,
						      WCD9378_MICB_EN_MASK,
						      WCD9378_MICB_DISABLE << 6);
		break;
	case MICB_ENABLE:
		wcd9378->micb_ref[micb_num - 1]++;
		if (wcd9378->micb_ref[micb_num - 1] == 1)
			snd_soc_component_update_bits(component, micb_reg,
						      WCD9378_MICB_EN_MASK,
						      WCD9378_MICB_ENABLE << 6);
		break;
	case MICB_DISABLE:
		if (wcd9378->micb_ref[micb_num - 1] > 0)
			wcd9378->micb_ref[micb_num - 1]--;
		if (wcd9378->micb_ref[micb_num - 1] == 0 &&
		    wcd9378->pullup_ref[micb_num - 1] > 0)
			snd_soc_component_update_bits(component, micb_reg,
						      WCD9378_MICB_EN_MASK,
						      WCD9378_MICB_PULL_UP << 6);
		else if (wcd9378->micb_ref[micb_num - 1] == 0 &&
			 wcd9378->pullup_ref[micb_num - 1] == 0)
			snd_soc_component_update_bits(component, micb_reg,
						      WCD9378_MICB_EN_MASK,
						      WCD9378_MICB_DISABLE << 6);
		break;
	default:
		dev_err(component->dev, "Invalid micbias request: %d\n", req);
		mutex_unlock(&wcd9378->micb_lock);
		return -EINVAL;
	}

	mutex_unlock(&wcd9378->micb_lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* DAPM event callbacks — RX path                                      */
/* ------------------------------------------------------------------ */
static int wcd9378_codec_hphl_dac_event(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *kcontrol,
					int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);
	int hph_mode = wcd9378->hph_mode;
	int ret;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (wcd9378->txdev) {
			ret = pm_runtime_resume_and_get(wcd9378->txdev);
		}
		wcd9378_rx_clk_enable(component);
		snd_soc_component_update_bits(component,
					      WCD9378_HPH_RDAC_CLK_CTL1,
					      BIT(7), 0x00);
		set_bit(HPH_COMP_DELAY, &wcd9378->status_mask);
		/* Enable HPHL RX digital path */
		snd_soc_component_update_bits(component,
					      WCD9378_CDC_HPH_GAIN_CTL,
					      WCD9378_CDC_HPH_GAIN_CTL_HPHL_RX_EN,
					      WCD9378_CDC_HPH_GAIN_CTL_HPHL_RX_EN);
		break;
	case SND_SOC_DAPM_POST_PMU:
		if (hph_mode == CLS_AB_HIFI || hph_mode == CLS_H_HIFI)
			snd_soc_component_update_bits(component,
						      WCD9378_HPH_NEW_INT_RDAC_HD2_CTL_L,
						      0x0f, BIT(1));
		else if (hph_mode == CLS_H_LOHIFI)
			snd_soc_component_update_bits(component,
						      WCD9378_HPH_NEW_INT_RDAC_HD2_CTL_L,
						      0x0f, 0x06);
		if (wcd9378->comp1_enable) {
			if (test_bit(HPH_COMP_DELAY, &wcd9378->status_mask)) {
				usleep_range(5000, 5110);
				clear_bit(HPH_COMP_DELAY, &wcd9378->status_mask);
			}
		}
		snd_soc_component_update_bits(component,
					      WCD9378_HPH_NEW_INT_HPH_TIMER1,
					      BIT(1), 0x00);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component,
					      WCD9378_HPH_NEW_INT_RDAC_HD2_CTL_L,
					      0x0f, BIT(0));
		snd_soc_component_update_bits(component,
					      WCD9378_CDC_HPH_GAIN_CTL,
					      WCD9378_CDC_HPH_GAIN_CTL_HPHL_RX_EN, 0);
		if (wcd9378->txdev)
			pm_runtime_put_autosuspend(wcd9378->txdev);
		break;
	}

	return 0;
}

static int wcd9378_codec_hphr_dac_event(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *kcontrol,
					int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);
	int hph_mode = wcd9378->hph_mode;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		wcd9378_rx_clk_enable(component);
		snd_soc_component_update_bits(component,
					      WCD9378_HPH_RDAC_CLK_CTL1,
					      BIT(7), 0x00);
		set_bit(HPH_COMP_DELAY, &wcd9378->status_mask);
		/* Enable HPHR RX digital path */
		snd_soc_component_update_bits(component,
					      WCD9378_CDC_HPH_GAIN_CTL,
					      WCD9378_CDC_HPH_GAIN_CTL_HPHR_RX_EN,
					      WCD9378_CDC_HPH_GAIN_CTL_HPHR_RX_EN);
		break;
	case SND_SOC_DAPM_POST_PMU:
		if (hph_mode == CLS_AB_HIFI || hph_mode == CLS_H_HIFI)
			snd_soc_component_update_bits(component,
						      WCD9378_HPH_NEW_INT_RDAC_HD2_CTL_R,
						      0x0f, BIT(1));
		else if (hph_mode == CLS_H_LOHIFI)
			snd_soc_component_update_bits(component,
						      WCD9378_HPH_NEW_INT_RDAC_HD2_CTL_R,
						      0x0f, 0x06);
		if (wcd9378->comp2_enable) {
			if (test_bit(HPH_COMP_DELAY, &wcd9378->status_mask)) {
				usleep_range(5000, 5110);
				clear_bit(HPH_COMP_DELAY, &wcd9378->status_mask);
			}
		}
		snd_soc_component_update_bits(component,
					      WCD9378_HPH_NEW_INT_HPH_TIMER1,
					      BIT(1), 0x00);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component,
					      WCD9378_HPH_NEW_INT_RDAC_HD2_CTL_R,
					      0x0f, BIT(0));
		snd_soc_component_update_bits(component,
					      WCD9378_CDC_HPH_GAIN_CTL,
					      WCD9378_CDC_HPH_GAIN_CTL_HPHR_RX_EN, 0);
		break;
	}

	return 0;
}

static int wcd9378_codec_ear_dac_event(struct snd_soc_dapm_widget *w,
				       struct snd_kcontrol *kcontrol,
				       int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);
	int hph_mode = wcd9378->hph_mode;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		wcd9378_rx_clk_enable(component);
		if (hph_mode == CLS_AB_HIFI || hph_mode == CLS_H_HIFI)
			snd_soc_component_update_bits(component,
						      WCD9378_HPH_NEW_INT_RDAC_HD2_CTL_L,
						      0x0f, BIT(1));
		else if (hph_mode == CLS_H_LOHIFI)
			snd_soc_component_update_bits(component,
						      WCD9378_HPH_NEW_INT_RDAC_HD2_CTL_L,
						      0x0f, 0x06);
		usleep_range(5000, 5010);
		wcd_clsh_ctrl_set_state(wcd9378->clsh_info,
					WCD_CLSH_EVENT_PRE_DAC,
					WCD_CLSH_STATE_EAR,
					hph_mode);
		break;
	case SND_SOC_DAPM_POST_PMD:
		if (hph_mode == CLS_AB_HIFI || hph_mode == CLS_H_LOHIFI ||
		    hph_mode == CLS_H_HIFI)
			snd_soc_component_update_bits(component,
						      WCD9378_HPH_NEW_INT_RDAC_HD2_CTL_L,
						      0x0f, BIT(0));
		break;
	}

	return 0;
}

static int wcd9378_codec_aux_dac_event(struct snd_soc_dapm_widget *w,
				       struct snd_kcontrol *kcontrol,
				       int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);
	int hph_mode = wcd9378->hph_mode;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		wcd9378_rx_clk_enable(component);
		snd_soc_component_update_bits(component,
					      WCD9378_AUX_AUXPA,
					      BIT(4), BIT(4));
		wcd_clsh_ctrl_set_state(wcd9378->clsh_info,
					WCD_CLSH_EVENT_PRE_DAC,
					WCD_CLSH_STATE_AUX,
					hph_mode);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component,
					      WCD9378_AUX_AUXPA,
					      BIT(4), 0x00);
		break;
	}

	return 0;
}

static int wcd9378_codec_enable_hphr_pa(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *kcontrol,
					int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);
	int hph_mode = wcd9378->hph_mode;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Wake TX slave so its regmap is not in cache-only mode */
			pm_runtime_get_sync(wcd9378->txdev);
		wcd_clsh_ctrl_set_state(wcd9378->clsh_info,
					WCD_CLSH_EVENT_PRE_DAC,
					WCD_CLSH_STATE_HPHR,
					hph_mode);
		snd_soc_component_update_bits(component,
					      WCD9378_ANA_HPH, BIT(4), BIT(4));
		usleep_range(100, 110);
		set_bit(HPH_PA_DELAY, &wcd9378->status_mask);
		snd_soc_component_update_bits(component,
					      WCD9378_PDM_WD_CTL1, 0x07, 0x03);
		break;
	case SND_SOC_DAPM_POST_PMU:
		if (test_bit(HPH_PA_DELAY, &wcd9378->status_mask)) {
			if (wcd9378->comp2_enable)
				usleep_range(7000, 7100);
			else
				usleep_range(20000, 20100);
			clear_bit(HPH_PA_DELAY, &wcd9378->status_mask);
		}
		snd_soc_component_update_bits(component,
					      WCD9378_HPH_NEW_INT_HPH_TIMER1,
					      BIT(1), BIT(1));
		if (hph_mode == CLS_AB || hph_mode == CLS_AB_HIFI)
			snd_soc_component_update_bits(component,
						      WCD9378_ANA_RX_SUPPLIES,
						      BIT(1), BIT(1));
		enable_irq(wcd9378->hphr_pdm_wd_int);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		disable_irq_nosync(wcd9378->hphr_pdm_wd_int);
		set_bit(HPH_PA_DELAY, &wcd9378->status_mask);
		wcd_mbhc_event_notify(wcd9378->wcd_mbhc,
				      WCD_EVENT_PRE_HPHR_PA_OFF);
		break;
	case SND_SOC_DAPM_POST_PMD:
		if (test_bit(HPH_PA_DELAY, &wcd9378->status_mask)) {
			if (wcd9378->comp2_enable)
				usleep_range(7000, 7100);
			else
				usleep_range(20000, 20100);
			clear_bit(HPH_PA_DELAY, &wcd9378->status_mask);
		}
		wcd_mbhc_event_notify(wcd9378->wcd_mbhc,
				      WCD_EVENT_POST_HPHR_PA_OFF);
		snd_soc_component_update_bits(component,
					      WCD9378_PDM_WD_CTL1, 0x07, 0x00);
		snd_soc_component_update_bits(component,
					      WCD9378_ANA_HPH, BIT(4), 0x00);
		wcd_clsh_ctrl_set_state(wcd9378->clsh_info,
					WCD_CLSH_EVENT_POST_PA,
					WCD_CLSH_STATE_HPHR,
					hph_mode);
		pm_runtime_put_autosuspend(wcd9378->txdev);
		break;
	}

	return 0;
}

static int wcd9378_codec_enable_hphl_pa(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *kcontrol,
					int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);
	int hph_mode = wcd9378->hph_mode;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Wake TX slave so its regmap is not in cache-only mode */
			pm_runtime_get_sync(wcd9378->txdev);
			wcd_clsh_ctrl_set_state(wcd9378->clsh_info,
					WCD_CLSH_EVENT_PRE_DAC,
					WCD_CLSH_STATE_HPHL,
					hph_mode);
		snd_soc_component_update_bits(component,
					      WCD9378_ANA_HPH, BIT(5), BIT(5));
		usleep_range(100, 110);
		set_bit(HPH_PA_DELAY, &wcd9378->status_mask);
		snd_soc_component_update_bits(component,
					      WCD9378_PDM_WD_CTL0, 0x07, 0x03);
		break;
	case SND_SOC_DAPM_POST_PMU:
		if (test_bit(HPH_PA_DELAY, &wcd9378->status_mask)) {
			if (!wcd9378->comp1_enable)
				usleep_range(20000, 20100);
			else
				usleep_range(7000, 7100);
			clear_bit(HPH_PA_DELAY, &wcd9378->status_mask);
		}
		snd_soc_component_update_bits(component,
					      WCD9378_HPH_NEW_INT_HPH_TIMER1,
					      BIT(1), BIT(1));
		if (hph_mode == CLS_AB || hph_mode == CLS_AB_HIFI)
			snd_soc_component_update_bits(component,
						      WCD9378_ANA_RX_SUPPLIES,
						      BIT(1), BIT(1));
		enable_irq(wcd9378->hphl_pdm_wd_int);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		disable_irq_nosync(wcd9378->hphl_pdm_wd_int);
		set_bit(HPH_PA_DELAY, &wcd9378->status_mask);
		wcd_mbhc_event_notify(wcd9378->wcd_mbhc,
				      WCD_EVENT_PRE_HPHL_PA_OFF);
		break;
	case SND_SOC_DAPM_POST_PMD:
		if (test_bit(HPH_PA_DELAY, &wcd9378->status_mask)) {
			if (!wcd9378->comp1_enable)
				usleep_range(20000, 20100);
			else
				usleep_range(7000, 7100);
			clear_bit(HPH_PA_DELAY, &wcd9378->status_mask);
		}
		wcd_mbhc_event_notify(wcd9378->wcd_mbhc,
				      WCD_EVENT_POST_HPHL_PA_OFF);
		snd_soc_component_update_bits(component,
					      WCD9378_PDM_WD_CTL0, 0x07, 0x00);
		snd_soc_component_update_bits(component,
					      WCD9378_ANA_HPH, BIT(5), 0x00);
		wcd_clsh_ctrl_set_state(wcd9378->clsh_info,
					WCD_CLSH_EVENT_POST_PA,
					WCD_CLSH_STATE_HPHL,
					hph_mode);
		pm_runtime_put_autosuspend(wcd9378->txdev);
		break;
	}

	return 0;
}

static int wcd9378_codec_enable_aux_pa(struct snd_soc_dapm_widget *w,
				       struct snd_kcontrol *kcontrol,
				       int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(component,
					      WCD9378_PDM_WD_CTL2, 0x07, 0x03);
		break;
	case SND_SOC_DAPM_POST_PMU:
		enable_irq(wcd9378->aux_pdm_wd_int);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		disable_irq_nosync(wcd9378->aux_pdm_wd_int);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component,
					      WCD9378_PDM_WD_CTL2, 0x07, 0x00);
		break;
	}

	return 0;
}

static int wcd9378_codec_enable_ear_pa(struct snd_soc_dapm_widget *w,
				       struct snd_kcontrol *kcontrol,
				       int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);
	int hph_mode = wcd9378->hph_mode;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* EAR PA enable is hardware-sequenced via RDAC3 */
		break;
	case SND_SOC_DAPM_POST_PMU:
		wcd_clsh_ctrl_set_state(wcd9378->clsh_info,
					WCD_CLSH_EVENT_PRE_DAC,
					WCD_CLSH_STATE_EAR,
					hph_mode);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		break;
	case SND_SOC_DAPM_POST_PMD:
		wcd_clsh_ctrl_set_state(wcd9378->clsh_info,
					WCD_CLSH_EVENT_POST_PA,
					WCD_CLSH_STATE_EAR,
					hph_mode);
		usleep_range(7000, 7100);
		break;
	}

	return 0;
}

static int wcd9378_enable_rx1(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	if (event == SND_SOC_DAPM_POST_PMD)
		wcd9378_rx_clk_disable(component);

	return 0;
}

static int wcd9378_enable_rx2(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	if (event == SND_SOC_DAPM_POST_PMD)
		wcd9378_rx_clk_disable(component);

	return 0;
}

static int wcd9378_enable_rx3(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	if (event == SND_SOC_DAPM_POST_PMD)
		wcd9378_rx_clk_disable(component);

	return 0;
}

/* ------------------------------------------------------------------ */
/* DAPM event callbacks — TX path                                      */
/* ------------------------------------------------------------------ */
static int wcd9378_tx_swr_ctrl(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);

	if (event == SND_SOC_DAPM_PRE_PMU &&
	    strnstr(w->name, "ADC", sizeof("ADC")))
		set_bit(AMIC2_BCS_ENABLE, &wcd9378->status_mask);


	return 0;
}

static int wcd9378_codec_enable_adc(struct snd_soc_dapm_widget *w,
				    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(component,
					      WCD9378_ANA_TX_CH1 + w->shift,
					      BIT(7), BIT(7));
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component,
					      WCD9378_ANA_TX_CH1 + w->shift,
					      BIT(7), 0x00);
		break;
	}

	return 0;
}

static int wcd9378_codec_enable_dmic(struct snd_soc_dapm_widget *w,
				     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	u8 dmic_clk_en = BIT(0);
	u16 dmic_clk_reg;
	int dmic_idx;

	if (sscanf(w->name, "DMIC%d", &dmic_idx) != 1) {
		dev_err(component->dev, "Invalid DMIC widget: %s\n", w->name);
		return -EINVAL;
	}

	/* DMIC0/1 share ANA_TX_CH1, DMIC2/3 share ANA_TX_CH2, etc. */
	dmic_clk_reg = WCD9378_ANA_TX_CH1 + (dmic_idx / 2);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(component, dmic_clk_reg,
					      dmic_clk_en, dmic_clk_en);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component, dmic_clk_reg,
					      dmic_clk_en, 0x00);
		break;
	}

	return 0;
}

static int wcd9378_codec_enable_micbias(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *kcontrol,
					int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	int micb_num = w->shift;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		wcd9378_micbias_control(component, micb_num, MICB_ENABLE, true);
		break;
	case SND_SOC_DAPM_POST_PMU:
		/* 2ms settling time after micbias enable */
		usleep_range(2000, 2100);
		break;
	case SND_SOC_DAPM_POST_PMD:
		wcd9378_micbias_control(component, micb_num, MICB_DISABLE, true);
		break;
	}

	return 0;
}

static int wcd9378_codec_enable_micbias_pullup(struct snd_soc_dapm_widget *w,
					       struct snd_kcontrol *kcontrol,
					       int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	int micb_num = w->shift;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		wcd9378_micbias_control(component, micb_num,
					MICB_PULLUP_ENABLE, true);
		break;
	case SND_SOC_DAPM_POST_PMU:
		usleep_range(2000, 2100);
		break;
	case SND_SOC_DAPM_POST_PMD:
		wcd9378_micbias_control(component, micb_num,
					MICB_PULLUP_DISABLE, true);
		break;
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* MBHC ops — full set matching wcd937x upstream                       */
/* ------------------------------------------------------------------ */
static void wcd9378_mbhc_clk_setup(struct snd_soc_component *component,
				   bool enable)
{
	snd_soc_component_write_field(component, WCD9378_MBHC_NEW_CTL_1,
				      WCD9378_MBHC_CTL_RCO_EN_MASK, enable);
}

static void wcd9378_mbhc_mbhc_bias_control(struct snd_soc_component *component,
					   bool enable)
{
	snd_soc_component_write_field(component, WCD9378_ANA_MBHC_ELECT,
				      WCD9378_ANA_MBHC_BIAS_EN_MASK, enable);
}

static void wcd9378_mbhc_program_btn_thr(struct snd_soc_component *component,
					 int *btn_low, int *btn_high,
					 int num_btn, bool is_micbias)
{
	int i, vth;

	if (num_btn > WCD_MBHC_DEF_BUTTONS) {
		dev_err(component->dev, "%s: invalid number of buttons: %d\n",
			__func__, num_btn);
		return;
	}

	for (i = 0; i < num_btn; i++) {
		vth = ((btn_high[i] * 2) / 25) & 0x3F;
		snd_soc_component_write_field(component,
					      WCD9378_ANA_MBHC_BTN0 + i,
					      WCD9378_MBHC_BTN_VTH_MASK, vth);
	}
}

static bool wcd9378_mbhc_micb_en_status(struct snd_soc_component *component,
					int micb_num)
{
	u8 val;

	if (micb_num == MIC_BIAS_2) {
		val = snd_soc_component_read_field(component,
						   WCD9378_ANA_MICB2,
						   WCD9378_ANA_MICB2_ENABLE_MASK);
		if (val == WCD9378_MICB_ENABLE)
			return true;
	}

	return false;
}

static void wcd9378_mbhc_hph_l_pull_up_control(struct snd_soc_component *component,
						int pull_up_cur)
{
	/* Default pull up current to 2uA */
	if (pull_up_cur > HS_PULLUP_I_OFF || pull_up_cur < HS_PULLUP_I_3P0_UA)
		pull_up_cur = HS_PULLUP_I_2P0_UA;

	snd_soc_component_write_field(component,
				      WCD9378_MBHC_NEW_INT_MECH_DET_CURRENT,
				      WCD9378_HSDET_PULLUP_C_MASK, pull_up_cur);
}

static int wcd9378_mbhc_request_micbias(struct snd_soc_component *component,
					int micb_num, int req)
{
	return wcd9378_micbias_control(component, micb_num, req, false);
}

static void wcd9378_mbhc_micb_ramp_control(struct snd_soc_component *component,
					   bool enable)
{
	if (enable) {
		snd_soc_component_write_field(component, WCD9378_ANA_MICB2_RAMP,
					      WCD9378_RAMP_SHIFT_CTRL_MASK, 0x0C);
		snd_soc_component_write_field(component, WCD9378_ANA_MICB2_RAMP,
					      WCD9378_RAMP_EN_MASK, 1);
	} else {
		snd_soc_component_write_field(component, WCD9378_ANA_MICB2_RAMP,
					      WCD9378_RAMP_EN_MASK, 0);
		snd_soc_component_write_field(component, WCD9378_ANA_MICB2_RAMP,
					      WCD9378_RAMP_SHIFT_CTRL_MASK, 0);
	}
}

static int wcd9378_mbhc_micb_adjust_voltage(struct snd_soc_component *component,
					    int req_volt, int micb_num)
{
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);
	int cur_vout_ctl, req_vout_ctl, micb_reg, micb_en, ret = 0;

	switch (micb_num) {
	case MIC_BIAS_1:
		micb_reg = WCD9378_ANA_MICB1;
		break;
	case MIC_BIAS_2:
		micb_reg = WCD9378_ANA_MICB2;
		break;
	case MIC_BIAS_3:
		micb_reg = WCD9378_ANA_MICB3;
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&wcd9378->micb_lock);

	micb_en = snd_soc_component_read_field(component, micb_reg,
					       WCD9378_MICB_EN_MASK);
	cur_vout_ctl = snd_soc_component_read_field(component, micb_reg,
						    WCD9378_MICB_VOUT_MASK);
	req_vout_ctl = wcd_get_micb_vout_ctl_val(component->dev, req_volt);
	if (req_vout_ctl < 0) {
		ret = -EINVAL;
		goto exit;
	}

	if (cur_vout_ctl == req_vout_ctl)
		goto exit;

	if (micb_en == WCD9378_MICB_ENABLE)
		snd_soc_component_write_field(component, micb_reg,
					      WCD9378_MICB_EN_MASK,
					      WCD9378_MICB_PULL_UP);

	snd_soc_component_write_field(component, micb_reg,
				      WCD9378_MICB_VOUT_MASK, req_vout_ctl);

	if (micb_en == WCD9378_MICB_ENABLE) {
		snd_soc_component_write_field(component, micb_reg,
					      WCD9378_MICB_EN_MASK,
					      WCD9378_MICB_ENABLE);
		usleep_range(2000, 2100);
	}
exit:
	mutex_unlock(&wcd9378->micb_lock);
	return ret;
}

static int wcd9378_mbhc_micb_ctrl_threshold_mic(struct snd_soc_component *component,
						int micb_num, bool req_en)
{
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);
	int micb_mv;

	if (micb_num != MIC_BIAS_2)
		return -EINVAL;

	if (wcd9378->common.micb_mv[1] >= WCD_MBHC_THR_HS_MICB_MV)
		return 0;

	micb_mv = req_en ? WCD_MBHC_THR_HS_MICB_MV : wcd9378->common.micb_mv[1];

	return wcd9378_mbhc_micb_adjust_voltage(component, micb_mv, MIC_BIAS_2);
}

static void wcd9378_mbhc_get_result_params(struct snd_soc_component *component,
					   s16 *d1_a, u16 noff,
					   int32_t *zdet)
{
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);
	int i;
	int val, val1;
	s16 c1;
	s32 x1, d1;
	s32 denom;
	static const int minCode_param[] = {
		3277, 1639, 820, 410, 205, 103, 52, 26
	};

	regmap_update_bits(wcd9378->regmap, WCD9378_ANA_MBHC_ZDET, 0x20, 0x20);
	for (i = 0; i < WCD9378_ZDET_NUM_MEASUREMENTS; i++) {
		regmap_read(wcd9378->regmap, WCD9378_ANA_MBHC_RESULT_2, &val);
		if (val & 0x80)
			break;
	}
	val = val << 0x8;
	regmap_read(wcd9378->regmap, WCD9378_ANA_MBHC_RESULT_1, &val1);
	val |= val1;
	regmap_update_bits(wcd9378->regmap, WCD9378_ANA_MBHC_ZDET, 0x20, 0x00);
	x1 = WCD9378_MBHC_GET_X1(val);
	c1 = WCD9378_MBHC_GET_C1(val);
	/* If ramp is not complete, give additional 5ms */
	if (c1 < 2 && x1)
		usleep_range(5000, 5050);

	if (!c1 || !x1) {
		dev_err(component->dev,
			"Impedance detect ramp error, c1=%d, x1=0x%x\n",
			c1, x1);
		goto ramp_down;
	}
	d1 = d1_a[c1];
	denom = (x1 * d1) - (1 << (14 - noff));
	if (denom > 0)
		*zdet = (WCD9378_MBHC_ZDET_CONST * 1000) / denom;
	else if (x1 < minCode_param[noff])
		*zdet = WCD9378_ZDET_FLOATING_IMPEDANCE;

	dev_dbg(component->dev,
		"%s: d1=%d, c1=%d, x1=0x%x, z_val=%d (milliohm)\n",
		__func__, d1, c1, x1, *zdet);
ramp_down:
	i = 0;
	while (x1) {
		regmap_read(wcd9378->regmap, WCD9378_ANA_MBHC_RESULT_1, &val);
		regmap_read(wcd9378->regmap, WCD9378_ANA_MBHC_RESULT_2, &val1);
		val = val << 0x08;
		val |= val1;
		x1 = WCD9378_MBHC_GET_X1(val);
		i++;
		if (i == WCD9378_ZDET_NUM_MEASUREMENTS)
			break;
	}
}

static void wcd9378_mbhc_zdet_ramp(struct snd_soc_component *component,
				   struct wcd9378_mbhc_zdet_param *zdet_param,
				   s32 *zl, s32 *zr, s16 *d1_a)
{
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);
	s32 zdet = 0;

	snd_soc_component_write_field(component, WCD9378_MBHC_NEW_ZDET_ANA_CTL,
				      WCD9378_ZDET_MAXV_CTL_MASK,
				      zdet_param->ldo_ctl);
	snd_soc_component_update_bits(component, WCD9378_ANA_MBHC_BTN5,
				      WCD9378_MBHC_BTN_VTH_MASK,
				      zdet_param->btn5);
	snd_soc_component_update_bits(component, WCD9378_ANA_MBHC_BTN6,
				      WCD9378_MBHC_BTN_VTH_MASK,
				      zdet_param->btn6);
	snd_soc_component_update_bits(component, WCD9378_ANA_MBHC_BTN7,
				      WCD9378_MBHC_BTN_VTH_MASK,
				      zdet_param->btn7);
	snd_soc_component_write_field(component, WCD9378_MBHC_NEW_ZDET_ANA_CTL,
				      WCD9378_ZDET_RANGE_CTL_MASK,
				      zdet_param->noff);
	snd_soc_component_update_bits(component, WCD9378_MBHC_NEW_ZDET_RAMP_CTL,
				      0x0F, zdet_param->nshift);

	if (!zl)
		goto z_right;

	/* Start impedance measurement for HPH_L */
	regmap_update_bits(wcd9378->regmap, WCD9378_ANA_MBHC_ZDET, 0x80, 0x80);
	wcd9378_mbhc_get_result_params(component, d1_a, zdet_param->noff,
				       &zdet);
	regmap_update_bits(wcd9378->regmap, WCD9378_ANA_MBHC_ZDET, 0x80, 0x00);
	*zl = zdet;

z_right:
	if (!zr)
		return;

	/* Start impedance measurement for HPH_R */
	regmap_update_bits(wcd9378->regmap, WCD9378_ANA_MBHC_ZDET, 0x40, 0x40);
	wcd9378_mbhc_get_result_params(component, d1_a, zdet_param->noff,
				       &zdet);
	regmap_update_bits(wcd9378->regmap, WCD9378_ANA_MBHC_ZDET, 0x40, 0x00);
	*zr = zdet;
}

static void wcd9378_wcd_mbhc_qfuse_cal(struct snd_soc_component *component,
					s32 *z_val, int flag_l_r)
{
	s16 q1;
	int q1_cal;

	if (*z_val < (WCD9378_ZDET_VAL_400 / 1000))
		q1 = snd_soc_component_read(component,
					    WCD9378_EFUSE_REG_23 +
					    (2 * flag_l_r));
	else
		q1 = snd_soc_component_read(component,
					    WCD9378_EFUSE_REG_24 +
					    (2 * flag_l_r));

	if (q1 & 0x80)
		q1_cal = (10000 - ((q1 & 0x7F) * 25));
	else
		q1_cal = (10000 + (q1 * 25));

	if (q1_cal > 0)
		*z_val = ((*z_val) * 10000) / q1_cal;
}

static void wcd9378_wcd_mbhc_calc_impedance(struct snd_soc_component *component,
					    u32 *zl, u32 *zr)
{
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);
	s16 reg0, reg1, reg2, reg3, reg4;
	s32 z1l, z1r, z1ls;
	int zMono, z_diff1, z_diff2;
	bool is_fsm_disable = false;
	struct wcd9378_mbhc_zdet_param zdet_param[] = {
		{4, 0, 4, 0x08, 0x14, 0x18}, /* < 32ohm */
		{2, 0, 3, 0x18, 0x7C, 0x90}, /* 32ohm < Z < 400ohm */
		{1, 4, 5, 0x18, 0x7C, 0x90}, /* 400ohm < Z < 1200ohm */
		{1, 6, 7, 0x18, 0x7C, 0x90}, /* >1200ohm */
	};
	struct wcd9378_mbhc_zdet_param *zdet_param_ptr = NULL;
	s16 d1_a[][4] = {
		{0, 30, 90, 30},
		{0, 30, 30, 5},
		{0, 30, 30, 5},
		{0, 30, 30, 5},
	};
	s16 *d1 = NULL;

	reg0 = snd_soc_component_read(component, WCD9378_ANA_MBHC_BTN5);
	reg1 = snd_soc_component_read(component, WCD9378_ANA_MBHC_BTN6);
	reg2 = snd_soc_component_read(component, WCD9378_ANA_MBHC_BTN7);
	reg3 = snd_soc_component_read(component, WCD9378_MBHC_CTL_CLK);
	reg4 = snd_soc_component_read(component, WCD9378_MBHC_NEW_ZDET_ANA_CTL);

	if (snd_soc_component_read(component, WCD9378_ANA_MBHC_ELECT) & 0x80) {
		is_fsm_disable = true;
		regmap_update_bits(wcd9378->regmap,
				   WCD9378_ANA_MBHC_ELECT, 0x80, 0x00);
	}

	/* For NO-jack, disable L_DET_EN before Z-det measurements */
	if (wcd9378->mbhc_cfg.hphl_swh)
		regmap_update_bits(wcd9378->regmap,
				   WCD9378_ANA_MBHC_MECH, 0x80, 0x00);

	/* Turn off 100k pull down on HPHL */
	regmap_update_bits(wcd9378->regmap, WCD9378_ANA_MBHC_MECH, 0x01, 0x00);

	/* Disable surge protection before impedance detection */
	regmap_update_bits(wcd9378->regmap,
			   WCD9378_HPH_SURGE_HPHLR_SURGE_EN, 0xC0, 0x00);
	usleep_range(1000, 1010);

	/* First get impedance on Left */
	d1 = d1_a[1];
	zdet_param_ptr = &zdet_param[1];
	wcd9378_mbhc_zdet_ramp(component, zdet_param_ptr, &z1l, NULL, d1);

	if (!WCD9378_MBHC_IS_SECOND_RAMP_REQUIRED(z1l))
		goto left_ch_impedance;

	/* Second ramp for left ch */
	if (z1l < WCD9378_ZDET_VAL_32) {
		zdet_param_ptr = &zdet_param[0];
		d1 = d1_a[0];
	} else if ((z1l > WCD9378_ZDET_VAL_400) &&
		   (z1l <= WCD9378_ZDET_VAL_1200)) {
		zdet_param_ptr = &zdet_param[2];
		d1 = d1_a[2];
	} else if (z1l > WCD9378_ZDET_VAL_1200) {
		zdet_param_ptr = &zdet_param[3];
		d1 = d1_a[3];
	}
	wcd9378_mbhc_zdet_ramp(component, zdet_param_ptr, &z1l, NULL, d1);

left_ch_impedance:
	if (z1l == WCD9378_ZDET_FLOATING_IMPEDANCE ||
	    z1l > WCD9378_ZDET_VAL_100K) {
		*zl = WCD9378_ZDET_FLOATING_IMPEDANCE;
		zdet_param_ptr = &zdet_param[1];
		d1 = d1_a[1];
	} else {
		*zl = z1l / 1000;
		wcd9378_wcd_mbhc_qfuse_cal(component, zl, 0);
	}

	/* Start of right impedance ramp and calculation */
	wcd9378_mbhc_zdet_ramp(component, zdet_param_ptr, NULL, &z1r, d1);
	if (WCD9378_MBHC_IS_SECOND_RAMP_REQUIRED(z1r)) {
		if ((z1r > WCD9378_ZDET_VAL_1200 &&
		     zdet_param_ptr->noff == 0x6) ||
		    ((*zl) != WCD9378_ZDET_FLOATING_IMPEDANCE))
			goto right_ch_impedance;
		/* Second ramp for right ch */
		if (z1r < WCD9378_ZDET_VAL_32) {
			zdet_param_ptr = &zdet_param[0];
			d1 = d1_a[0];
		} else if ((z1r > WCD9378_ZDET_VAL_400) &&
			   (z1r <= WCD9378_ZDET_VAL_1200)) {
			zdet_param_ptr = &zdet_param[2];
			d1 = d1_a[2];
		} else if (z1r > WCD9378_ZDET_VAL_1200) {
			zdet_param_ptr = &zdet_param[3];
			d1 = d1_a[3];
		}
		wcd9378_mbhc_zdet_ramp(component, zdet_param_ptr, NULL, &z1r,
				       d1);
	}
right_ch_impedance:
	if (z1r == WCD9378_ZDET_FLOATING_IMPEDANCE ||
	    z1r > WCD9378_ZDET_VAL_100K) {
		*zr = WCD9378_ZDET_FLOATING_IMPEDANCE;
	} else {
		*zr = z1r / 1000;
		wcd9378_wcd_mbhc_qfuse_cal(component, zr, 1);
	}

	/* Mono/stereo detection */
	if ((*zl == WCD9378_ZDET_FLOATING_IMPEDANCE) &&
	    (*zr == WCD9378_ZDET_FLOATING_IMPEDANCE)) {
		dev_dbg(component->dev,
			"%s: plug type is invalid or extension cable\n",
			__func__);
		goto zdet_complete;
	}
	if ((*zl == WCD9378_ZDET_FLOATING_IMPEDANCE) ||
	    (*zr == WCD9378_ZDET_FLOATING_IMPEDANCE) ||
	    ((*zl < WCD_MONO_HS_MIN_THR) && (*zr > WCD_MONO_HS_MIN_THR)) ||
	    ((*zl > WCD_MONO_HS_MIN_THR) && (*zr < WCD_MONO_HS_MIN_THR))) {
		wcd_mbhc_set_hph_type(wcd9378->wcd_mbhc, WCD_MBHC_HPH_MONO);
		goto zdet_complete;
	}
	snd_soc_component_write_field(component, WCD9378_HPH_R_ATEST,
				      WCD9378_HPHPA_GND_OVR_MASK, 1);
	snd_soc_component_write_field(component, WCD9378_HPH_PA_CTL2,
				      WCD9378_HPH_PA_CTL2_HPHR_GND_MASK, 1);
	if (*zl < (WCD9378_ZDET_VAL_32 / 1000))
		wcd9378_mbhc_zdet_ramp(component, &zdet_param[0], &z1ls,
				       NULL, d1);
	else
		wcd9378_mbhc_zdet_ramp(component, &zdet_param[1], &z1ls,
				       NULL, d1);
	snd_soc_component_write_field(component, WCD9378_HPH_PA_CTL2,
				      WCD9378_HPH_PA_CTL2_HPHR_GND_MASK, 0);
	snd_soc_component_write_field(component, WCD9378_HPH_R_ATEST,
				      WCD9378_HPHPA_GND_OVR_MASK, 0);
	z1ls /= 1000;
	wcd9378_wcd_mbhc_qfuse_cal(component, &z1ls, 0);
	/* Parallel of left Z and 9 ohm pull down resistor */
	zMono = ((*zl) * 9) / ((*zl) + 9);
	z_diff1 = (z1ls > zMono) ? (z1ls - zMono) : (zMono - z1ls);
	z_diff2 = ((*zl) > z1ls) ? ((*zl) - z1ls) : (z1ls - (*zl));
	if ((z_diff1 * (*zl + z1ls)) > (z_diff2 * (z1ls + zMono)))
		wcd_mbhc_set_hph_type(wcd9378->wcd_mbhc, WCD_MBHC_HPH_STEREO);
	else
		wcd_mbhc_set_hph_type(wcd9378->wcd_mbhc, WCD_MBHC_HPH_MONO);

	/* Re-enable surge protection after impedance detection */
	regmap_update_bits(wcd9378->regmap,
			   WCD9378_HPH_SURGE_HPHLR_SURGE_EN, 0xC0, 0xC0);
	/*
	 * MBHC bringup: immediately clear surge detection interrupt that fires
	 * when surge protection is re-enabled with no HPH load connected.
	 * This prevents SDCA_CASCADE in SDW_DP0_INT → SLAVE_PEND_IRQ ALERT loop.
	 */
	regmap_write(wcd9378->regmap, SWRS_SCP_SDCA_INTSTAT_1, 0xff);
	regmap_write(wcd9378->regmap, SWRS_SCP_SDCA_INTSTAT_2, 0xff);
	regmap_write(wcd9378->regmap, SWRS_SCP_SDCA_INTSTAT_3, 0xff);
zdet_complete:
	snd_soc_component_write(component, WCD9378_ANA_MBHC_BTN5, reg0);
	snd_soc_component_write(component, WCD9378_ANA_MBHC_BTN6, reg1);
	snd_soc_component_write(component, WCD9378_ANA_MBHC_BTN7, reg2);
	/* Turn on 100k pull down on HPHL */
	regmap_update_bits(wcd9378->regmap, WCD9378_ANA_MBHC_MECH, 0x01, 0x01);

	/* For NO-jack, re-enable L_DET_EN after Z-det measurements */
	if (wcd9378->mbhc_cfg.hphl_swh)
		regmap_update_bits(wcd9378->regmap,
				   WCD9378_ANA_MBHC_MECH, 0x80, 0x80);

	snd_soc_component_write(component, WCD9378_MBHC_NEW_ZDET_ANA_CTL, reg4);
	snd_soc_component_write(component, WCD9378_MBHC_CTL_CLK, reg3);
	if (is_fsm_disable)
		regmap_update_bits(wcd9378->regmap,
				   WCD9378_ANA_MBHC_ELECT, 0x80, 0x80);
}

static void wcd9378_mbhc_gnd_det_ctrl(struct snd_soc_component *component,
				      bool enable)
{
	if (enable) {
		snd_soc_component_write_field(component, WCD9378_ANA_MBHC_MECH,
					      WCD9378_MBHC_HSG_PULLUP_COMP_EN,
					      1);
		snd_soc_component_write_field(component, WCD9378_ANA_MBHC_MECH,
					      WCD9378_MBHC_GND_DET_EN_MASK, 1);
	} else {
		snd_soc_component_write_field(component, WCD9378_ANA_MBHC_MECH,
					      WCD9378_MBHC_GND_DET_EN_MASK, 0);
		snd_soc_component_write_field(component, WCD9378_ANA_MBHC_MECH,
					      WCD9378_MBHC_HSG_PULLUP_COMP_EN,
					      0);
	}
}

static void wcd9378_mbhc_hph_pull_down_ctrl(struct snd_soc_component *component,
					    bool enable)
{
	snd_soc_component_write_field(component, WCD9378_HPH_PA_CTL2,
				      WCD9378_HPH_PA_CTL2_HPHR_GND_MASK,
				      enable);
	snd_soc_component_write_field(component, WCD9378_HPH_PA_CTL2,
				      WCD9378_HPH_PA_CTL2_HPHL_GND_MASK,
				      enable);
}

static void wcd9378_mbhc_moisture_config(struct snd_soc_component *component)
{
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);

	if (wcd9378->mbhc_cfg.moist_rref == R_OFF) {
		snd_soc_component_write_field(component, WCD9378_MBHC_NEW_CTL_2,
					      WCD9378_M_RTH_CTL_MASK, R_OFF);
		return;
	}

	/* Do not enable moisture detection if jack type is NC */
	if (!wcd9378->mbhc_cfg.hphl_swh) {
		dev_dbg(component->dev,
			"%s: disable moisture detection for NC\n", __func__);
		snd_soc_component_write_field(component, WCD9378_MBHC_NEW_CTL_2,
					      WCD9378_M_RTH_CTL_MASK, R_OFF);
		return;
	}

	snd_soc_component_write_field(component, WCD9378_MBHC_NEW_CTL_2,
				      WCD9378_M_RTH_CTL_MASK,
				      wcd9378->mbhc_cfg.moist_rref);
}

static void wcd9378_mbhc_moisture_detect_en(struct snd_soc_component *component,
					    bool enable)
{
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);

	if (enable)
		snd_soc_component_write_field(component, WCD9378_MBHC_NEW_CTL_2,
					      WCD9378_M_RTH_CTL_MASK,
					      wcd9378->mbhc_cfg.moist_rref);
	else
		snd_soc_component_write_field(component, WCD9378_MBHC_NEW_CTL_2,
					      WCD9378_M_RTH_CTL_MASK, R_OFF);
}

static bool wcd9378_mbhc_get_moisture_status(struct snd_soc_component *component)
{
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);
	bool ret = false;

	if (wcd9378->mbhc_cfg.moist_rref == R_OFF) {
		snd_soc_component_write_field(component, WCD9378_MBHC_NEW_CTL_2,
					      WCD9378_M_RTH_CTL_MASK, R_OFF);
		goto done;
	}

	/* Do not enable moisture detection if jack type is NC */
	if (!wcd9378->mbhc_cfg.hphl_swh) {
		dev_dbg(component->dev,
			"%s: disable moisture detection for NC\n", __func__);
		snd_soc_component_write_field(component, WCD9378_MBHC_NEW_CTL_2,
					      WCD9378_M_RTH_CTL_MASK, R_OFF);
		goto done;
	}

	if (snd_soc_component_read_field(component, WCD9378_MBHC_NEW_CTL_2,
					 WCD9378_M_RTH_CTL_MASK))
		goto done;

	wcd9378_mbhc_moisture_detect_en(component, true);
	ret = ((snd_soc_component_read(component, WCD9378_MBHC_NEW_FSM_STATUS)
		& 0x20) ? 0 : 1);
done:
	return ret;
}

static void wcd9378_mbhc_moisture_polling_ctrl(struct snd_soc_component *component,
					       bool enable)
{
	snd_soc_component_write_field(component,
				      WCD9378_MBHC_NEW_INT_MOISTURE_DET_POLLING_CTRL,
				      WCD9378_MOISTURE_EN_POLLING_MASK, enable);
}

static const struct wcd_mbhc_cb mbhc_cb = {
	.clk_setup		  = wcd9378_mbhc_clk_setup,
	.mbhc_bias		  = wcd9378_mbhc_mbhc_bias_control,
	.set_btn_thr		  = wcd9378_mbhc_program_btn_thr,
	.micbias_enable_status	  = wcd9378_mbhc_micb_en_status,
	.hph_pull_up_control_v2	  = wcd9378_mbhc_hph_l_pull_up_control,
	.mbhc_micbias_control	  = wcd9378_mbhc_request_micbias,
	.mbhc_micb_ramp_control	  = wcd9378_mbhc_micb_ramp_control,
	.mbhc_micb_ctrl_thr_mic	  = wcd9378_mbhc_micb_ctrl_threshold_mic,
	.compute_impedance	  = wcd9378_wcd_mbhc_calc_impedance,
	.mbhc_gnd_det_ctrl	  = wcd9378_mbhc_gnd_det_ctrl,
	.hph_pull_down_ctrl	  = wcd9378_mbhc_hph_pull_down_ctrl,
	.mbhc_moisture_config	  = wcd9378_mbhc_moisture_config,
	.mbhc_get_moisture_status = wcd9378_mbhc_get_moisture_status,
	.mbhc_moisture_polling_ctrl = wcd9378_mbhc_moisture_polling_ctrl,
	.mbhc_moisture_detect_en  = wcd9378_mbhc_moisture_detect_en,
};

/* ------------------------------------------------------------------ */
/* HPH type / impedance kcontrols (added during mbhc_init)            */
/* ------------------------------------------------------------------ */
static int wcd9378_get_hph_type(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] =
		wcd_mbhc_get_hph_type(wcd9378->wcd_mbhc);

	return 0;
}

static int wcd9378_hph_impedance_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	u32 zl, zr;
	bool hphr;
	struct soc_mixer_control *mc;
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);

	mc = (struct soc_mixer_control *)(kcontrol->private_value);
	hphr = mc->shift;
	wcd_mbhc_get_impedance(wcd9378->wcd_mbhc, &zl, &zr);
	ucontrol->value.integer.value[0] = hphr ? zr : zl;

	return 0;
}

static const struct snd_kcontrol_new hph_type_detect_controls[] = {
	SOC_SINGLE_EXT("HPH Type", 0, 0, WCD_MBHC_HPH_STEREO, 0,
		       wcd9378_get_hph_type, NULL),
};

static const struct snd_kcontrol_new impedance_detect_controls[] = {
	SOC_SINGLE_EXT("HPHL Impedance", 0, 0, INT_MAX, 0,
		       wcd9378_hph_impedance_get, NULL),
	SOC_SINGLE_EXT("HPHR Impedance", 0, 1, INT_MAX, 0,
		       wcd9378_hph_impedance_get, NULL),
};

/* ------------------------------------------------------------------ */
/* MBHC init / deinit                                                  */
/* ------------------------------------------------------------------ */
static int wcd9378_mbhc_init(struct snd_soc_component *component)
{
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);
	struct wcd_mbhc_intr *intr_ids = &wcd9378->intr_ids;

	intr_ids->mbhc_sw_intr =
		regmap_irq_get_virq(wcd9378->irq_chip,
				    WCD9378_IRQ_MBHC_SW_DET);
	intr_ids->mbhc_btn_press_intr =
		regmap_irq_get_virq(wcd9378->irq_chip,
				    WCD9378_IRQ_MBHC_BUTTON_PRESS_DET);
	intr_ids->mbhc_btn_release_intr =
		regmap_irq_get_virq(wcd9378->irq_chip,
				    WCD9378_IRQ_MBHC_BUTTON_RELEASE_DET);
	intr_ids->mbhc_hs_ins_intr =
		regmap_irq_get_virq(wcd9378->irq_chip,
				    WCD9378_IRQ_MBHC_ELECT_INS_REM_LEG_DET);
	intr_ids->mbhc_hs_rem_intr =
		regmap_irq_get_virq(wcd9378->irq_chip,
				    WCD9378_IRQ_MBHC_ELECT_INS_REM_DET);
	intr_ids->hph_left_ocp =
		regmap_irq_get_virq(wcd9378->irq_chip,
				    WCD9378_IRQ_HPHL_OCP_INT);
	intr_ids->hph_right_ocp =
		regmap_irq_get_virq(wcd9378->irq_chip,
				    WCD9378_IRQ_HPHR_OCP_INT);

	wcd9378->wcd_mbhc = wcd_mbhc_init(component, &mbhc_cb, intr_ids,
					   wcd_mbhc_fields, true);
	if (IS_ERR(wcd9378->wcd_mbhc))
		return PTR_ERR(wcd9378->wcd_mbhc);

	snd_soc_add_component_controls(component, impedance_detect_controls,
				       ARRAY_SIZE(impedance_detect_controls));
	snd_soc_add_component_controls(component, hph_type_detect_controls,
				       ARRAY_SIZE(hph_type_detect_controls));

	return 0;
}

static void wcd9378_mbhc_deinit(struct snd_soc_component *component)
{
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);

	if (!wcd9378->wcd_mbhc)
		return;

	wcd_mbhc_deinit(wcd9378->wcd_mbhc);
}

/* ------------------------------------------------------------------ */
/* Watchdog IRQ handler                                                */
/* ------------------------------------------------------------------ */
static irqreturn_t wcd9378_wd_handle_irq(int irq, void *data)
{
	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* IRQ domain ops                                                      */
/* ------------------------------------------------------------------ */
static const struct irq_chip wcd9378_irq_chip = {
	.name = "WCD9378",
};

static int wcd9378_irq_chip_map(struct irq_domain *irqd, unsigned int virq,
				irq_hw_number_t hw)
{
	irq_set_chip_and_handler(virq, &wcd9378_irq_chip, handle_simple_irq);
	irq_set_nested_thread(virq, 1);
	irq_set_noprobe(virq);

	return 0;
}

static const struct irq_domain_ops wcd9378_domain_ops = {
	.map = wcd9378_irq_chip_map,
};

static int wcd9378_irq_init(struct wcd9378_priv *wcd, struct device *dev)
{
	wcd->virq = irq_domain_create_linear(NULL, 1,
					     &wcd9378_domain_ops, NULL);
	if (!wcd->virq) {
		dev_err(dev, "%s: Failed to add IRQ domain\n", __func__);
		return -EINVAL;
	}

	return devm_regmap_add_irq_chip(dev, wcd->regmap,
					irq_create_mapping(wcd->virq, 0),
					IRQF_ONESHOT, 0,
					&wcd9378_regmap_irq_chip,
					&wcd->irq_chip);
}

/* ------------------------------------------------------------------ */
/* Micbias data init                                                   */
/* ------------------------------------------------------------------ */
static void wcd9378_set_micbias_data(struct device *dev,
				     struct wcd9378_priv *wcd9378)
{
	regmap_update_bits(wcd9378->regmap, WCD9378_ANA_MICB1,
			   WCD9378_MICB_VOUT_MASK,
			   wcd9378->common.micb_vout[0]);
	regmap_update_bits(wcd9378->regmap, WCD9378_ANA_MICB2,
			   WCD9378_MICB_VOUT_MASK,
			   wcd9378->common.micb_vout[1]);
	regmap_update_bits(wcd9378->regmap, WCD9378_ANA_MICB3,
			   WCD9378_MICB_VOUT_MASK,
			   wcd9378->common.micb_vout[2]);
}

/* ------------------------------------------------------------------ */
/* Swap GND/MIC                                                        */
/* ------------------------------------------------------------------ */
static bool wcd9378_swap_gnd_mic(struct snd_soc_component *component)
{
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);
	int value;

	value = gpiod_get_value(wcd9378->us_euro_gpio);
	gpiod_set_value(wcd9378->us_euro_gpio, !value);

	return true;
}

/* ------------------------------------------------------------------ */

/* Patch 4 — DAPM widgets (playback + capture)                         */
/* ------------------------------------------------------------------ */
static const struct snd_kcontrol_new ear_rdac_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new aux_rdac_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new hphl_rdac_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new hphr_rdac_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const char * const rdac3_mux_text[] = {
	"RX1", "RX3"
};

static const struct soc_enum rdac3_enum =
	SOC_ENUM_SINGLE(WCD9378_CDC_EAR_PATH_CTL, 0,
			ARRAY_SIZE(rdac3_mux_text), rdac3_mux_text);

static const struct snd_kcontrol_new rx_rdac3_mux =
	SOC_DAPM_ENUM("RDAC3_MUX Mux", rdac3_enum);

static const struct snd_kcontrol_new adc1_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new adc2_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new adc3_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new dmic0_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new dmic1_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new dmic2_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new dmic3_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new dmic4_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new dmic5_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new dmic6_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const struct snd_kcontrol_new dmic7_switch[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static const char * const adc2_mux_text[] = {
	"INP2", "INP3"
};

static const struct soc_enum adc2_enum =
	SOC_ENUM_SINGLE(WCD9378_TX_NEW_TX_CH12_MUX,
			WCD9378_TX_CH12_MUX_CH2_SEL_SHIFT,
			ARRAY_SIZE(adc2_mux_text), adc2_mux_text);

static const struct snd_kcontrol_new tx_adc2_mux =
	SOC_DAPM_ENUM("ADC2 MUX Mux", adc2_enum);

static const struct snd_soc_dapm_widget wcd9378_dapm_widgets[] = {
	/* RX input ports */
	SND_SOC_DAPM_INPUT("IN1_HPHL"),
	SND_SOC_DAPM_INPUT("IN2_HPHR"),
	SND_SOC_DAPM_INPUT("IN3_AUX"),

	/* DSD input ports */
	SND_SOC_DAPM_INPUT("IN_DSD_L"),
	SND_SOC_DAPM_INPUT("IN_DSD_R"),

	/* Supply widgets */
	SND_SOC_DAPM_SUPPLY("VDD_BUCK", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY_S("CLS_H_PORT", 1, SND_SOC_NOPM, 0, 0, NULL, 0),

	/* RX mixer widgets */
	SND_SOC_DAPM_MIXER_E("RX1", SND_SOC_NOPM, 0, 0, NULL, 0,
			     wcd9378_enable_rx1,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("RX2", SND_SOC_NOPM, 0, 0, NULL, 0,
			     wcd9378_enable_rx2,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("RX3", SND_SOC_NOPM, 0, 0, NULL, 0,
			     wcd9378_enable_rx3,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	/* RDAC3 MUX — selects RX1 (HPH-L) or RX3 (AUX) for EAR path */
	SND_SOC_DAPM_MUX("RDAC3_MUX", SND_SOC_NOPM, 0, 0, &rx_rdac3_mux),

	/* DAC widgets */
	SND_SOC_DAPM_DAC_E("RDAC1", NULL, SND_SOC_NOPM, 0, 0,
			   wcd9378_codec_hphl_dac_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("RDAC2", NULL, SND_SOC_NOPM, 0, 0,
			   wcd9378_codec_hphr_dac_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("HPH SEQUENCER", SND_SOC_NOPM, 0, 0, NULL, 0,
			     wcd9378_hph_sequencer_enable,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("RDAC3", NULL, SND_SOC_NOPM, 0, 0,
			   wcd9378_codec_ear_dac_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("RDAC4", NULL, SND_SOC_NOPM, 0, 0,
			   wcd9378_codec_aux_dac_event,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	/* RDAC switch mixers */
	SND_SOC_DAPM_MIXER("EAR_RDAC", SND_SOC_NOPM, 0, 0,
			   ear_rdac_switch, ARRAY_SIZE(ear_rdac_switch)),
	SND_SOC_DAPM_MIXER("AUX_RDAC", SND_SOC_NOPM, 0, 0,
			   aux_rdac_switch, ARRAY_SIZE(aux_rdac_switch)),
	SND_SOC_DAPM_MIXER("HPHL_RDAC", SND_SOC_NOPM, 0, 0,
			   hphl_rdac_switch, ARRAY_SIZE(hphl_rdac_switch)),
	SND_SOC_DAPM_MIXER("HPHR_RDAC", SND_SOC_NOPM, 0, 0,
			   hphr_rdac_switch, ARRAY_SIZE(hphr_rdac_switch)),

	/* PA widgets */
	SND_SOC_DAPM_PGA_E("HPHL PGA", WCD9378_ANA_HPH, 7, 0, NULL, 0,
			   wcd9378_codec_enable_hphl_pa,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("HPHR PGA", WCD9378_ANA_HPH, 6, 0, NULL, 0,
			   wcd9378_codec_enable_hphr_pa,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("EAR PGA", WCD9378_ANA_EAR, 7, 0, NULL, 0,
			   wcd9378_codec_enable_ear_pa,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("AUX PGA", WCD9378_AUX_AUXPA, 7, 0, NULL, 0,
			   wcd9378_codec_enable_aux_pa,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	/* DSD PGA widgets — bypass interpolator chain */
	SND_SOC_DAPM_PGA("DSD_L PGA", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("DSD_R PGA", SND_SOC_NOPM, 0, 0, NULL, 0),

	/* RX output ports */
	SND_SOC_DAPM_OUTPUT("HPHL"),
	SND_SOC_DAPM_OUTPUT("HPHR"),
	SND_SOC_DAPM_OUTPUT("EAR"),
	SND_SOC_DAPM_OUTPUT("AUX"),

	/* Capture widgets */
	/* AMIC input ports */
	SND_SOC_DAPM_INPUT("AMIC1"),
	SND_SOC_DAPM_INPUT("AMIC2"),
	SND_SOC_DAPM_INPUT("AMIC3"),

	/* ADC widgets */
	SND_SOC_DAPM_ADC_E("ADC1", NULL, SND_SOC_NOPM, 0, 0,
			   wcd9378_codec_enable_adc,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC2", NULL, SND_SOC_NOPM, 1, 0,
			   wcd9378_codec_enable_adc,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC3", NULL, SND_SOC_NOPM, 2, 0,
			   wcd9378_codec_enable_adc,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	/* ADC2 input mux — selects AMIC2 or AMIC3 */
	SND_SOC_DAPM_MUX("ADC2 MUX", SND_SOC_NOPM, 0, 0, &tx_adc2_mux),

	/* DMIC widgets — DMIC0-7 across 2 physical TX SoundWire lanes */
	SND_SOC_DAPM_ADC_E("DMIC0", NULL, SND_SOC_NOPM, 0, 0,
			   wcd9378_codec_enable_dmic,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("DMIC1", NULL, SND_SOC_NOPM, 1, 0,
			   wcd9378_codec_enable_dmic,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("DMIC2", NULL, SND_SOC_NOPM, 2, 0,
			   wcd9378_codec_enable_dmic,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("DMIC3", NULL, SND_SOC_NOPM, 3, 0,
			   wcd9378_codec_enable_dmic,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("DMIC4", NULL, SND_SOC_NOPM, 4, 0,
			   wcd9378_codec_enable_dmic,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("DMIC5", NULL, SND_SOC_NOPM, 5, 0,
			   wcd9378_codec_enable_dmic,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("DMIC6", NULL, SND_SOC_NOPM, 6, 0,
			   wcd9378_codec_enable_dmic,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("DMIC7", NULL, SND_SOC_NOPM, 7, 0,
			   wcd9378_codec_enable_dmic,
			   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	/* TX SoundWire mixer widgets */
	SND_SOC_DAPM_MIXER_E("ADC1_MIXER", SND_SOC_NOPM, 0, 0,
			     adc1_switch, ARRAY_SIZE(adc1_switch),
			     wcd9378_tx_swr_ctrl,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("ADC2_MIXER", SND_SOC_NOPM, 1, 0,
			     adc2_switch, ARRAY_SIZE(adc2_switch),
			     wcd9378_tx_swr_ctrl,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("ADC3_MIXER", SND_SOC_NOPM, 2, 0,
			     adc3_switch, ARRAY_SIZE(adc3_switch),
			     wcd9378_tx_swr_ctrl,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("DMIC0_MIXER", SND_SOC_NOPM, 0, 0,
			     dmic0_switch, ARRAY_SIZE(dmic0_switch),
			     wcd9378_tx_swr_ctrl,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("DMIC1_MIXER", SND_SOC_NOPM, 1, 0,
			     dmic1_switch, ARRAY_SIZE(dmic1_switch),
			     wcd9378_tx_swr_ctrl,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("DMIC2_MIXER", SND_SOC_NOPM, 2, 0,
			     dmic2_switch, ARRAY_SIZE(dmic2_switch),
			     wcd9378_tx_swr_ctrl,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("DMIC3_MIXER", SND_SOC_NOPM, 3, 0,
			     dmic3_switch, ARRAY_SIZE(dmic3_switch),
			     wcd9378_tx_swr_ctrl,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("DMIC4_MIXER", SND_SOC_NOPM, 4, 0,
			     dmic4_switch, ARRAY_SIZE(dmic4_switch),
			     wcd9378_tx_swr_ctrl,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("DMIC5_MIXER", SND_SOC_NOPM, 5, 0,
			     dmic5_switch, ARRAY_SIZE(dmic5_switch),
			     wcd9378_tx_swr_ctrl,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("DMIC6_MIXER", SND_SOC_NOPM, 6, 0,
			     dmic6_switch, ARRAY_SIZE(dmic6_switch),
			     wcd9378_tx_swr_ctrl,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("DMIC7_MIXER", SND_SOC_NOPM, 7, 0,
			     dmic7_switch, ARRAY_SIZE(dmic7_switch),
			     wcd9378_tx_swr_ctrl,
			     SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	/* Micbias supply widgets */
	SND_SOC_DAPM_SUPPLY("MIC BIAS1", SND_SOC_NOPM, MIC_BIAS_1, 0,
			    wcd9378_codec_enable_micbias,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			    SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS2", SND_SOC_NOPM, MIC_BIAS_2, 0,
			    wcd9378_codec_enable_micbias,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			    SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("MIC BIAS3", SND_SOC_NOPM, MIC_BIAS_3, 0,
			    wcd9378_codec_enable_micbias,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			    SND_SOC_DAPM_POST_PMD),

	/* VA micbias pull-up supply widgets */
	SND_SOC_DAPM_SUPPLY("VA MIC BIAS1", SND_SOC_NOPM, MIC_BIAS_1, 0,
			    wcd9378_codec_enable_micbias_pullup,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			    SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("VA MIC BIAS2", SND_SOC_NOPM, MIC_BIAS_2, 0,
			    wcd9378_codec_enable_micbias_pullup,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			    SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("VA MIC BIAS3", SND_SOC_NOPM, MIC_BIAS_3, 0,
			    wcd9378_codec_enable_micbias_pullup,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			    SND_SOC_DAPM_POST_PMD),

	/* TX output ports */
	SND_SOC_DAPM_OUTPUT("ADC1_OUTPUT"),
	SND_SOC_DAPM_OUTPUT("ADC2_OUTPUT"),
	SND_SOC_DAPM_OUTPUT("ADC3_OUTPUT"),
	SND_SOC_DAPM_OUTPUT("DMIC0_OUTPUT"),
	SND_SOC_DAPM_OUTPUT("DMIC1_OUTPUT"),
	SND_SOC_DAPM_OUTPUT("DMIC2_OUTPUT"),
	SND_SOC_DAPM_OUTPUT("DMIC3_OUTPUT"),
	SND_SOC_DAPM_OUTPUT("DMIC4_OUTPUT"),
	SND_SOC_DAPM_OUTPUT("DMIC5_OUTPUT"),
	SND_SOC_DAPM_OUTPUT("DMIC6_OUTPUT"),
	SND_SOC_DAPM_OUTPUT("DMIC7_OUTPUT"),
	SND_SOC_DAPM_OUTPUT("WCD_TX_OUTPUT"),
};


/* ------------------------------------------------------------------ */

/* Patch 6 — Mixer kcontrols                                           */
/* ------------------------------------------------------------------ */
static int wcd9378_get_swr_port(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);
	struct wcd9378_sdw_priv *wcd;
	struct soc_mixer_control *mixer =
		(struct soc_mixer_control *)kcontrol->private_value;
	int dai_id = mixer->shift;
	int ch_idx = mixer->reg;
	int val;

	wcd = wcd9378->sdw_priv[dai_id];
	val = wcd->ch_info[ch_idx].master_ch_mask ? 1 : 0;
	ucontrol->value.integer.value[0] = val;

	return 0;
}

static void wcd9378_connect_port(struct wcd9378_sdw_priv *wcd,
				 u8 ch_idx, bool enable)
{
	const struct wcd_sdw_ch_info *ch_info = &wcd->ch_info[ch_idx];
	struct sdw_port_config *port_config;
	u8 port_num = ch_info->port_num;
	u8 ch_mask = ch_info->ch_mask;
	u8 mstr_port_num;

	if (port_num == 0 || port_num > WCD9378_MAX_SWR_PORTS)
		return;

	port_config = &wcd->port_config[port_num - 1];
	port_config->num = port_num;

	mstr_port_num = wcd->sdev->m_port_map[port_num];

	if (enable) {
		port_config->ch_mask |= ch_mask;
		wcd->master_channel_map[mstr_port_num] |= ch_info->master_ch_mask;
	} else {
		port_config->ch_mask &= ~ch_mask;
		wcd->master_channel_map[mstr_port_num] &= ~ch_info->master_ch_mask;
	}
}

static int wcd9378_set_swr_port(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);
	struct wcd9378_sdw_priv *wcd;
	struct soc_mixer_control *mixer =
		(struct soc_mixer_control *)kcontrol->private_value;
	int dai_id = mixer->shift;
	int ch_idx = mixer->reg;
	bool enable = ucontrol->value.integer.value[0];

	wcd = wcd9378->sdw_priv[dai_id];
	if (enable)
		wcd->ch_info[ch_idx].master_ch_mask =
			WCD9378_SWRM_CH_MASK(ch_idx + 1);
	else
		wcd->ch_info[ch_idx].master_ch_mask = 0;

	wcd9378_connect_port(wcd, ch_idx, enable);

	return 0;
}

static int wcd9378_rx_hph_mode_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = wcd9378->hph_mode;

	return 0;
}

static int wcd9378_rx_hph_mode_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);
	u32 mode_val;

	mode_val = ucontrol->value.enumerated.item[0];
	if (mode_val == 0) {
		dev_err(component->dev, "%s: Invalid HPH Mode\n", __func__);
		return -EINVAL;
	}
	wcd9378->hph_mode = mode_val;

	return 0;
}

static const char * const rx_hph_mode_mux_text[] = {
	"CLS_H_INVALID", "CLS_H_HIFI", "CLS_H_LP", "CLS_AB",
	"CLS_H_LOHIFI", "CLS_H_ULP", "CLS_AB_HIFI",
};

static const struct soc_enum rx_hph_mode_mux_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(rx_hph_mode_mux_text),
			    rx_hph_mode_mux_text);

static int wcd9378_get_compander(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)(kcontrol->private_value);

	ucontrol->value.integer.value[0] = mc->shift ? wcd9378->comp2_enable :
						   wcd9378->comp1_enable;
	return 0;
}

static int wcd9378_set_compander(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)(kcontrol->private_value);
	int value = ucontrol->value.integer.value[0];

	if (mc->shift)
		wcd9378->comp2_enable = value;
	else
		wcd9378->comp1_enable = value;

	return 0;
}

static const struct snd_kcontrol_new wcd9378_snd_controls[] = {
	SOC_SINGLE_TLV("EAR_PA Volume", WCD9378_ANA_EAR_COMPANDER_CTL,
		       2, 0x10, 0, ear_pa_gain),
	SOC_ENUM_EXT("RX HPH Mode", rx_hph_mode_mux_enum,
		     wcd9378_rx_hph_mode_get, wcd9378_rx_hph_mode_put),

	SOC_SINGLE_TLV("HPHL Volume", WCD9378_HPH_L_EN, 0, 20, 1, line_gain),
	SOC_SINGLE_TLV("HPHR Volume", WCD9378_HPH_R_EN, 0, 20, 1, line_gain),
	SOC_SINGLE_TLV("ADC1 Volume", WCD9378_ANA_TX_CH1, 0, 20, 0, analog_gain),
	SOC_SINGLE_TLV("ADC2 Volume", WCD9378_ANA_TX_CH2, 0, 20, 0, analog_gain),
	SOC_SINGLE_TLV("ADC3 Volume", WCD9378_ANA_TX_CH3, 0, 20, 0, analog_gain),

	/* RX SoundWire port enable switches */
	SOC_SINGLE_EXT("HPHL Switch", WCD9378_HPH_L, 0, 1, 0,
		       wcd9378_get_swr_port, wcd9378_set_swr_port),
	SOC_SINGLE_EXT("HPHR Switch", WCD9378_HPH_R, 0, 1, 0,
		       wcd9378_get_swr_port, wcd9378_set_swr_port),
	SOC_SINGLE_EXT("HPHL_COMP Switch", WCD9378_COMP_L, 0, 1, 0,
		       wcd9378_get_compander, wcd9378_set_compander),
	SOC_SINGLE_EXT("HPHR_COMP Switch", WCD9378_COMP_R, 1, 1, 0,
		       wcd9378_get_compander, wcd9378_set_compander),
	SOC_SINGLE_EXT("LO Switch", WCD9378_LO, 0, 1, 0,
		       wcd9378_get_swr_port, wcd9378_set_swr_port),
	SOC_SINGLE_EXT("CLSH PA Switch", WCD9378_CLSH, 0, 1, 0,
		       wcd9378_get_swr_port, wcd9378_set_swr_port),
	SOC_SINGLE_EXT("DSD_L Switch", WCD9378_DSD_L, 0, 1, 0,
		       wcd9378_get_swr_port, wcd9378_set_swr_port),
	SOC_SINGLE_EXT("DSD_R Switch", WCD9378_DSD_R, 0, 1, 0,
		       wcd9378_get_swr_port, wcd9378_set_swr_port),

	/* TX SoundWire port enable switches */
	SOC_SINGLE_EXT("ADC1 Switch", WCD9378_ADC1, 1, 1, 0,
		       wcd9378_get_swr_port, wcd9378_set_swr_port),
	SOC_SINGLE_EXT("ADC2 Switch", WCD9378_ADC2, 1, 1, 0,
		       wcd9378_get_swr_port, wcd9378_set_swr_port),
	SOC_SINGLE_EXT("ADC3 Switch", WCD9378_ADC3, 1, 1, 0,
		       wcd9378_get_swr_port, wcd9378_set_swr_port),
	SOC_SINGLE_EXT("DMIC0 Switch", WCD9378_DMIC0, 1, 1, 0,
		       wcd9378_get_swr_port, wcd9378_set_swr_port),
	SOC_SINGLE_EXT("DMIC1 Switch", WCD9378_DMIC1, 1, 1, 0,
		       wcd9378_get_swr_port, wcd9378_set_swr_port),
	SOC_SINGLE_EXT("MBHC Switch", WCD9378_MBHC, 1, 1, 0,
		       wcd9378_get_swr_port, wcd9378_set_swr_port),
	SOC_SINGLE_EXT("DMIC2 Switch", WCD9378_DMIC2, 1, 1, 0,
		       wcd9378_get_swr_port, wcd9378_set_swr_port),
	SOC_SINGLE_EXT("DMIC3 Switch", WCD9378_DMIC3, 1, 1, 0,
		       wcd9378_get_swr_port, wcd9378_set_swr_port),
	SOC_SINGLE_EXT("DMIC4 Switch", WCD9378_DMIC4, 1, 1, 0,
		       wcd9378_get_swr_port, wcd9378_set_swr_port),
	SOC_SINGLE_EXT("DMIC5 Switch", WCD9378_DMIC5, 1, 1, 0,
		       wcd9378_get_swr_port, wcd9378_set_swr_port),
	SOC_SINGLE_EXT("DMIC6 Switch", WCD9378_DMIC6, 1, 1, 0,
		       wcd9378_get_swr_port, wcd9378_set_swr_port),
	SOC_SINGLE_EXT("DMIC7 Switch", WCD9378_DMIC7, 1, 1, 0,
		       wcd9378_get_swr_port, wcd9378_set_swr_port),
};

/* ------------------------------------------------------------------ */

/* Patch 7 — DAPM routes                                               */
/* ------------------------------------------------------------------ */
static const struct snd_soc_dapm_route wcd9378_audio_map[] = {
	/* ADC1 path */
	{ "ADC1_OUTPUT", NULL, "ADC1_MIXER" },
	{ "ADC1_MIXER", "Switch", "ADC1" },
	{ "ADC1", NULL, "AMIC1" },

	/* ADC2 path — mux selects AMIC2 or AMIC3 */
	{ "ADC2_OUTPUT", NULL, "ADC2_MIXER" },
	{ "ADC2_MIXER", "Switch", "ADC2" },
	{ "ADC2", NULL, "ADC2 MUX" },
	{ "ADC2 MUX", "INP3", "AMIC3" },
	{ "ADC2 MUX", "INP2", "AMIC2" },

	/* ADC3 path */
	{ "ADC3_OUTPUT", NULL, "ADC3_MIXER" },
	{ "ADC3_MIXER", "Switch", "ADC3" },
	{ "ADC3", NULL, "AMIC3" },

	/* DMIC paths — DMIC0-7 across 2 physical TX SoundWire lanes */
	{ "DMIC0_OUTPUT", NULL, "DMIC0_MIXER" },
	{ "DMIC0_MIXER", "Switch", "DMIC0" },

	{ "DMIC1_OUTPUT", NULL, "DMIC1_MIXER" },
	{ "DMIC1_MIXER", "Switch", "DMIC1" },

	{ "DMIC2_OUTPUT", NULL, "DMIC2_MIXER" },
	{ "DMIC2_MIXER", "Switch", "DMIC2" },

	{ "DMIC3_OUTPUT", NULL, "DMIC3_MIXER" },
	{ "DMIC3_MIXER", "Switch", "DMIC3" },

	{ "DMIC4_OUTPUT", NULL, "DMIC4_MIXER" },
	{ "DMIC4_MIXER", "Switch", "DMIC4" },

	{ "DMIC5_OUTPUT", NULL, "DMIC5_MIXER" },
	{ "DMIC5_MIXER", "Switch", "DMIC5" },

	{ "DMIC6_OUTPUT", NULL, "DMIC6_MIXER" },
	{ "DMIC6_MIXER", "Switch", "DMIC6" },

	{ "DMIC7_OUTPUT", NULL, "DMIC7_MIXER" },
	{ "DMIC7_MIXER", "Switch", "DMIC7" },

	/* HPHL RX path */
	{ "IN1_HPHL", NULL, "VDD_BUCK" },
	{ "IN1_HPHL", NULL, "CLS_H_PORT" },
	{ "RX1", NULL, "IN1_HPHL" },
	{ "HPH SEQUENCER", NULL, "RX1" },
	{ "RDAC1", NULL, "HPH SEQUENCER" },
	{ "HPHL_RDAC", "Switch", "RDAC1" },
	{ "HPHL PGA", NULL, "HPHL_RDAC" },
	{ "HPHL", NULL, "HPHL PGA" },

	/* HPHR RX path */
	{ "IN2_HPHR", NULL, "VDD_BUCK" },
	{ "IN2_HPHR", NULL, "CLS_H_PORT" },
	{ "RX2", NULL, "IN2_HPHR" },
	{ "HPH SEQUENCER", NULL, "RX2" },
	{ "RDAC2", NULL, "HPH SEQUENCER" },
	{ "HPHR_RDAC", "Switch", "RDAC2" },
	{ "HPHR PGA", NULL, "HPHR_RDAC" },
	{ "HPHR", NULL, "HPHR PGA" },

	/* AUX RX path */
	{ "IN3_AUX", NULL, "VDD_BUCK" },
	{ "IN3_AUX", NULL, "CLS_H_PORT" },
	{ "RX3", NULL, "IN3_AUX" },
	{ "RDAC4", NULL, "RX3" },
	{ "AUX_RDAC", "Switch", "RDAC4" },
	{ "AUX PGA", NULL, "AUX_RDAC" },
	{ "AUX", NULL, "AUX PGA" },

	/* EAR RX path — RDAC3 MUX selects RX1 or RX3 */
	{ "RDAC3_MUX", "RX3", "RX3" },
	{ "RDAC3_MUX", "RX1", "RX1" },
	{ "RDAC3", NULL, "RDAC3_MUX" },
	{ "EAR_RDAC", "Switch", "RDAC3" },
	{ "EAR PGA", NULL, "EAR_RDAC" },
	{ "EAR", NULL, "EAR PGA" },

	/* DSD playback — bypass interpolator, feed directly to HPH */
	{ "DSD_L PGA", NULL, "IN_DSD_L" },
	{ "DSD_R PGA", NULL, "IN_DSD_R" },
	{ "HPHL", NULL, "DSD_L PGA" },
	{ "HPHR", NULL, "DSD_R PGA" },
};

/* ------------------------------------------------------------------ */

/* Component probe / remove / set_jack                                 */
/* ------------------------------------------------------------------ */
static int wcd9378_soc_codec_probe(struct snd_soc_component *component)
{
	struct snd_soc_dapm_context *dapm =
		snd_soc_component_to_dapm(component);
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);
	struct sdw_slave *tx_sdw_dev = wcd9378->tx_sdw_dev;
	struct device *dev = component->dev;
	unsigned long time_left;
	int ret;

	dev_err(dev, "WCD-DBG codec_probe: START waiting for TX init_complete\n");
	time_left = wait_for_completion_timeout(
			&tx_sdw_dev->initialization_complete,
			msecs_to_jiffies(15000));
	if (!time_left) {
		dev_err(dev, "soundwire device init timeout\n");
			return -ETIMEDOUT;
	}
	dev_err(dev, "WCD-DBG codec_probe: TX init_complete received time_left=%lu\n",
		time_left);

	snd_soc_component_init_regmap(component, wcd9378->regmap);

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	wcd9378->clsh_info = wcd_clsh_ctrl_alloc(component, WCD9378);
	if (IS_ERR(wcd9378->clsh_info)) {
		pm_runtime_put(dev);
		return PTR_ERR(wcd9378->clsh_info);
	}

	wcd9378_io_init(wcd9378->regmap);

	pm_runtime_put(dev);

	wcd9378->hphr_pdm_wd_int =
		regmap_irq_get_virq(wcd9378->irq_chip,
				    WCD9378_IRQ_HPHR_PDM_WD_INT);
	wcd9378->hphl_pdm_wd_int =
		regmap_irq_get_virq(wcd9378->irq_chip,
				    WCD9378_IRQ_HPHL_PDM_WD_INT);
	wcd9378->aux_pdm_wd_int =
		regmap_irq_get_virq(wcd9378->irq_chip,
				    WCD9378_IRQ_AUX_PDM_WD_INT);

	ret = devm_request_threaded_irq(dev, wcd9378->hphr_pdm_wd_int,
					NULL, wcd9378_wd_handle_irq,
					IRQF_ONESHOT | IRQF_TRIGGER_RISING,
					"HPHR PDM WDOG INT", wcd9378);
	if (ret)
		dev_err(dev, "Failed to request HPHR WD irq (%d)\n", ret);

	ret = devm_request_threaded_irq(dev, wcd9378->hphl_pdm_wd_int,
					NULL, wcd9378_wd_handle_irq,
					IRQF_ONESHOT | IRQF_TRIGGER_RISING,
					"HPHL PDM WDOG INT", wcd9378);
	if (ret)
		dev_err(dev, "Failed to request HPHL WD irq (%d)\n", ret);

	ret = devm_request_threaded_irq(dev, wcd9378->aux_pdm_wd_int,
					NULL, wcd9378_wd_handle_irq,
					IRQF_ONESHOT | IRQF_TRIGGER_RISING,
					"AUX PDM WDOG INT", wcd9378);
	if (ret)
		dev_err(dev, "Failed to request AUX WD irq (%d)\n", ret);

	/* Disable watchdog interrupts until PA is enabled */
	disable_irq_nosync(wcd9378->hphr_pdm_wd_int);
	disable_irq_nosync(wcd9378->hphl_pdm_wd_int);
	disable_irq_nosync(wcd9378->aux_pdm_wd_int);

	snd_soc_dapm_ignore_suspend(dapm, "AMIC1");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC2");
	snd_soc_dapm_ignore_suspend(dapm, "AMIC3");
	snd_soc_dapm_ignore_suspend(dapm, "IN1_HPHL");
	snd_soc_dapm_ignore_suspend(dapm, "IN2_HPHR");
	snd_soc_dapm_ignore_suspend(dapm, "IN3_AUX");
	snd_soc_dapm_ignore_suspend(dapm, "HPHL");
	snd_soc_dapm_ignore_suspend(dapm, "HPHR");
	snd_soc_dapm_ignore_suspend(dapm, "EAR");
	snd_soc_dapm_ignore_suspend(dapm, "AUX");

	snd_soc_dapm_sync(dapm);

	/*
	 * MBHC disabled for initial bringup.
	 * Re-enable: call wcd9378_mbhc_init(component) here.
	 * See context file section 63 for details on the
	 * SDCA_CASCADE / surge detection issue to fix first.
	 */
	return 0;
}

static void wcd9378_soc_codec_remove(struct snd_soc_component *component)
{
	struct wcd9378_priv *wcd9378 = snd_soc_component_get_drvdata(component);

	wcd9378_mbhc_deinit(component);
	free_irq(wcd9378->aux_pdm_wd_int, wcd9378);
	free_irq(wcd9378->hphl_pdm_wd_int, wcd9378);
	free_irq(wcd9378->hphr_pdm_wd_int, wcd9378);
	wcd_clsh_ctrl_free(wcd9378->clsh_info);
}

static int wcd9378_codec_set_jack(struct snd_soc_component *comp,
				  struct snd_soc_jack *jack, void *data)
{
	struct wcd9378_priv *wcd9378 = dev_get_drvdata(comp->dev);
	int ret = 0;

	if (!wcd9378->wcd_mbhc)
		return 0;

	if (jack)
		ret = wcd_mbhc_start(wcd9378->wcd_mbhc,
				     &wcd9378->mbhc_cfg, jack);
	else
		wcd_mbhc_stop(wcd9378->wcd_mbhc);

	return ret;
}

static const struct snd_soc_component_driver soc_codec_dev_wcd9378 = {
	.name		  = "wcd937x_codec",
	.probe		  = wcd9378_soc_codec_probe,
	.remove		  = wcd9378_soc_codec_remove,
	.controls	  = wcd9378_snd_controls,
	.num_controls	  = ARRAY_SIZE(wcd9378_snd_controls),
	.dapm_widgets	  = wcd9378_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(wcd9378_dapm_widgets),
	.dapm_routes	  = wcd9378_audio_map,
	.num_dapm_routes  = ARRAY_SIZE(wcd9378_audio_map),
	.set_jack	  = wcd9378_codec_set_jack,
	.endianness	  = 1,
};

/* ------------------------------------------------------------------ */
/* DAI ops                                                             */
/* ------------------------------------------------------------------ */
static int wcd9378_codec_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params,
				   struct snd_soc_dai *dai)
{
	struct wcd9378_priv *wcd9378 = dev_get_drvdata(dai->dev);
	struct wcd9378_sdw_priv *wcd = wcd9378->sdw_priv[dai->id];

	return wcd9378_sdw_hw_params(wcd, substream, params, dai);
}

static int wcd9378_codec_free(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct wcd9378_priv *wcd9378 = dev_get_drvdata(dai->dev);
	struct wcd9378_sdw_priv *wcd = wcd9378->sdw_priv[dai->id];

	return sdw_stream_remove_slave(wcd->sdev, wcd->sruntime);
}

static int wcd9378_codec_set_sdw_stream(struct snd_soc_dai *dai,
					void *stream, int direction)
{
	struct wcd9378_priv *wcd9378 = dev_get_drvdata(dai->dev);
	struct wcd9378_sdw_priv *wcd = wcd9378->sdw_priv[dai->id];

	wcd->sruntime = stream;

	return 0;
}

static int wcd9378_get_channel_map(const struct snd_soc_dai *dai,
				   unsigned int *tx_num, unsigned int *tx_slot,
				   unsigned int *rx_num, unsigned int *rx_slot)
{
	struct wcd9378_priv *wcd9378 = dev_get_drvdata(dai->dev);
	struct wcd9378_sdw_priv *wcd = wcd9378->sdw_priv[dai->id];
	int i;

	switch (dai->id) {
	case AIF1_PB:
		if (!rx_slot || !rx_num) {
			dev_err(dai->dev, "Invalid rx_slot %p or rx_num %p\n",
				rx_slot, rx_num);
			return -EINVAL;
		}
		for (i = 0; i < SDW_MAX_PORTS; i++)
			rx_slot[i] = wcd->master_channel_map[i];
		*rx_num = i;
		break;
	case AIF1_CAP:
		if (!tx_slot || !tx_num) {
			dev_err(dai->dev, "Invalid tx_slot %p or tx_num %p\n",
				tx_slot, tx_num);
			return -EINVAL;
		}
		for (i = 0; i < SDW_MAX_PORTS; i++)
			tx_slot[i] = wcd->master_channel_map[i];
		*tx_num = i;
		break;
	default:
		break;
	}

	return 0;
}

static const struct snd_soc_dai_ops wcd9378_sdw_dai_ops = {
	.hw_params	 = wcd9378_codec_hw_params,
	.hw_free	 = wcd9378_codec_free,
	.set_stream	 = wcd9378_codec_set_sdw_stream,
	.get_channel_map = wcd9378_get_channel_map,
};

static struct snd_soc_dai_driver wcd9378_dais[] = {
	[0] = {
		.name = "wcd9378-sdw-rx",
		.playback = {
			.stream_name = "WCD9378 AIF Playback",
			.rates = WCD9378_RATES | WCD9378_FRAC_RATES,
			.formats = WCD9378_FORMATS,
			.rate_min = 8000,
			.rate_max = 384000,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &wcd9378_sdw_dai_ops,
	},
	[1] = {
		.name = "wcd9378-sdw-tx",
		.capture = {
			.stream_name = "WCD9378 AIF Capture",
			.rates = WCD9378_RATES | WCD9378_FRAC_RATES,
			.formats = WCD9378_FORMATS,
			.rate_min = 8000,
			.rate_max = 384000,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &wcd9378_sdw_dai_ops,
	},
};

/* ------------------------------------------------------------------ */
/* Component master bind / unbind                                      */
/* ------------------------------------------------------------------ */
static int wcd9378_bind(struct device *dev)
{
	struct wcd9378_priv *wcd9378 = dev_get_drvdata(dev);
	int ret;

	/* Give SDW subdevices time to settle */
	usleep_range(5000, 5010);

	ret = component_bind_all(dev, wcd9378);
	if (ret) {
		dev_err(dev, "Slave bind failed, ret = %d\n", ret);
		return ret;
	}

	wcd9378->rxdev = of_sdw_find_device_by_node(wcd9378->rxnode);
	if (!wcd9378->rxdev) {
		dev_err(dev, "could not find slave with matching of node\n");
		ret = -EINVAL;
		goto err_component_unbind;
	}

	wcd9378->sdw_priv[AIF1_PB] = dev_get_drvdata(wcd9378->rxdev);
	wcd9378->sdw_priv[AIF1_PB]->wcd9378 = wcd9378;
	wcd9378->sdw_priv[AIF1_PB]->codec_dev = dev;

	wcd9378->txdev = of_sdw_find_device_by_node(wcd9378->txnode);
	if (!wcd9378->txdev) {
		dev_err(dev, "could not find txslave with matching of node\n");
		ret = -EINVAL;
		goto err_put_rxdev;
	}

	wcd9378->sdw_priv[AIF1_CAP] = dev_get_drvdata(wcd9378->txdev);
	wcd9378->sdw_priv[AIF1_CAP]->wcd9378 = wcd9378;
	wcd9378->tx_sdw_dev = dev_to_sdw_dev(wcd9378->txdev);
	wcd9378->sdw_priv[AIF1_CAP]->codec_dev = dev;

	/*
	 * TX is the main CSR register interface and must not be suspended
	 * before RX. Add explicit device link to enforce ordering.
	 */
	if (!device_link_add(wcd9378->rxdev, wcd9378->txdev,
			     DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME)) {
		dev_err(dev, "Could not devlink TX and RX\n");
		ret = -EINVAL;
		goto err_put_txdev;
	}

	if (!device_link_add(dev, wcd9378->txdev,
			     DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME)) {
		dev_err(dev, "Could not devlink WCD and TX\n");
		ret = -EINVAL;
		goto err_remove_link1;
	}

	if (!device_link_add(dev, wcd9378->rxdev,
			     DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME)) {
		dev_err(dev, "Could not devlink WCD and RX\n");
		ret = -EINVAL;
		goto err_remove_link2;
	}

	wcd9378->regmap = wcd9378->sdw_priv[AIF1_CAP]->regmap;
	if (!wcd9378->regmap) {
		dev_err(dev, "could not get TX device regmap\n");
		ret = -EINVAL;
		goto err_remove_link3;
	}

	ret = wcd9378_irq_init(wcd9378, dev);
	if (ret) {
		dev_err(dev, "IRQ init failed: %d\n", ret);
		goto err_remove_link3;
	}

	wcd9378->sdw_priv[AIF1_PB]->slave_irq = wcd9378->virq;
	wcd9378->sdw_priv[AIF1_CAP]->slave_irq = wcd9378->virq;

	wcd9378_set_micbias_data(dev, wcd9378);

	ret = snd_soc_register_component(dev, &soc_codec_dev_wcd9378,
					 wcd9378_dais,
					 ARRAY_SIZE(wcd9378_dais));
	if (ret) {
		dev_err(dev, "Codec registration failed\n");
		goto err_remove_link3;
	}

	return ret;

err_remove_link3:
	device_link_remove(dev, wcd9378->rxdev);
err_remove_link2:
	device_link_remove(dev, wcd9378->txdev);
err_remove_link1:
	device_link_remove(wcd9378->rxdev, wcd9378->txdev);
err_put_txdev:
	put_device(wcd9378->txdev);
err_put_rxdev:
	put_device(wcd9378->rxdev);
err_component_unbind:
	component_unbind_all(dev, wcd9378);
	return ret;
}

static void wcd9378_unbind(struct device *dev)
{
	struct wcd9378_priv *wcd9378 = dev_get_drvdata(dev);

	snd_soc_unregister_component(dev);
	device_link_remove(dev, wcd9378->txdev);
	device_link_remove(dev, wcd9378->rxdev);
	device_link_remove(wcd9378->rxdev, wcd9378->txdev);
	component_unbind_all(dev, wcd9378);
	mutex_destroy(&wcd9378->micb_lock);
	put_device(wcd9378->txdev);
	put_device(wcd9378->rxdev);
}

static const struct component_master_ops wcd9378_comp_ops = {
	.bind   = wcd9378_bind,
	.unbind = wcd9378_unbind,
};

static int wcd9378_add_slave_components(struct wcd9378_priv *wcd9378,
					struct device *dev,
					struct component_match **matchptr)
{
	struct device_node *np = dev->of_node;

	wcd9378->rxnode = of_parse_phandle(np, "qcom,rx-device", 0);
	if (!wcd9378->rxnode) {
		dev_err(dev, "Couldn't parse phandle to qcom,rx-device!\n");
		return -ENODEV;
	}
	component_match_add_release(dev, matchptr, component_release_of,
				    component_compare_of, wcd9378->rxnode);

	wcd9378->txnode = of_parse_phandle(np, "qcom,tx-device", 0);
	if (!wcd9378->txnode) {
		dev_err(dev, "Couldn't parse phandle to qcom,tx-device\n");
		return -ENODEV;
	}
	component_match_add_release(dev, matchptr, component_release_of,
				    component_compare_of, wcd9378->txnode);

	return 0;
}

/* ------------------------------------------------------------------ */
/* Platform driver probe / remove                                      */
/* ------------------------------------------------------------------ */
static int wcd9378_probe(struct platform_device *pdev)
{
	struct component_match *match = NULL;
	struct device *dev = &pdev->dev;
	struct wcd9378_priv *wcd9378;
	struct wcd_mbhc_config *cfg;
	int ret;

	wcd9378 = devm_kzalloc(dev, sizeof(*wcd9378), GFP_KERNEL);
	if (!wcd9378)
		return -ENOMEM;

	dev_set_drvdata(dev, wcd9378);
	mutex_init(&wcd9378->micb_lock);
	wcd9378->common.dev = dev;
	wcd9378->common.max_bias = WCD9378_MAX_MICBIAS;

	wcd9378->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(wcd9378->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(wcd9378->reset_gpio),
				     "failed to get reset gpio\n");

	wcd9378->us_euro_gpio = devm_gpiod_get_optional(dev, "us-euro",
							GPIOD_OUT_LOW);
	if (IS_ERR(wcd9378->us_euro_gpio))
		return dev_err_probe(dev, PTR_ERR(wcd9378->us_euro_gpio),
				     "us-euro swap GPIO not found\n");

	cfg = &wcd9378->mbhc_cfg;
	cfg->swap_gnd_mic = wcd9378_swap_gnd_mic;

	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(wcd9378_supplies),
					     wcd9378_supplies);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to get and enable supplies\n");

	ret = wcd_dt_parse_micbias_info(&wcd9378->common);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get micbias\n");

	cfg->mbhc_micbias = MIC_BIAS_2;
	cfg->anc_micbias  = MIC_BIAS_2;
	cfg->v_hs_max	  = WCD_MBHC_HS_V_MAX;
	cfg->num_btn	  = WCD9378_MBHC_MAX_BUTTONS;
	cfg->micb_mv	  = wcd9378->common.micb_mv[1];
	cfg->linein_th	  = 5000;
	cfg->hs_thr	  = 1700;
	cfg->hph_thr	  = 50;
	cfg->moist_rref	  = WCD9378_MBHC_MOISTURE_RREF;

	wcd_dt_parse_mbhc_data(dev, &wcd9378->mbhc_cfg);

	ret = wcd9378_add_slave_components(wcd9378, dev, &match);
	if (ret)
		return ret;

	/*
	 * Reset the codec before registering the component master.
	 * The SWR master's qcom_swrm_init() resets the SWR hardware,
	 * which causes the codec slave to see a new SWR clock and
	 * enumerate. The probe poll loop in qcom_swrm_probe() detects
	 * this and signals slave->initialization_complete.
	 */
	wcd9378_reset(wcd9378);
	usleep_range(5000, 5010);


	ret = component_master_add_with_match(dev, &wcd9378_comp_ops, match);
	if (ret)
		return ret;

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;
}

static void wcd9378_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	component_master_del(dev, &wcd9378_comp_ops);
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_dont_use_autosuspend(dev);
}

static const struct of_device_id wcd9378_of_match[] = {
	{ .compatible = "qcom,wcd9378-codec" },
	{ }
};
MODULE_DEVICE_TABLE(of, wcd9378_of_match);

static struct platform_driver wcd9378_codec_driver = {
	.probe  = wcd9378_probe,
	.remove = wcd9378_remove,
	.driver = {
		.name		  = "wcd9378_codec",
		.of_match_table	  = wcd9378_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(wcd9378_codec_driver);

MODULE_DESCRIPTION("WCD9378 Codec driver");
MODULE_LICENSE("GPL");

//*********************************************************************************
// Generated by SmartRF Studio version 2.9.0 (build #168)
// Tested for SimpleLink SDK version: CC13x0 SDK 2.10.xx.xx
// Device: CC1350 (STK) Rev. 2.1
//
//*********************************************************************************


#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_hs_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_common_cmd.h)
#include DeviceFamily_constructPath(driverlib/rf_hs_cmd.h)
#include DeviceFamily_constructPath(driverlib/rf_prop_cmd.h)
#include DeviceFamily_constructPath(rf_patches/rf_patch_cpe_genfsk.h)
#include DeviceFamily_constructPath(rf_patches/rf_patch_rfe_genfsk.h)
#include DeviceFamily_constructPath(rf_patches/rf_patch_cpe_lrm.h)
#include DeviceFamily_constructPath(rf_patches/rf_patch_rfe_lrm.h)
#include DeviceFamily_constructPath(rf_patches/rf_patch_cpe_genook.h)
#include DeviceFamily_constructPath(rf_patches/rf_patch_mce_genook.h)
#include DeviceFamily_constructPath(rf_patches/rf_patch_rfe_genook.h)
#include DeviceFamily_constructPath(rf_patches/rf_patch_mce_hsp_4mbps.h)
#include DeviceFamily_constructPath(rf_patches/rf_patch_rfe_hsp_4mbps.h)
#include DeviceFamily_constructPath(rf_patches/rf_patch_cpe_sl_longrange.h)
#include DeviceFamily_constructPath(rf_patches/rf_patch_mce_sl_longrange.h)
#include DeviceFamily_constructPath(rf_patches/rf_patch_rfe_sl_longrange.h)

#include <ti/drivers/rf/RF.h>
#include "smartrf_settings_predefined.h"

// TI-RTOS RF Mode Object
RF_Mode RF_prop_fsk =
{
    .rfMode = RF_MODE_PROPRIETARY_SUB_1,
    .cpePatchFxn = &rf_patch_cpe_genfsk,
    .mcePatchFxn = 0,
    .rfePatchFxn = &rf_patch_rfe_genfsk,
};

RF_Mode RF_prop_lrm =
{
    .rfMode = RF_MODE_PROPRIETARY_SUB_1,
    .cpePatchFxn = &rf_patch_cpe_lrm,
    .mcePatchFxn = 0,
    .rfePatchFxn = &rf_patch_rfe_lrm,
};

RF_Mode RF_prop_ook =
{
    .rfMode = RF_MODE_PROPRIETARY_SUB_1,
    .cpePatchFxn = &rf_patch_cpe_genook,
    .mcePatchFxn = &rf_patch_mce_genook,
    .rfePatchFxn = &rf_patch_rfe_genook,
};

RF_Mode RF_prop_hsm =
{
    .rfMode = RF_MODE_PROPRIETARY_SUB_1,
    .cpePatchFxn =  0,
    .mcePatchFxn =  &rf_patch_mce_hsp_4mbps,
    .rfePatchFxn =  &rf_patch_rfe_hsp_4mbps,
};

RF_Mode RF_prop_sl_lr =
{
    .rfMode = RF_MODE_PROPRIETARY_SUB_1,
    .cpePatchFxn =  &rf_patch_cpe_sl_longrange,
    .mcePatchFxn =  &rf_patch_mce_sl_longrange,
    .rfePatchFxn =  &rf_patch_rfe_sl_longrange,
};

// TX Power table
// The RF_TxPowerTable_DEFAULT_PA_ENTRY macro is defined in RF.h and requires the following arguments:
// RF_TxPowerTable_DEFAULT_PA_ENTRY(bias, gain, boost coefficient)
// See the Technical Reference Manual for further details about the "txPower" Command field.
// The PA settings require the CCFG_FORCE_VDDR_HH = 0 unless stated otherwise.
const RF_TxPowerTable_Entry PROP_RF_txPowerTable[] = 
{ 
    {-10, RF_TxPowerTable_DEFAULT_PA_ENTRY(0, 3, 0, 2) },
    {0, RF_TxPowerTable_DEFAULT_PA_ENTRY(3, 3, 0, 9) },
    {1, RF_TxPowerTable_DEFAULT_PA_ENTRY(4, 3, 0, 11) },
    {2, RF_TxPowerTable_DEFAULT_PA_ENTRY(5, 3, 0, 12) },
    {3, RF_TxPowerTable_DEFAULT_PA_ENTRY(6, 3, 0, 14) },
    {4, RF_TxPowerTable_DEFAULT_PA_ENTRY(4, 1, 0, 12) },
    {5, RF_TxPowerTable_DEFAULT_PA_ENTRY(8, 3, 0, 16) },
    {6, RF_TxPowerTable_DEFAULT_PA_ENTRY(9, 3, 0, 18) },
    {7, RF_TxPowerTable_DEFAULT_PA_ENTRY(11, 3, 0, 21) },
    {8, RF_TxPowerTable_DEFAULT_PA_ENTRY(14, 3, 0, 25) },
    {9, RF_TxPowerTable_DEFAULT_PA_ENTRY(18, 3, 0, 32) },
    {10, RF_TxPowerTable_DEFAULT_PA_ENTRY(24, 3, 0, 44) },
    {11, RF_TxPowerTable_DEFAULT_PA_ENTRY(37, 3, 0, 72) },
    {12, RF_TxPowerTable_DEFAULT_PA_ENTRY(43, 0, 0, 94) },
    {14, RF_TxPowerTable_DEFAULT_PA_ENTRY(63, 0, 1, 85) }, // This setting requires CCFG_FORCE_VDDR_HH = 1.
    RF_TxPowerTable_TERMINATION_ENTRY
};

const uint8_t PROP_RF_txPowerTableSize = sizeof(PROP_RF_txPowerTable)/sizeof(RF_TxPowerTable_Entry);

// Overrides for CMD_PROP_RADIO_DIV_SETUP FSK
uint32_t pOverrides_fsk[] =
{
    // override_use_patch_prop_genfsk.xml
    // PHY: Use MCE ROM bank 4, RFE RAM patch
    MCE_RFE_OVERRIDE(0,4,0,1,0,0),
    // override_synth_prop_863_930_div5.xml
    // Synth: Set recommended RTRIM to 7
    HW_REG_OVERRIDE(0x4038,0x0037),
    // Synth: Set Fref to 4 MHz
    (uint32_t)0x000684A3,
    // Synth: Configure fine calibration setting
    HW_REG_OVERRIDE(0x4020,0x7F00),
    // Synth: Configure fine calibration setting
    HW_REG_OVERRIDE(0x4064,0x0040),
    // Synth: Configure fine calibration setting
    (uint32_t)0xB1070503,
    // Synth: Configure fine calibration setting
    (uint32_t)0x05330523,
    // Synth: Set loop bandwidth after lock to 20 kHz
    (uint32_t)0x0A480583,
    // Synth: Set loop bandwidth after lock to 20 kHz
    (uint32_t)0x7AB80603,
    // Synth: Configure VCO LDO (in ADI1, set VCOLDOCFG=0x9F to use voltage input reference)
    ADI_REG_OVERRIDE(1,4,0x9F),
    // Synth: Configure synth LDO (in ADI1, set SLDOCTL0.COMP_CAP=1)
    ADI_HALFREG_OVERRIDE(1,7,0x4,0x4),
    // Synth: Use 24 MHz XOSC as synth clock, enable extra PLL filtering
    (uint32_t)0x02010403,
    // Synth: Configure extra PLL filtering
    (uint32_t)0x00108463,
    // Synth: Increase synth programming timeout (0x04B0 RAT ticks = 300 us)
    (uint32_t)0x04B00243,
    // override_phy_rx_aaf_bw_0xd.xml
    // Rx: Set anti-aliasing filter bandwidth to 0xD (in ADI0, set IFAMPCTL3[7:4]=0xD)
    ADI_HALFREG_OVERRIDE(0,61,0xF,0xD),
    // override_phy_gfsk_rx.xml
    // Rx: Set LNA bias current trim offset to 1
    (uint32_t)0x00018883,
    // Rx: Freeze RSSI on sync found event
    HW_REG_OVERRIDE(0x6084,0x35F1),
    // override_phy_gfsk_pa_ramp_agc_reflevel_0x1a.xml
    // Tx: Configure PA ramping setting (0x41). Rx: Set AGC reference level to 0x1A.
    HW_REG_OVERRIDE(0x6088,0x411A),
    // Tx: Configure PA ramping setting
    HW_REG_OVERRIDE(0x608C,0x8213),
    // override_phy_rx_rssi_offset_10db.xml
    // Rx: Set RSSI offset to adjust reported RSSI by +10 dB
    (uint32_t)0x00F688A3,
    // TX power override
    // Tx: Set PA trim to max (in ADI0, set PACTL0=0xF8)
    ADI_REG_OVERRIDE(0,12,0xF8),
    ADI_HALFREG_OVERRIDE(0, 16, 0x7, 2),
    (uint32_t)0xFFFFFFFF,
};

// Overrides for CMD_PROP_RADIO_DIV_SETUP LRM
uint32_t pOverrides_lrm[] =
{
    // override_use_patch_prop_lrm.xml
    // PHY: Use MCE ROM bank 3, RFE RAM patch
    MCE_RFE_OVERRIDE(0,3,0,1,0,0),
    // override_synth_prop_863_930_div5.xml
    // Synth: Set recommended RTRIM to 7
    HW_REG_OVERRIDE(0x4038,0x0037),
    // Synth: Set Fref to 4 MHz
    (uint32_t)0x000684A3,
    // Synth: Configure fine calibration setting
    HW_REG_OVERRIDE(0x4020,0x7F00),
    // Synth: Configure fine calibration setting
    HW_REG_OVERRIDE(0x4064,0x0040),
    // Synth: Configure fine calibration setting
    (uint32_t)0xB1070503,
    // Synth: Configure fine calibration setting
    (uint32_t)0x05330523,
    // Synth: Set loop bandwidth after lock to 20 kHz
    (uint32_t)0x0A480583,
    // Synth: Set loop bandwidth after lock to 20 kHz
    (uint32_t)0x7AB80603,
    // Synth: Configure VCO LDO (in ADI1, set VCOLDOCFG=0x9F to use voltage input reference)
    ADI_REG_OVERRIDE(1,4,0x9F),
    // Synth: Configure synth LDO (in ADI1, set SLDOCTL0.COMP_CAP=1)
    ADI_HALFREG_OVERRIDE(1,7,0x4,0x4),
    // Synth: Use 24 MHz XOSC as synth clock, enable extra PLL filtering
    (uint32_t)0x02010403,
    // Synth: Configure extra PLL filtering
    (uint32_t)0x00108463,
    // Synth: Increase synth programming timeout (0x04B0 RAT ticks = 300 us)
    (uint32_t)0x04B00243,
    // override_phy_rx_aaf_bw_0xd.xml
    // Rx: Set anti-aliasing filter bandwidth to 0xD (in ADI0, set IFAMPCTL3[7:4]=0xD)
    ADI_HALFREG_OVERRIDE(0,61,0xF,0xD),
    // override_phy_gfsk_rx.xml
    // Rx: Set LNA bias current trim offset to 1
    (uint32_t)0x00018883,
    // Rx: Freeze RSSI on sync found event
    HW_REG_OVERRIDE(0x6084,0x35F1),
    // override_phy_gfsk_pa_ramp_agc_reflevel_0x1a.xml
    // Tx: Configure PA ramping setting (0x41). Rx: Set AGC reference level to 0x1A.
    HW_REG_OVERRIDE(0x6088,0x411A),
    // Tx: Configure PA ramping setting
    HW_REG_OVERRIDE(0x608C,0x8213),
    // override_phy_lrm_rom_dsss8.xml
    // PHY: Configure DSSS=8
    HW_REG_OVERRIDE(0x505C,0x073C),
    // override_phy_rx_rssi_offset_10db.xml
    // Rx: Set RSSI offset to adjust reported RSSI by +10 dB
    (uint32_t)0x00F688A3,
    // TX power override
    // Tx: Set PA trim to max (in ADI0, set PACTL0=0xF8)
    ADI_REG_OVERRIDE(0,12,0xF8),
    ADI_HALFREG_OVERRIDE(0, 16, 0x7, 2),
    (uint32_t)0xFFFFFFFF,
};

// Overrides for CMD_PROP_RADIO_DIV_SETUP OOK
uint32_t pOverrides_ook[] =
{
    // override_use_patch_prop_genook_nrz.xml
    // PHY: Use MCE RAM patch, RFE RAM patch
    MCE_RFE_OVERRIDE(1,0,0,1,0,0),
    // override_synth_prop_863_930_div5.xml
    // Synth: Set recommended RTRIM to 7
    HW_REG_OVERRIDE(0x4038,0x0037),
    // Synth: Set Fref to 4 MHz
    (uint32_t)0x000684A3,
    // Synth: Configure fine calibration setting
    HW_REG_OVERRIDE(0x4020,0x7F00),
    // Synth: Configure fine calibration setting
    HW_REG_OVERRIDE(0x4064,0x0040),
    // Synth: Configure fine calibration setting
    (uint32_t)0xB1070503,
    // Synth: Configure fine calibration setting
    (uint32_t)0x05330523,
    // Synth: Set loop bandwidth after lock to 20 kHz
    (uint32_t)0x0A480583,
    // Synth: Set loop bandwidth after lock to 20 kHz
    (uint32_t)0x7AB80603,
    // Synth: Configure VCO LDO (in ADI1, set VCOLDOCFG=0x9F to use voltage input reference)
    ADI_REG_OVERRIDE(1,4,0x9F),
    // Synth: Configure synth LDO (in ADI1, set SLDOCTL0.COMP_CAP=1)
    ADI_HALFREG_OVERRIDE(1,7,0x4,0x4),
    // Synth: Use 24 MHz XOSC as synth clock, enable extra PLL filtering
    (uint32_t)0x02010403,
    // Synth: Configure extra PLL filtering
    (uint32_t)0x00108463,
    // Synth: Increase synth programming timeout (0x04B0 RAT ticks = 300 us)
    (uint32_t)0x04B00243,
    // override_phy_rx_aaf_bw_0xd.xml
    // Rx: Set anti-aliasing filter bandwidth to 0xD (in ADI0, set IFAMPCTL3[7:4]=0xD)
    ADI_HALFREG_OVERRIDE(0,61,0xF,0xD),
    // override_phy_agc_reflevel_0x19.xml
    // Rx: Set AGC reference level to 0x19
    HW_REG_OVERRIDE(0x6088,0x0019),
    // override_phy_ook_rx.xml
    // Rx: Set LNA bias current trim offset to 1
    (uint32_t)0x00018883,
    // Rx: Freeze RSSI on sync found event
    HW_REG_OVERRIDE(0x6084,0x35F1),
    // override_phy_ook_tx_symbol_4_8kbaud.xml
    // Tx: Set symbol duty-cycle delay before symbol ramp-down to 0x78 (=120). This means symbol ramp down will begin after reaching (T_symbol/2) plus wait a delay of (120/2)=60 us.
    HW_REG_OVERRIDE(0x52B8,0x8078),
    // override_phy_ook_rx_filter_iir_k_1div4.xml
    // Rx: Set data filter to IIR, k=1/4. Explanation: 0x0000: k=1 (no filter), 0x0001: k=1/2, 0x0002: k=1/4, 0x0003: k=1/8.
    HW_REG_OVERRIDE(0x5204,0x0002),
    // override_phy_ook_tx_power_max.xml
    // Tx: Ramp symbol shape to maximum PA level (0x7200). Explanation: min power=0x6100, ..., max power=0x7200. Bits [15:13] sets wait delay per PA ramp level. Bits[12:8] sets number of PA levels to use from ramp LUT (range 1-18). Bits[7:0] reserved.
    HW_REG_OVERRIDE(0x6098,0x7200),
    // override_phy_rx_rssi_offset_10db.xml
    // Rx: Set RSSI offset to adjust reported RSSI by +10 dB
    (uint32_t)0x00F688A3,
    ADI_HALFREG_OVERRIDE(0, 16, 0x7, 2),
    (uint32_t)0xFFFFFFFF,
};

// Overrides for CMD_RADIO_SETUP HSM
uint32_t shapeovr[] = {0x00000000, 0x00000000, 0x00000000, 0x12010000, 0x72685C43, 0x8986817A};

uint32_t pOverrides_hsm[] =
{
    MCE_RFE_OVERRIDE(1,0,0,1,0,0),
    ADI_HALFREG_OVERRIDE(0,61,0xF,0x0),
    ADI_REG_OVERRIDE(1,4,0x9F),
    ADI_HALFREG_OVERRIDE(1,7,0x4,0x4),
    HW_REG_OVERRIDE(0x4038,0x003A),
    HW_REG_OVERRIDE(0x4020,0x7F00),
    HW_REG_OVERRIDE(0x4064,0x0040),
    0x000604A3,
    0xB1070503,
    0x05330523,
    0x0A480583,
    0x7AB80603,
    0x00108463,
    0x02010403,
    0x04B00243,
    0x00038883,
    0xC0040031,
    (uint32_t) &shapeovr[0],
    0xC0040021,
    (uint32_t) (0x00000035),
    0x000388A3,
    HW_REG_OVERRIDE(0x50B4,0x6666),
    HW_REG_OVERRIDE(0x50B8,0x000C),
    ADI_HALFREG_OVERRIDE(0, 16, 0x7, 2),
    (uint32_t)0xFFFFFFFF,
};

// Overrides for CMD_RADIO_DIV_SETUP SIMPLELINK LONGRANGE
uint32_t pOverrides_sl_lr[] =
{
    // override_use_patch_simplelink_long_range.xml
    // PHY: Use MCE RAM patch, RFE RAM patch
    MCE_RFE_OVERRIDE(1,0,0,1,0,0),
    // override_synth_prop_863_930_div5_lbw60k.xml
    // Synth: Set recommended RTRIM to 7
    HW_REG_OVERRIDE(0x4038,0x0037),
    // Synth: Set Fref to 4 MHz
    (uint32_t)0x000684A3,
    // Synth: Configure fine calibration setting
    HW_REG_OVERRIDE(0x4020,0x7F00),
    // Synth: Configure fine calibration setting
    HW_REG_OVERRIDE(0x4064,0x0040),
    // Synth: Configure fine calibration setting
    (uint32_t)0xB1070503,
    // Synth: Configure fine calibration setting
    (uint32_t)0x05330523,
    // Synth: Set loop bandwidth after lock to 60 kHz
    (uint32_t)0x40410583,
    // Synth: Set loop bandwidth after lock to 60 kHz
    (uint32_t)0x32CC0603,
    // Synth: Set loop bandwidth after lock to 60 kHz
    (uint32_t)0x00010623,
    // Synth: Configure VCO LDO (in ADI1, set VCOLDOCFG=0x9F to use voltage input reference)
    ADI_REG_OVERRIDE(1,4,0x9F),
    // Synth: Configure synth LDO (in ADI1, set SLDOCTL0.COMP_CAP=1)
    ADI_HALFREG_OVERRIDE(1,7,0x4,0x4),
    // Synth: Use 24 MHz XOSC as synth clock, enable extra PLL filtering
    (uint32_t)0x02010403,
    // Synth: Configure extra PLL filtering
    (uint32_t)0x00108463,
    // Synth: Increase synth programming timeout (0x04B0 RAT ticks = 300 us)
    (uint32_t)0x04B00243,
    // override_synth_disable_bias_div5.xml
    // Synth: Set divider bias to disabled
    HW32_ARRAY_OVERRIDE(0x405C,1),
    // Synth: Set divider bias to disabled (specific for loDivider=5)
    (uint32_t)0x18000200,
    // override_phy_rx_aaf_bw_0xd.xml
    // Rx: Set anti-aliasing filter bandwidth to 0xD (in ADI0, set IFAMPCTL3[7:4]=0xD)
    ADI_HALFREG_OVERRIDE(0,61,0xF,0xD),
    // override_phy_gfsk_rx.xml
    // Rx: Set LNA bias current trim offset to 1
    (uint32_t)0x00018883,
    // Rx: Freeze RSSI on sync found event
    HW_REG_OVERRIDE(0x6084,0x35F1),
    // override_phy_gfsk_pa_ramp_agc_reflevel_0x14.xml
    // Tx: Configure PA ramping setting (0x41). Rx: Set AGC reference level to 0x14.
    HW_REG_OVERRIDE(0x6088,0x4114),
    // Tx: Configure PA ramping setting
    HW_REG_OVERRIDE(0x608C,0x8213),
    // override_phy_long_range_dsss2.xml
    // PHY: Configure DSSS SF=2
    HW_REG_OVERRIDE(0x505C,0x0100),
    // override_phy_rx_rssi_offset_10db.xml
    // Rx: Set RSSI offset to adjust reported RSSI by +10 dB
    (uint32_t)0x00F688A3,
    // TX power override
    // Tx: Set PA trim to max (in ADI0, set PACTL0=0xF8)
    ADI_REG_OVERRIDE(0,12,0xF8),
    ADI_HALFREG_OVERRIDE(0, 16, 0x7, 2),
    (uint32_t)0xFFFFFFFF,
};

// CMD_PROP_RADIO_DIV_SETUP FSK
rfc_CMD_PROP_RADIO_DIV_SETUP_t RF_cmdPropRadioDivSetup_fsk =
{
    .commandNo = 0x3807,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .modulation.modType = 0x1,
    .modulation.deviation = 0x64,
    .symbolRate.preScale = 0xF,
    .symbolRate.rateWord = 0x8000,
    .rxBw = 0x24,
    .preamConf.nPreamBytes = 0x4,
    .preamConf.preamMode = 0x0,
    .formatConf.nSwBits = 0x20,
    .formatConf.bBitReversal = 0x0,
    .formatConf.bMsbFirst = 0x1,
    .formatConf.fecMode = 0x0,
    .formatConf.whitenMode = 0x0,
    .config.frontEndMode = 0x2,
    .config.biasMode = 0x1,
    .config.analogCfgMode = 0x0,
    .config.bNoFsPowerUp = 0x0,
    .txPower = 0xAB3F,
    .pRegOverride = pOverrides_fsk,
    .centerFreq = 0x0364,
    .intFreq = 0x8000,
    .loDivider = 0x05,
};

// CMD_PROP_RADIO_DIV_SETUP LRM
rfc_CMD_PROP_RADIO_DIV_SETUP_t RF_cmdPropRadioDivSetup_lrm =
{
    .commandNo = 0x3807,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .modulation.modType = 0x0,
    .modulation.deviation = 0x14,
    .symbolRate.preScale = 0xF,
    .symbolRate.rateWord = 0x199A,
    .rxBw = 0x20,
    .preamConf.nPreamBytes = 0x5,
    .preamConf.preamMode = 0x0,
    .formatConf.nSwBits = 0x20,
    .formatConf.bBitReversal = 0x0,
    .formatConf.bMsbFirst = 0x0,
    .formatConf.fecMode = 0x8,
    .formatConf.whitenMode = 0x0,
    .config.frontEndMode = 0x2,
    .config.biasMode = 0x1,
    .config.analogCfgMode = 0x0,
    .config.bNoFsPowerUp = 0x0,
    .txPower = 0xAB3F,
    .pRegOverride = pOverrides_lrm,
    .centerFreq = 0x0364,
    .intFreq = 0x8000,
    .loDivider = 0x05,
};

// CMD_PROP_RADIO_DIV_SETUP OOK
rfc_CMD_PROP_RADIO_DIV_SETUP_t RF_cmdPropRadioDivSetup_ook =
{
    .commandNo = 0x3807,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .modulation.modType = 0x2,
    .modulation.deviation = 0x0,
    .symbolRate.preScale = 0xF,
    .symbolRate.rateWord = 0xC4A,
    .rxBw = 0x20,
    .preamConf.nPreamBytes = 0x4,
    .preamConf.preamMode = 0x0,
    .formatConf.nSwBits = 0x20,
    .formatConf.bBitReversal = 0x0,
    .formatConf.bMsbFirst = 0x1,
    .formatConf.fecMode = 0x0,
    .formatConf.whitenMode = 0x1,
    .config.frontEndMode = 0x2,
    .config.biasMode = 0x1,
    .config.analogCfgMode = 0x0,
    .config.bNoFsPowerUp = 0x0,
    .txPower = 0xAB3F,
    .pRegOverride = pOverrides_ook,
    .centerFreq = 0x0364,
    .intFreq = 0x8000,
    .loDivider = 0x05,
};

// CMD_RADIO_SETUP HSM
rfc_CMD_RADIO_SETUP_t RF_cmdRadioSetup_hsm =
{
    .commandNo = CMD_RADIO_SETUP,
    .status = 0x0000,
    .pNextOp = 0x00000000,
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .mode = 0x05,
    .loDivider = 5,
    .config.frontEndMode = 0x2,
    .config.biasMode = 0x1,
    .config.bNoFsPowerUp = 0,
    .txPower = 0x23F,
    .pRegOverride = pOverrides_hsm,
};

// CMD_PROP_RADIO_DIV_SETUP SIMPLELINK LONGRANGE
rfc_CMD_PROP_RADIO_DIV_SETUP_t RF_cmdPropRadioDivSetup_sl_lr =
{
    .commandNo = 0x3807,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .modulation.modType = 0x1,
    .modulation.deviation = 0x14,
    .symbolRate.preScale = 0xF,
    .symbolRate.rateWord = 0x3333,
    .rxBw = 0x21,
    .preamConf.nPreamBytes = 0x2,
    .preamConf.preamMode = 0x0,
    .formatConf.nSwBits = 0x20,
    .formatConf.bBitReversal = 0x0,
    .formatConf.bMsbFirst = 0x0,
    .formatConf.fecMode = 0x8,
    .formatConf.whitenMode = 0x0,
    .config.frontEndMode = 0x2,
    .config.biasMode = 0x1,
    .config.analogCfgMode = 0x0,
    .config.bNoFsPowerUp = 0x0,
    .txPower = 0xAB3F,
    .pRegOverride = pOverrides_sl_lr,
    .centerFreq = 0x0364,
    .intFreq = 0x8000,
    .loDivider = 0x05,
};

// CMD_TX_HS
rfc_CMD_HS_TX_t RF_cmdTxHS =
{
    .commandNo = 0x3841,
    .status = 0x0000,
    .pNextOp = 0x00000000,
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .pktConf.bFsOff = 0x0,
    .pktConf.bUseCrc = 0x1,
    .pktConf.bVarLen = 0x1,
    .pQueue = 0,
};

// CMD_RX_HS
rfc_CMD_HS_RX_t RF_cmdRxHS =
{
    .commandNo = CMD_HS_RX,
    .status = 0x0000,
    .pNextOp = 0x00000000,
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .pktConf.bFsOff = 0,
    .pktConf.bUseCrc = 1,
    .pktConf.bVarLen = 1,
    .pktConf.bRepeatOk = 0,
    .pktConf.bRepeatNok = 0,
    .pktConf.addressMode = 0,
    .rxConf.bAutoFlushCrcErr = 0,
    .rxConf.bIncludeLen = 1,
    .rxConf.bIncludeCrc = 0,
    .rxConf.bAppendStatus = 0,
    .rxConf.bAppendTimestamp = 0,
    .maxPktLen = 0,
    .address0 = 0,
    .address1 = 0,
    .__dummy0 = 0,
    .endTrigger.triggerType = 1,
    .endTrigger.bEnaCmd = 0,
    .endTrigger.triggerNo = 0,
    .endTrigger.pastTrig = 0,
    .endTime = 0,
    .pQueue = 0,
    .pOutput = 0,
};

// CMD_FS
rfc_CMD_FS_t RF_cmdFs_preDef =
{
    .commandNo = 0x0803,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .frequency = 0x0364,
    .fractFreq = 0x0000,
    .synthConf.bTxMode = 0x0,
    .synthConf.refFreq = 0x0,
    .__dummy0 = 0x00,
    .__dummy1 = 0x00,
    .__dummy2 = 0x00,
    .__dummy3 = 0x0000,
};

// CMD_PROP_TX
rfc_CMD_PROP_TX_t RF_cmdPropTx_preDef =
{
    .commandNo = 0x3801,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .pktConf.bFsOff = 0x0,
    .pktConf.bUseCrc = 0x1,
    .pktConf.bVarLen = 0x1,
    .pktLen = 0x14, // SET APPLICATION PAYLOAD LENGTH
    .syncWord = 0x930B51DE,
    .pPkt = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
};

// CMD_PROP_RX_ADV
rfc_CMD_PROP_RX_ADV_t RF_cmdPropRxAdv_preDef =
{
    .commandNo = 0x3804,
    .status = 0x0000,
    .pNextOp = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .startTime = 0x00000000,
    .startTrigger.triggerType = 0x0,
    .startTrigger.bEnaCmd = 0x0,
    .startTrigger.triggerNo = 0x0,
    .startTrigger.pastTrig = 0x0,
    .condition.rule = 0x1,
    .condition.nSkip = 0x0,
    .pktConf.bFsOff = 0x0,
    .pktConf.bRepeatOk = 0x0,
    .pktConf.bRepeatNok = 0x0,
    .pktConf.bUseCrc = 0x1,
    .pktConf.bCrcIncSw = 0x0,
    .pktConf.bCrcIncHdr = 0x1,
    .pktConf.endType = 0x0,
    .pktConf.filterOp = 0x1,
    .rxConf.bAutoFlushIgnored = 0x0,
    .rxConf.bAutoFlushCrcErr = 0x0,
    .rxConf.bIncludeHdr = 0x1,
    .rxConf.bIncludeCrc = 0x0,
    .rxConf.bAppendRssi = 0x0,
    .rxConf.bAppendTimestamp = 0x0,
    .rxConf.bAppendStatus = 0x0,
    .syncWord0 = 0x930B51DE,
    .syncWord1 = 0,
    .maxPktLen = 0,
    .hdrConf.numHdrBits = 8,
    .hdrConf.lenPos = 0,
    .hdrConf.numLenBits = 8,
    .addrConf.addrType = 0,
    .addrConf.addrSize = 0,
    .addrConf.addrPos = 0,
    .addrConf.numAddr = 1,
    .lenOffset = 0,
    .endTrigger.triggerType = 0x1,
    .endTrigger.bEnaCmd = 0x0,
    .endTrigger.triggerNo = 0x0,
    .endTrigger.pastTrig = 0x0,
    .endTime = 0x00000000,
    .pAddr = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
    .pQueue = 0, // INSERT APPLICABLE POINTER: (dataQueue_t*)&xxx
    .pOutput = 0, // INSERT APPLICABLE POINTER: (uint8_t*)&xxx
};

/* RF command pointers for easylink */
RF_Mode *RF_pProp_fsk = &RF_prop_fsk;
RF_Mode *RF_pProp_lrm = &RF_prop_lrm;
RF_Mode *RF_pProp_ook = &RF_prop_ook;
RF_Mode *RF_pProp_hsm = &RF_prop_hsm;
RF_Mode *RF_pProp_sl_lr = &RF_prop_sl_lr;
/* Stub out nsupported modes */
RF_Mode *RF_pProp_2_4G_fsk = 0;

rfc_CMD_PROP_RADIO_DIV_SETUP_t *RF_pCmdPropRadioDivSetup_fsk = &RF_cmdPropRadioDivSetup_fsk;
rfc_CMD_PROP_RADIO_DIV_SETUP_t *RF_pCmdPropRadioDivSetup_lrm = &RF_cmdPropRadioDivSetup_lrm;
rfc_CMD_PROP_RADIO_DIV_SETUP_t *RF_pCmdPropRadioDivSetup_ook = &RF_cmdPropRadioDivSetup_ook;
rfc_CMD_RADIO_SETUP_t *RF_pCmdRadioSetup_hsm = &RF_cmdRadioSetup_hsm;
rfc_CMD_PROP_RADIO_DIV_SETUP_t *RF_pCmdPropRadioDivSetup_sl_lr = &RF_cmdPropRadioDivSetup_sl_lr;
/* Stub out nsupported modes */
rfc_CMD_PROP_RADIO_SETUP_t *RF_pCmdPropRadioSetup_2_4G_fsk = 0;

rfc_CMD_HS_TX_t *RF_pCmdTxHS = &RF_cmdTxHS;
rfc_CMD_HS_RX_t *RF_pCmdRxHS = &RF_cmdRxHS;
rfc_CMD_FS_t *RF_pCmdFs_preDef = &RF_cmdFs_preDef;
rfc_CMD_PROP_TX_t *RF_pCmdPropTx_preDef = &RF_cmdPropTx_preDef;
rfc_CMD_PROP_RX_ADV_t *RF_pCmdPropRxAdv_preDef = &RF_cmdPropRxAdv_preDef;

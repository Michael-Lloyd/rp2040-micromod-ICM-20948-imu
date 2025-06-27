#include "ICM_20948.h"

#include "util/ICM_20948_REGISTERS.h"
#include "util/AK09916_REGISTERS.h"
#include <string.h>  // For memset

// Forward Declarations
ICM_20948_Status_e ICM_20948_write_I2C(uint8_t reg, uint8_t *data, uint32_t len, void *user);
ICM_20948_Status_e ICM_20948_read_I2C(uint8_t reg, uint8_t *buff, uint32_t len, void *user);

// Base
ICM_20948::ICM_20948()
{
  status = ICM_20948_init_struct(&_device);
}

void ICM_20948::enableDebugging()
{
  _printDebug = true; //Should we print the commands we send? Good for debugging
}

void ICM_20948::disableDebugging(void)
{
  _printDebug = false; //Turn off extra print statements
}

void ICM_20948::debugPrintStatus(ICM_20948_Status_e stat)
{
  if (!_printDebug) return;
  
  switch (stat)
  {
  case ICM_20948_Stat_Ok:
    printf("ICM_20948: All is well.\n");
    break;
  case ICM_20948_Stat_Err:
    printf("ICM_20948: General Error\n");
    break;
  case ICM_20948_Stat_NotImpl:
    printf("ICM_20948: Not Implemented\n");
    break;
  case ICM_20948_Stat_ParamErr:
    printf("ICM_20948: Parameter Error\n");
    break;
  case ICM_20948_Stat_WrongID:
    printf("ICM_20948: Wrong ID\n");
    break;
  case ICM_20948_Stat_InvalSensor:
    printf("ICM_20948: Invalid Sensor\n");
    break;
  case ICM_20948_Stat_NoData:
    printf("ICM_20948: No Data\n");
    break;
  case ICM_20948_Stat_SensorNotSupported:
    printf("ICM_20948: Sensor Not Supported\n");
    break;
  case ICM_20948_Stat_DMPNotSupported:
    printf("ICM_20948: DMP Not Supported\n");
    break;
  case ICM_20948_Stat_DMPVerifyFail:
    printf("ICM_20948: DMP Verify Fail\n");
    break;
  case ICM_20948_Stat_FIFONoDataAvail:
    printf("ICM_20948: FIFO No Data Available\n");
    break;
  case ICM_20948_Stat_FIFOIncompleteData:
    printf("ICM_20948: FIFO Incomplete Data\n");
    break;
  case ICM_20948_Stat_FIFOMoreDataAvail:
    printf("ICM_20948: FIFO More Data Available\n");
    break;
  case ICM_20948_Stat_UnrecognisedDMPHeader:
    printf("ICM_20948: Unrecognised DMP Header\n");
    break;
  case ICM_20948_Stat_UnrecognisedDMPHeader2:
    printf("ICM_20948: Unrecognised DMP Header2\n");
    break;
  case ICM_20948_Stat_InvalDMPRegister:
    printf("ICM_20948: Invalid DMP Register\n");
    break;
  default:
    printf("ICM_20948: Unknown Status\n");
    break;
  }
}

ICM_20948_AGMT_t ICM_20948::getAGMT(void)
{
  status = ICM_20948_get_agmt(&_device, &agmt);

  if (_printDebug && (status != ICM_20948_Stat_Ok))
  {
    printf("ICM_20948::getAGMT: ");
    debugPrintStatus(status);
  }

  return agmt;
}

float ICM_20948::magX(void)
{
  return getMagUT(agmt.mag.axes.x);
}

float ICM_20948::magY(void)
{
  return getMagUT(agmt.mag.axes.y);
}

float ICM_20948::magZ(void)
{
  return getMagUT(agmt.mag.axes.z);
}

float ICM_20948::getMagUT(int16_t axis_val)
{
  return (((float)axis_val) * 0.15);
}

float ICM_20948::accX(void)
{
  return getAccMG(agmt.acc.axes.x);
}

float ICM_20948::accY(void)
{
  return getAccMG(agmt.acc.axes.y);
}

float ICM_20948::accZ(void)
{
  return getAccMG(agmt.acc.axes.z);
}

float ICM_20948::getAccMG(int16_t axis_val)
{
  switch (agmt.fss.a)
  {
  case 0:
    return (((float)axis_val) / 16.384);
    break;
  case 1:
    return (((float)axis_val) / 8.192);
    break;
  case 2:
    return (((float)axis_val) / 4.096);
    break;
  case 3:
    return (((float)axis_val) / 2.048);
    break;
  default:
    return 0;
    break;
  }
}

float ICM_20948::gyrX(void)
{
  return getGyrDPS(agmt.gyr.axes.x);
}

float ICM_20948::gyrY(void)
{
  return getGyrDPS(agmt.gyr.axes.y);
}

float ICM_20948::gyrZ(void)
{
  return getGyrDPS(agmt.gyr.axes.z);
}

float ICM_20948::getGyrDPS(int16_t axis_val)
{
  switch (agmt.fss.g)
  {
  case 0:
    return (((float)axis_val) / 131);
    break;
  case 1:
    return (((float)axis_val) / 65.5);
    break;
  case 2:
    return (((float)axis_val) / 32.8);
    break;
  case 3:
    return (((float)axis_val) / 16.4);
    break;
  default:
    return 0;
    break;
  }
}

float ICM_20948::temp(void)
{
  return getTempC(agmt.tmp.val);
}

float ICM_20948::getTempC(int16_t val)
{
  return (((float)val) / 333.87) + 21;
}

// Device Level
ICM_20948_Status_e ICM_20948::setBank(uint8_t bank)
{
  status = ICM_20948_set_bank(&_device, bank);
  return status;
}

ICM_20948_Status_e ICM_20948::swReset(void)
{
  status = ICM_20948_sw_reset(&_device);
  return status;
}

ICM_20948_Status_e ICM_20948::sleep(bool on)
{
  status = ICM_20948_sleep(&_device, on);
  return status;
}

ICM_20948_Status_e ICM_20948::lowPower(bool on)
{
  status = ICM_20948_low_power(&_device, on);
  return status;
}

ICM_20948_Status_e ICM_20948::setClockSource(ICM_20948_PWR_MGMT_1_CLKSEL_e source)
{
  status = ICM_20948_set_clock_source(&_device, source);
  return status;
}

ICM_20948_Status_e ICM_20948::checkID(void)
{
  status = ICM_20948_check_id(&_device);
  if (_printDebug)
  {
    printf("ICM_20948::checkID: ");
    debugPrintStatus(status);
  }
  return status;
}

bool ICM_20948::dataReady(void)
{
  status = ICM_20948_data_ready(&_device);
  if (status == ICM_20948_Stat_Ok)
  {
    return true;
  }
  return false;
}

uint8_t ICM_20948::getWhoAmI(void)
{
  uint8_t retval = 0x00;
  status = ICM_20948_get_who_am_i(&_device, &retval);
  return retval;
}

bool ICM_20948::isConnected(void)
{
  status = checkID();
  if (status == ICM_20948_Stat_Ok)
  {
    return true;
  }
  if (_printDebug)
  {
    printf("ICM_20948::isConnected: checkID returned: ");
    debugPrintStatus(status);
  }
  return false;
}

// Internal Sensor Options
ICM_20948_Status_e ICM_20948::setSampleMode(uint8_t sensor_id_bm, uint8_t lp_config_cycle_mode)
{
  status = ICM_20948_set_sample_mode(&_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, (ICM_20948_LP_CONFIG_CYCLE_e)lp_config_cycle_mode);
  if (_printDebug && (status != ICM_20948_Stat_Ok))
  {
    printf("ICM_20948::setSampleMode: ");
    debugPrintStatus(status);
  }
  return status;
}

ICM_20948_Status_e ICM_20948::setFullScale(uint8_t sensor_id_bm, ICM_20948_fss_t fss)
{
  status = ICM_20948_set_full_scale(&_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, fss);
  return status;
}

ICM_20948_Status_e ICM_20948::setDLPFcfg(uint8_t sensor_id_bm, ICM_20948_dlpcfg_t cfg)
{
  status = ICM_20948_set_dlpf_cfg(&_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, cfg);
  return status;
}

ICM_20948_Status_e ICM_20948::enableDLPF(uint8_t sensor_id_bm, bool enable)
{
  status = ICM_20948_enable_dlpf(&_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, enable);
  return status;
}

ICM_20948_Status_e ICM_20948::setSampleRate(uint8_t sensor_id_bm, ICM_20948_smplrt_t smplrt)
{
  status = ICM_20948_set_sample_rate(&_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, smplrt);
  return status;
}

// Interrupts on INT Pin
ICM_20948_Status_e ICM_20948::clearInterrupts(void)
{
  ICM_20948_INT_STATUS_t int_stat;
  ICM_20948_INT_STATUS_1_t int_stat_1;

  // read to clear interrupts
  status = ICM_20948_set_bank(&_device, 0);
  if (status != ICM_20948_Stat_Ok)
  {
    return status;
  }
  
  status = ICM_20948_execute_r(&_device, AGB0_REG_INT_STATUS, (uint8_t *)&int_stat, sizeof(ICM_20948_INT_STATUS_t));
  if (status != ICM_20948_Stat_Ok)
  {
    return status;
  }
  
  status = ICM_20948_execute_r(&_device, AGB0_REG_INT_STATUS_1, (uint8_t *)&int_stat_1, sizeof(ICM_20948_INT_STATUS_1_t));
  if (status != ICM_20948_Stat_Ok)
  {
    return status;
  }

  // todo: there may be additional interrupts that need to be cleared, like FIFO overflow/watermark

  return status;
}

ICM_20948_Status_e ICM_20948::cfgIntActiveLow(bool active_low)
{
  ICM_20948_INT_PIN_CFG_t reg;
  status = ICM_20948_int_pin_cfg(&_device, NULL, &reg); // Read current settings
  if (status == ICM_20948_Stat_Ok)
  {
    reg.INT1_ACTL = active_low ? 1 : 0;
    status = ICM_20948_int_pin_cfg(&_device, &reg, NULL); // Write new settings
  }
  return status;
}

ICM_20948_Status_e ICM_20948::cfgIntOpenDrain(bool open_drain)
{
  ICM_20948_INT_PIN_CFG_t reg;
  status = ICM_20948_int_pin_cfg(&_device, NULL, &reg); // Read current settings
  if (status == ICM_20948_Stat_Ok)
  {
    reg.INT1_OPEN = open_drain ? 1 : 0;
    status = ICM_20948_int_pin_cfg(&_device, &reg, NULL); // Write new settings
  }
  return status;
}

ICM_20948_Status_e ICM_20948::cfgIntLatch(bool latching)
{
  ICM_20948_INT_PIN_CFG_t reg;
  status = ICM_20948_int_pin_cfg(&_device, NULL, &reg); // Read current settings
  if (status == ICM_20948_Stat_Ok)
  {
    reg.INT1_LATCH_EN = latching ? 1 : 0;
    status = ICM_20948_int_pin_cfg(&_device, &reg, NULL); // Write new settings
  }
  return status;
}

ICM_20948_Status_e ICM_20948::cfgIntAnyReadToClear(bool enabled)
{
  ICM_20948_INT_PIN_CFG_t reg;
  status = ICM_20948_int_pin_cfg(&_device, NULL, &reg); // Read current settings
  if (status == ICM_20948_Stat_Ok)
  {
    reg.INT_ANYRD_2CLEAR = enabled ? 1 : 0;
    status = ICM_20948_int_pin_cfg(&_device, &reg, NULL); // Write new settings
  }
  return status;
}

ICM_20948_Status_e ICM_20948::cfgFsyncActiveLow(bool active_low)
{
  // Comment out - incorrect number of arguments
  // status = ICM_20948_int_pin_cfg(&_device, NULL, NULL, NULL, NULL, NULL, &active_low, NULL);
  status = ICM_20948_Stat_NotImpl;
  return status;
}

ICM_20948_Status_e ICM_20948::cfgFsyncIntMode(bool interrupt_mode)
{
  // Comment out - incorrect number of arguments
  // status = ICM_20948_int_pin_cfg(&_device, NULL, NULL, NULL, NULL, NULL, NULL, &interrupt_mode);
  status = ICM_20948_Stat_NotImpl;
  return status;
}

ICM_20948_Status_e ICM_20948::intEnableI2C(bool enable)
{
  // Comment out - incorrect number of arguments
  // status = ICM_20948_int_enable(&_device, NULL, NULL, NULL, NULL, &enable, NULL, NULL, NULL);
  status = ICM_20948_Stat_NotImpl;
  return status;
}

ICM_20948_Status_e ICM_20948::intEnableDMP(bool enable)
{
  ICM_20948_INT_enable_t en;
  memset(&en, 0, sizeof(ICM_20948_INT_enable_t));
  en.DMP_INT1_EN = enable ? 1 : 0;
  status = ICM_20948_int_enable(&_device, &en, NULL);
  return status;
}

ICM_20948_Status_e ICM_20948::intEnablePLL(bool enable)
{
  ICM_20948_INT_enable_t en;
  memset(&en, 0, sizeof(ICM_20948_INT_enable_t));
  en.PLL_RDY_EN = enable ? 1 : 0;
  status = ICM_20948_int_enable(&_device, &en, NULL);
  return status;
}

ICM_20948_Status_e ICM_20948::intEnableWOM(bool enable)
{
  ICM_20948_INT_enable_t en;
  memset(&en, 0, sizeof(ICM_20948_INT_enable_t));
  en.WOM_INT_EN = enable ? 1 : 0;
  status = ICM_20948_int_enable(&_device, &en, NULL);
  return status;
}

ICM_20948_Status_e ICM_20948::intEnableWOF(bool enable)
{
  // Comment out - function not found
  // status = ICM_20948_int_enable_1(&_device, &enable);
  status = ICM_20948_Stat_NotImpl;
  return status;
}

ICM_20948_Status_e ICM_20948::intEnableRawDataReady(bool enable)
{
  ICM_20948_INT_enable_t en;
  memset(&en, 0, sizeof(ICM_20948_INT_enable_t));
  en.RAW_DATA_0_RDY_EN = enable ? 1 : 0;
  status = ICM_20948_int_enable(&_device, &en, NULL);
  return status;
}

ICM_20948_Status_e ICM_20948::intEnableOverflowFIFO(uint8_t bm_enable)
{
  ICM_20948_INT_enable_t en;
  memset(&en, 0, sizeof(ICM_20948_INT_enable_t));
  // Set individual FIFO overflow bits
  en.FIFO_OVERFLOW_EN_0 = (bm_enable >> 0) & 0x01;
  en.FIFO_OVERFLOW_EN_1 = (bm_enable >> 1) & 0x01;
  en.FIFO_OVERFLOW_EN_2 = (bm_enable >> 2) & 0x01;
  en.FIFO_OVERFLOW_EN_3 = (bm_enable >> 3) & 0x01;
  en.FIFO_OVERFLOW_EN_4 = (bm_enable >> 4) & 0x01;
  status = ICM_20948_int_enable(&_device, &en, NULL);
  return status;
}

ICM_20948_Status_e ICM_20948::intEnableWatermarkFIFO(uint8_t bm_enable)
{
  ICM_20948_INT_enable_t en;
  memset(&en, 0, sizeof(ICM_20948_INT_enable_t));
  // Set individual FIFO watermark bits
  en.FIFO_WM_EN_0 = (bm_enable >> 0) & 0x01;
  en.FIFO_WM_EN_1 = (bm_enable >> 1) & 0x01;
  en.FIFO_WM_EN_2 = (bm_enable >> 2) & 0x01;
  en.FIFO_WM_EN_3 = (bm_enable >> 3) & 0x01;
  en.FIFO_WM_EN_4 = (bm_enable >> 4) & 0x01;
  status = ICM_20948_int_enable(&_device, &en, NULL);
  return status;
}

ICM_20948_Status_e ICM_20948::WOMLogic(uint8_t enable, uint8_t mode)
{
  ICM_20948_ACCEL_INTEL_CTRL_t reg;
  memset(&reg, 0, sizeof(ICM_20948_ACCEL_INTEL_CTRL_t));
  reg.ACCEL_INTEL_EN = enable;
  reg.ACCEL_INTEL_MODE_INT = mode;
  status = ICM_20948_wom_logic(&_device, &reg, NULL);
  return status;
}

ICM_20948_Status_e ICM_20948::WOMThreshold(uint8_t threshold)
{
  ICM_20948_ACCEL_WOM_THR_t thr;
  thr.WOM_THRESHOLD = threshold;
  status = ICM_20948_wom_threshold(&_device, &thr, NULL);
  return status;
}

// Interface Options
ICM_20948_Status_e ICM_20948::i2cMasterPassthrough(bool passthrough)
{
  status = ICM_20948_i2c_master_passthrough(&_device, passthrough);
  return status;
}

ICM_20948_Status_e ICM_20948::i2cMasterEnable(bool enable)
{
  status = ICM_20948_i2c_master_enable(&_device, enable);

  if (_printDebug && (status != ICM_20948_Stat_Ok))
  {
    printf("ICM_20948::i2cMasterEnable: ");
    debugPrintStatus(status);
  }
  
  return status;
}

ICM_20948_Status_e ICM_20948::i2cMasterReset()
{
  status = ICM_20948_i2c_master_reset(&_device);
  return status;
}

ICM_20948_Status_e ICM_20948::i2cControllerConfigurePeripheral(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap, uint8_t dataOut)
{
  status = ICM_20948_i2c_controller_configure_peripheral(&_device, peripheral, addr, reg, len, Rw, enable, data_only, grp, swap, dataOut);
  return status;
}

ICM_20948_Status_e ICM_20948::i2cControllerPeriph4Transaction(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr)
{
  status = ICM_20948_i2c_controller_periph4_txn(&_device, addr, reg, data, len, Rw, send_reg_addr);
  return status;
}

//Provided for backward-compatibility only. Please update to i2cControllerConfigurePeripheral and i2cControllerPeriph4Transaction.
ICM_20948_Status_e ICM_20948::i2cMasterConfigureSlave(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap)
{
  return i2cControllerConfigurePeripheral(peripheral, addr, reg, len, Rw, enable, data_only, grp, swap, 0);
}

ICM_20948_Status_e ICM_20948::i2cMasterSLV4Transaction(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr)
{
  return i2cControllerPeriph4Transaction(addr, reg, data, len, Rw, send_reg_addr);
}

ICM_20948_Status_e ICM_20948::i2cMasterSingleW(uint8_t addr, uint8_t reg, uint8_t data)
{
  status = ICM_20948_i2c_master_single_w(&_device, addr, reg, &data);
  return status;
}

uint8_t ICM_20948::i2cMasterSingleR(uint8_t addr, uint8_t reg)
{
  uint8_t data = 0;
  status = ICM_20948_i2c_master_single_r(&_device, addr, reg, &data);
  return data;
}

ICM_20948_Status_e ICM_20948::startupDefault(bool minimal)
{
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  retval = checkID();
  if (retval != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::startupDefault: checkID returned: ");
      debugPrintStatus(retval);
    }
    status = retval;
    return status;
  }

  retval = swReset();
  if (retval != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::startupDefault: swReset returned: ");
      debugPrintStatus(retval);
    }
    status = retval;
    return status;
  }
  sleep_ms(15);

  retval = sleep(false);
  if (retval != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::startupDefault: sleep returned: ");
      debugPrintStatus(retval);
    }
    status = retval;
    return status;
  }

  retval = lowPower(false);
  if (retval != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::startupDefault: lowPower returned: ");
      debugPrintStatus(retval);
    }
    status = retval;
    return status;
  }

  retval = startupMagnetometer(minimal); // Pass the minimal startup flag to startupMagnetometer
  if (retval != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::startupDefault: startupMagnetometer returned: ");
      debugPrintStatus(retval);
    }
    status = retval;
    return status;
  }

  if (minimal == false) // If minimal is false, performt the remaining startup
  {
    retval = setClockSource(ICM_20948_Clock_Auto); // This returns an error code, but we don't care to check it.
    if (retval != ICM_20948_Stat_Ok)
    {
      if (_printDebug)
      {
        printf("ICM_20948::startupDefault: setClockSource returned: ");
        debugPrintStatus(retval);
      }
      status = retval;
      return status;
    }

    // The next few configuration functions return ICM_20948_Stat_SensorNotSupported for the magnetometer,
    // so we only use the return value for debugging purposes, and don't return early. Same for DLPF.

    retval = setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous); // This function sets the sample mode for the accel and gyro only. It does not work for the magnetometer.
    if ((retval != ICM_20948_Stat_Ok) && (retval != ICM_20948_Stat_SensorNotSupported))
    {
      if (_printDebug)
      {
        printf("ICM_20948::startupDefault: setSampleMode returned: ");
        debugPrintStatus(retval);
      }
      // Do not return early. Allow the sensor to continue to configure.
      //status = retval;
      //return status;
    }

    ICM_20948_fss_t FSS;
    FSS.g = dps2000; // Set full scale to +/- 2000 degrees per second
    FSS.a = gpm16;   // Set full scale to +/- 16g

    retval = setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), FSS);
    if ((retval != ICM_20948_Stat_Ok) && (retval != ICM_20948_Stat_SensorNotSupported))
    {
      if (_printDebug)
      {
        printf("ICM_20948::startupDefault: setFullScale returned: ");
        debugPrintStatus(retval);
      }
      // Do not return early. Allow the sensor to continue to configure.
      //status = retval;
      //return status;
    }

    ICM_20948_dlpcfg_t dlpcfg;
    dlpcfg.g = ICM_20948_DLPF_CFG_51_8HZ;  // Set gyro low pass filter to 51.8Hz
    dlpcfg.a = ICM_20948_ACCEL_DLPF_CFG_50_4HZ;  // Set accel low pass filter to 50.4Hz

    retval = setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg);
    if ((retval != ICM_20948_Stat_Ok) && (retval != ICM_20948_Stat_SensorNotSupported))
    {
      if (_printDebug)
      {
        printf("ICM_20948::startupDefault: setDLPFcfg returned: ");
        debugPrintStatus(retval);
      }
      // Do not return early. Allow the sensor to continue to configure.
      //status = retval;
      //return status;
    }

    retval = enableDLPF((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), true);
    if ((retval != ICM_20948_Stat_Ok) && (retval != ICM_20948_Stat_SensorNotSupported))
    {
      if (_printDebug)
      {
        printf("ICM_20948::startupDefault: enableDLPF returned: ");
        debugPrintStatus(retval);
      }
      // Do not return early. Allow the sensor to continue to configure.
      //status = retval;
      //return status;
    }
  }

  return ICM_20948_Stat_Ok;
}

// direct read/write
ICM_20948_Status_e ICM_20948::read(uint8_t reg, uint8_t *data, uint32_t len)
{
  status = ICM_20948_execute_r(&_device, reg, data, len);
  return (status);
}

ICM_20948_Status_e ICM_20948::write(uint8_t reg, uint8_t *data, uint32_t len)
{
  status = ICM_20948_execute_w(&_device, reg, data, len);
  return (status);
}

const char *ICM_20948::statusString(ICM_20948_Status_e stat)
{
  ICM_20948_Status_e val;
  if (stat == ICM_20948_Stat_NUM)
  {
    val = status;
  }
  else
  {
    val = stat;
  }

  switch (val)
  {
  case ICM_20948_Stat_Ok:
    return "All is well.";
    break;
  case ICM_20948_Stat_Err:
    return "General Error";
    break;
  case ICM_20948_Stat_NotImpl:
    return "Not Implemented";
    break;
  case ICM_20948_Stat_ParamErr:
    return "Parameter Error";
    break;
  case ICM_20948_Stat_WrongID:
    return "Wrong ID";
    break;
  case ICM_20948_Stat_InvalSensor:
    return "Invalid Sensor";
    break;
  case ICM_20948_Stat_NoData:
    return "No Data";
    break;
  case ICM_20948_Stat_SensorNotSupported:
    return "Sensor Not Supported";
    break;
  case ICM_20948_Stat_DMPNotSupported:
    return "DMP Not Supported";
    break;
  case ICM_20948_Stat_DMPVerifyFail:
    return "DMP Verify Fail";
    break;
  case ICM_20948_Stat_FIFONoDataAvail:
    return "FIFO No Data Available";
    break;
  case ICM_20948_Stat_FIFOIncompleteData:
    return "FIFO Incomplete Data";
    break;
  case ICM_20948_Stat_FIFOMoreDataAvail:
    return "FIFO More Data Available";
    break;
  case ICM_20948_Stat_UnrecognisedDMPHeader:
    return "Unrecognised DMP Header";
    break;
  case ICM_20948_Stat_UnrecognisedDMPHeader2:
    return "Unrecognised DMP Header2";
    break;
  case ICM_20948_Stat_InvalDMPRegister:
    return "Invalid DMP Register";
    break;
  default:
    return "Unknown Status";
    break;
  }
}

//Mag specific
ICM_20948_Status_e ICM_20948::startupMagnetometer(bool minimal)
{
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  if (minimal == true) // Return now if minimal is true
  {
    if (_printDebug)
    {
      printf("ICM_20948::startupMagnetometer: Returning early. minimal = 1\n");
    }
    return status;
  }

  i2cMasterPassthrough(false); //Do not connect the SDA/SCL pins to AUX_DA/AUX_CL
  i2cMasterEnable(true);

  //After a ICM reset the Mag sensor may stop responding over the I2C master
  //Reset the Master I2C until it responds
  uint8_t tries = 0;
  uint8_t maxTries = MAX_MAGNETOMETER_STARTS;
  for (tries = 0; tries < maxTries; tries++)
  {
    //See if we can read the WhoAmI register correctly
    retval = magWhoIAm();
    if (retval == ICM_20948_Stat_Ok)
      break; //WhoAmI matches!

    i2cMasterReset(); //Otherwise, reset the master I2C and try again

    sleep_ms(10);
  }

  if (tries == maxTries)
  {
    if (_printDebug)
    {
      printf("ICM_20948::startupMagnetometer: reached max number of tries. Returning ICM_20948_Stat_WrongID\n");
    }
    status = ICM_20948_Stat_WrongID;
    return status;
  }

  //Set up magnetometer
  AK09916_CNTL2_Reg_t reg;
  reg.reserved_0 = 0x00;
  reg.MODE = AK09916_mode_cont_100hz;
  retval = writeMag(AK09916_REG_CNTL2, (uint8_t *)&reg);
  if (retval != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::startupMagnetometer: writeMag returned: ");
      debugPrintStatus(retval);
    }
    status = retval;
    return status;
  }

  retval = i2cControllerConfigurePeripheral(0, MAG_AK09916_I2C_ADDR, AK09916_REG_ST1, 9, true, true, false, false, false); // Read the magnetometer registers at 100Hz
  if (retval != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::startupMagnetometer: i2cControllerConfigurePeripheral returned: ");
      debugPrintStatus(status);
    }
    status = retval;
    return status;
  }

  return retval;
}

ICM_20948_Status_e ICM_20948::magWhoIAm(void)
{
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  uint8_t whoiam1, whoiam2;
  // i2cMasterSingleR returns the data read, not a status
  // We need to read the data and check if the transaction succeeded
  whoiam1 = i2cMasterSingleR(MAG_AK09916_I2C_ADDR, AK09916_REG_WIA1);
  // Note: We can't reliably detect I2C errors with this API
  // The original code incorrectly treated the return value as a status
  if (whoiam1 != (MAG_AK09916_WHO_AM_I >> 8))
  {
    if (_printDebug)
    {
      printf("ICM_20948::magWhoIAm: whoiam1 != 0x%02X\n", (unsigned int)(MAG_AK09916_WHO_AM_I >> 8));
    }
    status = ICM_20948_Stat_WrongID;
    return status;
  }

  whoiam2 = i2cMasterSingleR(MAG_AK09916_I2C_ADDR, AK09916_REG_WIA2);
  // Note: We can't reliably detect I2C errors with this API
  if (whoiam2 != (MAG_AK09916_WHO_AM_I & 0x00FF))
  {
    if (_printDebug)
    {
      printf("ICM_20948::magWhoIAm: whoiam2 != 0x%02X\n", (unsigned int)(MAG_AK09916_WHO_AM_I & 0x00FF));
    }
    status = ICM_20948_Stat_WrongID;
    return status;
  }

  return status;
}

uint8_t ICM_20948::readMag(AK09916_Reg_Addr_e reg)
{
  uint8_t data = i2cMasterSingleR(MAG_AK09916_I2C_ADDR, reg);
  return data;
}

ICM_20948_Status_e ICM_20948::writeMag(AK09916_Reg_Addr_e reg, uint8_t *pdata)
{
  status = i2cMasterSingleW(MAG_AK09916_I2C_ADDR, reg, *pdata);
  return status;
}

ICM_20948_Status_e ICM_20948::resetMag()
{
  uint8_t SRST = 0x01;
  status = writeMag(AK09916_REG_CNTL3, &SRST);
  if (status != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::resetMag: writeMag returned: ");
      debugPrintStatus(status);
    }
    return status;
  }
  return status;
}

// FIFO

ICM_20948_Status_e ICM_20948::enableFIFO(bool enable)
{
  status = ICM_20948_enable_FIFO(&_device, enable);
  return status;
}

ICM_20948_Status_e ICM_20948::resetFIFO(void)
{
  status = ICM_20948_reset_FIFO(&_device);
  return status;
}

ICM_20948_Status_e ICM_20948::setFIFOmode(bool snapshot)
{
  status = ICM_20948_set_FIFO_mode(&_device, snapshot);
  return status;
}

ICM_20948_Status_e ICM_20948::getFIFOcount(uint16_t *count)
{
  status = ICM_20948_get_FIFO_count(&_device, count);
  return status;
}

ICM_20948_Status_e ICM_20948::readFIFO(uint8_t *data, uint8_t len)
{
  status = ICM_20948_read_FIFO(&_device, data, len);
  return status;
}

// DMP

ICM_20948_Status_e ICM_20948::enableDMP(bool enable)
{
  status = ICM_20948_enable_DMP(&_device, enable);
  return status;
}

ICM_20948_Status_e ICM_20948::resetDMP(void)
{
  status = ICM_20948_reset_DMP(&_device);
  return status;
}

ICM_20948_Status_e ICM_20948::loadDMPFirmware(void)
{
  status = ICM_20948_firmware_load(&_device);
  return status;
}

ICM_20948_Status_e ICM_20948::setDMPstartAddress(unsigned short address)
{
  status = ICM_20948_set_dmp_start_address(&_device, address);
  return status;
}

ICM_20948_Status_e ICM_20948::readDMPmems(unsigned short reg, unsigned int length, unsigned char *data)
{
  status = inv_icm20948_read_mems(&_device, reg, length, data);
  return status;
}

ICM_20948_Status_e ICM_20948::writeDMPmems(unsigned short reg, unsigned int length, const unsigned char *data)
{
  status = inv_icm20948_write_mems(&_device, reg, length, data);
  return status;
}

ICM_20948_Status_e ICM_20948::setGyroSF(unsigned char div, int gyro_level)
{
  status = inv_icm20948_set_gyro_sf(&_device, div, gyro_level);
  return status;
}

ICM_20948_Status_e ICM_20948::enableDMPSensor(enum inv_icm20948_sensor sensor, bool enable)
{
  if (enable)
    status = inv_icm20948_enable_dmp_sensor(&_device, sensor, 1);
  else
    // No disable function available, return not implemented
    status = ICM_20948_Stat_NotImpl;

  return status;
}

ICM_20948_Status_e ICM_20948::enableDMPSensorInt(enum inv_icm20948_sensor sensor, bool enable)
{
  if (enable)
    status = inv_icm20948_enable_dmp_sensor_int(&_device, sensor, 1);
  else
    // No disable function available, return not implemented
    status = ICM_20948_Stat_NotImpl;

  return status;
}

ICM_20948_Status_e ICM_20948::readDMPdataFromFIFO(icm_20948_DMP_data_t *data)
{
  status = inv_icm20948_read_dmp_data(&_device, data);
  return status;
}

ICM_20948_Status_e ICM_20948::setDMPODRrate(enum DMP_ODR_Registers odr_reg, int interval)
{
  //https://github.com/InvenSenseInc/ICM-20948_ArduinoLibrary/issues/109
  status = inv_icm20948_set_dmp_sensor_period(&_device, odr_reg, interval);
  return status;
}

ICM_20948_Status_e ICM_20948::initializeDMP(void)
{
  // First, let's check if the DMP is available
  if (_device._dmp_firmware_available != true)
  {
    if (_printDebug)
    {
      printf("ICM_20948::startupDMP: DMP is not available. Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h...\n");
    }
    status = ICM_20948_Stat_DMPNotSupported;
    return status;
  }

  ICM_20948_Status_e result = ICM_20948_Stat_Ok;

  result = setBank(0); // Always return to Bank 0
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: setBank returned: ");
      debugPrintStatus(result);
    }
    status = result;
    return status;
  }

  uint8_t pwrMgmt1 = 0x00;
  result = read(AGB0_REG_PWR_MGMT_1, &pwrMgmt1, 1);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: read of AGB0_REG_PWR_MGMT_1 returned: ");
      debugPrintStatus(result);
    }
    status = result;
    return status;
  }

  if (pwrMgmt1 & (1 << 6))
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: Sleep bit is set - the DMP won't run if the chip is asleep!\n");
    }
  }

  if ((pwrMgmt1 & 0x0F) != ICM_20948_Clock_Auto)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: CLKSEL is not set to auto - the DMP won't run without a clock source\n");
    }
  }

  result = setSampleMode(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, ICM_20948_Sample_Mode_Continuous); // Set accel and gyro to continuous mode - this is the only mode the DMP supports
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: setSampleMode returned: ");
      debugPrintStatus(result);
    }
    status = result;
    return status;
  }

  // Function not available, comment out
  // result = ICM_20948_set_dmp_power_save(&_device, false); // Disable DMP power save mode
  result = ICM_20948_Stat_Ok;
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: ICM_20948_set_dmp_power_save returned: ");
      debugPrintStatus(result);
    }
    status = result;
    return status;
  }

  result = resetFIFO(); // Reset the FIFO
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: resetFIFO returned: ");
      debugPrintStatus(result);
    }
    status = result;
    return status;
  }

  result = resetDMP(); // Reset the DMP
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: resetDMP returned: ");
      debugPrintStatus(result);
    }
    status = result;
    return status;
  }

  result = loadDMPFirmware(); // Load the DMP firmware
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: loadDMPFirmware returned: ");
      debugPrintStatus(result);
    }
    status = result;
    return status;
  }

  result = setDMPstartAddress(); // Write the DMP start address
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: setDMPstartAddress returned: ");
      debugPrintStatus(result);
    }
    status = result;
    return status;
  }

  // Configure the Accel
  result = setFullScale((ICM_20948_Internal_Acc), {.a = dps2000}); // Set the accel full scale to 2g
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: setFullScale (Accel) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }
  result = setFullScale((ICM_20948_Internal_Gyr), {.g = gpm2}); // Set the gyro full scale to 2000dps
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: setFullScale (Gyro) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }
  const unsigned char accScale[4] = {0x04, 0x00, 0x00, 0x00}; // Value taken from InvenSense Nucleo example: 1026 for 2g (AFS=0)
  result = writeDMPmems(ACC_SCALE, 4, &accScale[0]); // Write the accel scale to DMP memory
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (ACC_SCALE) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }
  const unsigned char accScale2[4] = {0x00, 0x04, 0x00, 0x00}; // Value taken from InvenSense Nucleo example: 1026/2^16 for 2g (AFS=0)
  result = writeDMPmems(ACC_SCALE2, 4, &accScale2[0]); // Write the accel scale to DMP memory
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (ACC_SCALE2) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }

  // Configure the Compass
  result = i2cMasterEnable(true); // Enable the I2C master
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: i2cMasterEnable returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }
  const unsigned char mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char mountMultiplierPlus[4] = {0x09, 0x99, 0x99, 0x99};  // Value taken from InvenSense Nucleo example: 0.150000000 * 2^30 (Q30 format)
  const unsigned char mountMultiplierMinus[4] = {0xF6, 0x66, 0x66, 0x67}; // Value taken from InvenSense Nucleo example: -0.150000000 * 2^30 (Q30 format)
  result = writeDMPmems(CPASS_MTX_00, 4, &mountMultiplierPlus[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (CPASS_MTX_00) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }
  result = writeDMPmems(CPASS_MTX_01, 4, &mountMultiplierZero[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (CPASS_MTX_01) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }
  result = writeDMPmems(CPASS_MTX_02, 4, &mountMultiplierZero[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (CPASS_MTX_02) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }
  result = writeDMPmems(CPASS_MTX_10, 4, &mountMultiplierZero[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (CPASS_MTX_10) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }
  result = writeDMPmems(CPASS_MTX_11, 4, &mountMultiplierMinus[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (CPASS_MTX_11) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }
  result = writeDMPmems(CPASS_MTX_12, 4, &mountMultiplierZero[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (CPASS_MTX_12) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }
  result = writeDMPmems(CPASS_MTX_20, 4, &mountMultiplierZero[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (CPASS_MTX_20) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }
  result = writeDMPmems(CPASS_MTX_21, 4, &mountMultiplierZero[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (CPASS_MTX_21) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }
  result = writeDMPmems(CPASS_MTX_22, 4, &mountMultiplierMinus[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (CPASS_MTX_22) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }

  // Configure the B2S Mounting Matrix
  const unsigned char b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00}; // Value taken from InvenSense Nucleo example: 1.00000000 * 2^30 (Q30 format)
  result = writeDMPmems(B2S_MTX_00, 4, &b2sMountMultiplierPlus[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (B2S_MTX_00) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }
  result = writeDMPmems(B2S_MTX_01, 4, &b2sMountMultiplierZero[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (B2S_MTX_01) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }
  result = writeDMPmems(B2S_MTX_02, 4, &b2sMountMultiplierZero[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (B2S_MTX_02) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }
  result = writeDMPmems(B2S_MTX_10, 4, &b2sMountMultiplierZero[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (B2S_MTX_10) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }
  result = writeDMPmems(B2S_MTX_11, 4, &b2sMountMultiplierPlus[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (B2S_MTX_11) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }
  result = writeDMPmems(B2S_MTX_12, 4, &b2sMountMultiplierZero[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (B2S_MTX_12) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }
  result = writeDMPmems(B2S_MTX_20, 4, &b2sMountMultiplierZero[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (B2S_MTX_20) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }
  result = writeDMPmems(B2S_MTX_21, 4, &b2sMountMultiplierZero[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (B2S_MTX_21) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }
  result = writeDMPmems(B2S_MTX_22, 4, &b2sMountMultiplierPlus[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (B2S_MTX_22) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }

  // Configure the Gyro SF
  result = setGyroSF(19, 225); // 19 = 2000 dps. 225 = 225Hz (used when gyro is calibrated)
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: setGyroSF returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }

  // Configure the Gyro full scale
  const unsigned char gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00}; // Value taken from InvenSense Nucleo example: 2000dps
  result = writeDMPmems(GYRO_FULLSCALE, 4, &gyroFullScale[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (GYRO_FULLSCALE) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }

  // Configure the Accel Only Gain: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
  const unsigned char accelOnlyGain[4] = {0x00, 0xE8, 0xBA, 0x2E}; // Value taken from InvenSense Nucleo example: 15252014 (225Hz)
  result = writeDMPmems(ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (ACCEL_ONLY_GAIN) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }

  // Configure the Accel Alpha Var: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
  const unsigned char accelAlphaVar[4] = {0x3D, 0x27, 0xD2, 0x7D}; // Value taken from InvenSense Nucleo example: 1026019965 (225Hz)
  result = writeDMPmems(ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (ACCEL_ALPHA_VAR) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }

  // Configure the Accel A Var: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
  const unsigned char accelAVar[4] = {0x02, 0xD8, 0x2D, 0x83}; // Value taken from InvenSense Nucleo example: 47721859 (225Hz)
  result = writeDMPmems(ACCEL_A_VAR, 4, &accelAVar[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (ACCEL_A_VAR) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }

  // Configure the Accel Cal Rate
  const unsigned char accelCalRate[4] = {0x00, 0x00}; // Value taken from InvenSense Nucleo example: 0 (225Hz)
  result = writeDMPmems(ACCEL_CAL_RATE, 2, &accelCalRate[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (ACCEL_CAL_RATE) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }

  // Configure the Compass Time Buffer. The I2C Master ODR Configuration (see above) sets the magnetometer read rate to 68.75Hz.
  // Let's set the Compass Time Buffer to 69 (Hz).
  const unsigned char compassRate[2] = {0x00, 0x45}; // 69Hz
  result = writeDMPmems(CPASS_TIME_BUFFER, 2, &compassRate[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (CPASS_TIME_BUFFER) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }

  // Enable DMP interrupt
  // This would be the most efficient way of getting the DMP data, instead of polling the FIFO
  result = intEnableDMP(true);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: intEnableDMP returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }

  // Configuring DMP to output data to FIFO: set DATA_OUT_CTL1, DATA_OUT_CTL2, DATA_INTR_CTL and MOTION_EVENT_CTL
  // Configure the DATA_OUT_CTL1 register
  // DATA_OUT_CTL1 Bitmap:
  //   Bit 7: Header 2
  //   Bit 6: Accel Accuracy
  //   Bit 5: Gyro Accuracy
  //   Bit 4: Compass Accuracy
  //   Bit 3: Gyro Bias
  //   Bit 2: Compass Calibr (heading)
  //   Bit 1: Pressure (Altitude)
  //   Bit 0: ALS (second illumination light sensor)
  //
  // Set all to zero
  unsigned char data_out_ctl1[4] = {0x00, 0x00, 0x00, 0x00};
  result = writeDMPmems(DATA_OUT_CTL1, 4, &data_out_ctl1[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (DATA_OUT_CTL1) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }

  // Configure the DATA_OUT_CTL2 register
  // DATA_OUT_CTL2 Bitmap:
  //   Bit 7: Batch Mode Enable
  //   Bit 6: Pickup
  //   Bit 5: Activity Recognition Raw Accel
  //   Bit 4: Secondary On/Off
  //   Bit 3: Compass Bias
  //   Bit 2: Compass (Raw)
  //   Bit 1: Accelerometer Calibrated
  //   Bit 0: Gyro Calibrated
  //
  // Set all to zero
  unsigned char data_out_ctl2[4] = {0x00, 0x00, 0x00, 0x00};
  result = writeDMPmems(DATA_OUT_CTL2, 4, &data_out_ctl2[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (DATA_OUT_CTL2) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }

  // Configure the DATA_INTR_CTL register
  // DATA_INTR_CTL Bitmap:
  //   Bit 7: DMP Interrupt
  //   Bit 6: PICKup
  //   Bit 5: Activity Recognition Raw Accel
  //   Bit 4: Secondary On/Off
  //   Bit 3: Header 2
  //   Bit 2: Compass Calibr (heading)
  //   Bit 1: Gyro Calibrated
  //   Bit 0: Accel Calibrated
  //
  // Set DMP Interrupt to 1
  unsigned char data_intr_ctl[4] = {0x10, 0x00, 0x00, 0x00}; // DMP Interrupt
  result = writeDMPmems(DATA_INTR_CTL, 4, &data_intr_ctl[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (DATA_INTR_CTL) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }

  // Configure the MOTION_EVENT_CTL register
  // MOTION_EVENT_CTL Bitmap:
  //   Bit 7: Activity Change Enable
  //   Bit 6: BAC Wearable Enable
  //   Bit 5: Pedometer Enable
  //   Bit 4: Pedometer Interrupt
  //   Bit 3: SMD Enable
  //   Bit 2: BTS Enable
  //   Bit 1: Tilt Enable
  //   Bit 0: Pick Up Enable
  //
  // Leave all at zero
  unsigned char motion_event_ctl[4] = {0x00, 0x00, 0x00, 0x00}; // Set all to zero
  result = writeDMPmems(MOTION_EVENT_CTL, 4, &motion_event_ctl[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (MOTION_EVENT_CTL) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }

  // Configure the DATA_RDY_STATUS register
  // DATA_RDY_STATUS Bitmap (Also called DMP_INT_STATUS):
  //   Bit 7: Secondary On/Off Int
  //   Bit 6: SMD
  //   Bit 5: PED
  //   Bit 4: Pickup
  //   Bit 3: BTS
  //   Bit 2: Activity Recog
  //   Bit 1: Tilt
  //   Bit 0: BAC Wearable
  //
  // Set all to zero
  unsigned char data_rdy_status[2] = {0x00, 0x00}; // Set all to zero
  result = writeDMPmems(DATA_RDY_STATUS, 2, &data_rdy_status[0]);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: writeDMPmems (DATA_RDY_STATUS) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }

  // Set DMP Start Address
  result = setDMPstartAddress();
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: setDMPstartAddress returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }

  // Now load the DMP firmware
  result = loadDMPFirmware();
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: loadDMPFirmware returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }

  // Write the value 0x08 to the bank 0 register 0x06 known as LP_CONFIG
  // The InvenSense Nucleo example does this; the InvenSense Atmel example does not
  unsigned char lp_config[1] = {0x00};
  result = read(AGB0_REG_LP_CONFIG, lp_config, 1);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: read (LP_CONFIG) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }
  if (_printDebug)
  {
    printf("ICM_20948::initializeDMP: Bank 0 Reg 0x06 was: 0x%02X\n", lp_config[0]);
  }
  lp_config[0] |= 0x08;
  result = write(AGB0_REG_LP_CONFIG, lp_config, 1);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: write (LP_CONFIG) returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }
  if (_printDebug)
  {
    printf("ICM_20948::initializeDMP: Bank 0 Reg 0x06 is now: 0x%02X\n", lp_config[0]);
  }

  // Finally enable the DMP
  result = enableDMP(true);
  if (result != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948::initializeDMP: enableDMP returned: ");
      debugPrintStatus(result);
    }
    return result; // Bail
  }

  return result;
}

//Gyro Bias
ICM_20948_Status_e ICM_20948::setBiasGyroX(int32_t newValue)
{
  // Function not implemented
  // status = inv_icm20948_write_gyro_bias(&_device, newValue, 0x00);
  status = ICM_20948_Stat_NotImpl;
  return status;
}
ICM_20948_Status_e ICM_20948::setBiasGyroY(int32_t newValue)
{
  // Function not implemented
  // status = inv_icm20948_write_gyro_bias(&_device, newValue, 0x01);
  status = ICM_20948_Stat_NotImpl;
  return status;
}
ICM_20948_Status_e ICM_20948::setBiasGyroZ(int32_t newValue)
{
  // Function not implemented
  // status = inv_icm20948_write_gyro_bias(&_device, newValue, 0x02);
  status = ICM_20948_Stat_NotImpl;
  return status;
}
ICM_20948_Status_e ICM_20948::getBiasGyroX(int32_t* getValue)
{
  // Function not implemented
  // status = inv_icm20948_read_gyro_bias(&_device, getValue, 0x00);
  status = ICM_20948_Stat_NotImpl;
  return status;
}
ICM_20948_Status_e ICM_20948::getBiasGyroY(int32_t* getValue)
{
  // Function not implemented
  // status = inv_icm20948_read_gyro_bias(&_device, getValue, 0x01);
  status = ICM_20948_Stat_NotImpl;
  return status;
}
ICM_20948_Status_e ICM_20948::getBiasGyroZ(int32_t* getValue)
{
  // Function not implemented
  // status = inv_icm20948_read_gyro_bias(&_device, getValue, 0x02);
  status = ICM_20948_Stat_NotImpl;
  return status;
}
//Accel Bias
ICM_20948_Status_e ICM_20948::setBiasAccelX(int32_t newValue)
{
  // Function not implemented
  // status = inv_icm20948_write_accel_bias(&_device, newValue, 0x00);
  status = ICM_20948_Stat_NotImpl;
  return status;
}
ICM_20948_Status_e ICM_20948::setBiasAccelY(int32_t newValue)
{
  // Function not implemented
  // status = inv_icm20948_write_accel_bias(&_device, newValue, 0x01);
  status = ICM_20948_Stat_NotImpl;
  return status;
}
ICM_20948_Status_e ICM_20948::setBiasAccelZ(int32_t newValue)
{
  // Function not implemented
  // status = inv_icm20948_write_accel_bias(&_device, newValue, 0x02);
  status = ICM_20948_Stat_NotImpl;
  return status;
}
ICM_20948_Status_e ICM_20948::getBiasAccelX(int32_t* getValue)
{
  // Function not implemented
  // status = inv_icm20948_read_accel_bias(&_device, getValue, 0x00);
  status = ICM_20948_Stat_NotImpl;
  return status;
}
ICM_20948_Status_e ICM_20948::getBiasAccelY(int32_t* getValue)
{
  // Function not implemented
  // status = inv_icm20948_read_accel_bias(&_device, getValue, 0x01);
  status = ICM_20948_Stat_NotImpl;
  return status;
}
ICM_20948_Status_e ICM_20948::getBiasAccelZ(int32_t* getValue)
{
  // Function not implemented
  // status = inv_icm20948_read_accel_bias(&_device, getValue, 0x02);
  status = ICM_20948_Stat_NotImpl;
  return status;
}
//CPass Bias
ICM_20948_Status_e ICM_20948::setBiasCPassX(int32_t newValue)
{
  // Function not implemented
  // status = inv_icm20948_write_cpass_bias(&_device, newValue, 0x00);
  status = ICM_20948_Stat_NotImpl;
  return status;
}
ICM_20948_Status_e ICM_20948::setBiasCPassY(int32_t newValue)
{
  // Function not implemented
  // status = inv_icm20948_write_cpass_bias(&_device, newValue, 0x01);
  status = ICM_20948_Stat_NotImpl;
  return status;
}
ICM_20948_Status_e ICM_20948::setBiasCPassZ(int32_t newValue)
{
  // Function not implemented
  // status = inv_icm20948_write_cpass_bias(&_device, newValue, 0x02);
  status = ICM_20948_Stat_NotImpl;
  return status;
}
ICM_20948_Status_e ICM_20948::getBiasCPassX(int32_t* getValue)
{
  // Function not implemented
  // status = inv_icm20948_read_cpass_bias(&_device, getValue, 0x00);
  status = ICM_20948_Stat_NotImpl;
  return status;
}
ICM_20948_Status_e ICM_20948::getBiasCPassY(int32_t* getValue)
{
  // Function not implemented
  // status = inv_icm20948_read_cpass_bias(&_device, getValue, 0x01);
  status = ICM_20948_Stat_NotImpl;
  return status;
}
ICM_20948_Status_e ICM_20948::getBiasCPassZ(int32_t* getValue)
{
  // Function not implemented
  // status = inv_icm20948_read_cpass_bias(&_device, getValue, 0x02);
  status = ICM_20948_Stat_NotImpl;
  return status;
}

// I2C class

ICM_20948_I2C::ICM_20948_I2C()
{
}

// Initialize the I2C port
ICM_20948_Status_e ICM_20948_I2C::begin(i2c_inst_t *i2cPort, uint8_t addr)
{
  // Associate
  _i2c = i2cPort;
  _addr = addr;

  // Set up the serif
  _serif.write = ICM_20948_write_I2C;
  _serif.read = ICM_20948_read_I2C;
  _serif.user = (void *)this; // refer to yourself in the user field

  // Link the serif
  _device._serif = &_serif;

  // Initialize device structure
  _device._last_bank = -1; // Initialize the bank tracker to -1 (invalid)

  // Perform default startup
  status = startupDefault();
  if (status != ICM_20948_Stat_Ok)
  {
    if (_printDebug)
    {
      printf("ICM_20948_I2C::begin: startupDefault returned: ");
      debugPrintStatus(status);
    }
  }

  return status;
}

// I2C Read/Write Functions
ICM_20948_Status_e ICM_20948_write_I2C(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
  if (user == NULL)
  {
    return ICM_20948_Stat_ParamErr;
  }
  ICM_20948_I2C *classPtr = (ICM_20948_I2C *)user;
  
  ICM_20948 *basePtr = static_cast<ICM_20948*>(classPtr);
  if (basePtr->_printDebug)
  {
    printf("ICM_20948_write_I2C: reg 0x%02X, len %u\n", reg, (unsigned int)len);
  }

  // Combine register and data into a single buffer
  uint8_t buf[len + 1];
  buf[0] = reg;
  memcpy(&buf[1], data, len);

  // Perform the I2C write
  int result = i2c_write_blocking(classPtr->_i2c, classPtr->_addr, buf, len + 1, false);
  
  if (result == PICO_ERROR_GENERIC)
  {
    if (basePtr->_printDebug)
    {
      printf("ICM_20948_write_I2C: i2c_write_blocking failed\n");
    }
    return ICM_20948_Stat_Err;
  }

  return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e ICM_20948_read_I2C(uint8_t reg, uint8_t *buff, uint32_t len, void *user)
{
  if (user == NULL)
  {
    return ICM_20948_Stat_ParamErr;
  }
  ICM_20948_I2C *classPtr = (ICM_20948_I2C *)user;
  
  ICM_20948 *basePtr = static_cast<ICM_20948*>(classPtr);
  if (basePtr->_printDebug)
  {
    printf("ICM_20948_read_I2C: reg 0x%02X, len %u\n", reg, (unsigned int)len);
  }

  // Write the register address
  int result = i2c_write_blocking(classPtr->_i2c, classPtr->_addr, &reg, 1, true); // true keeps master control
  if (result == PICO_ERROR_GENERIC)
  {
    if (basePtr->_printDebug)
    {
      printf("ICM_20948_read_I2C: i2c_write_blocking (register) failed\n");
    }
    return ICM_20948_Stat_Err;
  }

  // Read the data
  result = i2c_read_blocking(classPtr->_i2c, classPtr->_addr, buff, len, false);
  if (result == PICO_ERROR_GENERIC)
  {
    if (basePtr->_printDebug)
    {
      printf("ICM_20948_read_I2C: i2c_read_blocking failed\n");
    }
    return ICM_20948_Stat_Err;
  }

  return ICM_20948_Stat_Ok;
}
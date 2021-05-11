#include "imu.h"
#include "system/delay/delay.h"

#define DRIVE_TYPE 1

static int16_t lsm6ds3tr_gyroRaw[3] = {0};
static int16_t lsm6ds3tr_accelRaw[3] = {0};
static int16_t lsm6ds3tr_temperatureRaw = 0;
static float lsm6ds3tr_angular_rate_mdps[3] = {0};
static float lsm6ds3tr_acceleration_mg[3] = {0};
static float lsm6ds3tr_temperature_degC = 0;

static int16_t lsm303ah_accelRaw[3] = {0};
static int16_t lsm303ah_magneticRaw[3] = {0};
static float lsm303ah_acceleration_mg[3] = {0};
static float lsm303ah_magnetic_mG[3] = {0};

stmdev_ctx_t lsm6ds3_dev_ctx;
stmdev_ctx_t lsm303_dev_ctx;

static int32_t platform_write(void *handle, uint8_t reg, uint8_t *pbuf, uint16_t len)
{
  uint8_t ret = 0;
  for (uint16_t i = 0; i < len; i++)
  {
    ret = IIC_WriteByte((uint8_t)((uint32_t)handle), reg++, pbuf++);
    if (ret != 0)
      return ret;
  }
  return ret;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *pbuf, uint16_t len)
{
  uint8_t ret = 0;
  for (uint16_t i = 0; i < len; i++)
  {
    ret = IIC_ReadByte((uint8_t)((uint32_t)handle), reg++, pbuf++);
    if (ret != 0)
      return ret;
  }
  return ret;
}

uint8_t lsm6ds3tr_init()
{
  uint8_t whoamI = 0, rst = 0;
  lsm6ds3_dev_ctx.write_reg = platform_write;
  lsm6ds3_dev_ctx.read_reg = platform_read;
  lsm6ds3_dev_ctx.handle = (void *)(LSM6DS3TR_C_I2C_ADD_L);

  lsm6ds3tr_c_device_id_get(&lsm6ds3_dev_ctx, &whoamI);
  if (whoamI != LSM6DS3TR_C_ID) /*manage here device not found */
  {
    return 1;
  }

  /* Restore default configuration */
  lsm6ds3tr_c_reset_set(&lsm6ds3_dev_ctx, PROPERTY_ENABLE);

  do
  {
    lsm6ds3tr_c_reset_get(&lsm6ds3_dev_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update */
  lsm6ds3tr_c_block_data_update_set(&lsm6ds3_dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  lsm6ds3tr_c_xl_data_rate_set(&lsm6ds3_dev_ctx, LSM6DS3TR_C_XL_ODR_1k66Hz);
  lsm6ds3tr_c_gy_data_rate_set(&lsm6ds3_dev_ctx, LSM6DS3TR_C_GY_ODR_1k66Hz);
  /* Set full scale */
  lsm6ds3tr_c_xl_full_scale_set(&lsm6ds3_dev_ctx, LSM6DS3TR_C_8g);
  lsm6ds3tr_c_gy_full_scale_set(&lsm6ds3_dev_ctx, LSM6DS3TR_C_1000dps);
  /* Configure filtering chain(No aux interface) */
  /* Accelerometer - analog filter */
  // lsm6ds3tr_c_xl_filter_analog_set(&lsm6ds3_dev_ctx, LSM6DS3TR_C_XL_ANA_BW_400Hz);
  /* Accelerometer - LPF1 path ( LPF2 not used )*/
  //lsm6ds3tr_c_xl_lp1_bandwidth_set(&lsm6ds3_dev_ctx, LSM6DS3TR_C_XL_LP1_ODR_DIV_4);
  /* Accelerometer - LPF1 + LPF2 path */
  // lsm6ds3tr_c_xl_lp2_bandwidth_set(&lsm6ds3_dev_ctx, LSM6DS3TR_C_XL_LOW_NOISE_LP_ODR_DIV_100);
  /* Accelerometer - High Pass / Slope path */
  //lsm6ds3tr_c_xl_reference_mode_set(&lsm6ds3_dev_ctx, PROPERTY_DISABLE);
  //lsm6ds3tr_c_xl_hp_bandwidth_set(&lsm6ds3_dev_ctx, LSM6DS3TR_C_XL_HP_ODR_DIV_100);
  /* Gyroscope - filtering chain */
  // lsm6ds3tr_c_gy_band_pass_set(&lsm6ds3_dev_ctx, LSM6DS3TR_C_HP_260mHz_LP1_STRONG);
  return 0;
}

uint8_t lsm6ds3tr_getData(int16_t *gyro_raw, int16_t *accel_raw, int16_t *temperature_raw)
{
  uint8_t result = 0;
  /* Read output only if new value is available */
  lsm6ds3tr_c_reg_t reg;
  lsm6ds3tr_c_status_reg_get(&lsm6ds3_dev_ctx, &reg.status_reg);

  if (reg.status_reg.gda)
  {
    /* Read magnetic field data */
    memset(gyro_raw, 0x00, 3 * sizeof(int16_t));
    lsm6ds3tr_c_angular_rate_raw_get(&lsm6ds3_dev_ctx, gyro_raw);
  }
  else
  {
    result |= (1 << 0);
  }

  if (reg.status_reg.xlda)
  {
    /* Read magnetic field data */
    memset(accel_raw, 0x00, 3 * sizeof(int16_t));
    lsm6ds3tr_c_acceleration_raw_get(&lsm6ds3_dev_ctx, accel_raw);
  }
  else
  {
    result |= (1 << 1);
  }

  if (reg.status_reg.tda)
  {
    /* Read temperature data */
    memset(temperature_raw, 0x00, sizeof(int16_t));
    lsm6ds3tr_c_temperature_raw_get(&lsm6ds3_dev_ctx, temperature_raw);
  }
  else
  {
    result |= (1 << 2);
  }
  return result;
}

uint8_t lsm303ah_init()
{
  uint8_t whoamI = 0;
  // uint8_t rst = 0;
  lsm303_dev_ctx.write_reg = platform_write;
  lsm303_dev_ctx.read_reg = platform_read;
  lsm303_dev_ctx.handle = (void *)(LSM303AH_I2C_ADD_XL);

  lsm303ah_xl_device_id_get(&lsm303_dev_ctx, &whoamI);
  // if (whoamI != LSM303AH_ID_XL) /*manage here device not found */
  // {
  //   return 1;
  // }
  /* Restore default configuration */
  lsm303ah_xl_reset_set(&lsm303_dev_ctx, PROPERTY_ENABLE);
  // do
  // {
  //   lsm303ah_xl_reset_get(&lsm303_dev_ctx, &rst);
  // } while (rst);
  delay_ms(5);

  /* Enable Block Data Update */
  lsm303ah_xl_block_data_update_set(&lsm303_dev_ctx, PROPERTY_ENABLE);
  /* Set full scale */
  lsm303ah_xl_full_scale_set(&lsm303_dev_ctx, LSM303AH_XL_8g);
  /* Set Output Data Rate */
  lsm303ah_xl_data_rate_set(&lsm303_dev_ctx, LSM303AH_XL_ODR_1k6Hz_HF);
  /* Configure filtering chain */
  /* Accelerometer - High Pass / Slope path */
  //lsm303ah_xl_hp_path_set(&dev_ctx_xl, LSM303AH_HP_ON_OUTPUTS);

  whoamI = 0;
  lsm303_dev_ctx.handle = (void *)(LSM303AH_I2C_ADD_MG);
  lsm303ah_mg_device_id_get(&lsm303_dev_ctx, &whoamI);
  // if (whoamI != LSM303AH_ID_MG) /*manage here device not found */
  // {
  //   return 2;
  // }
  /* Restore default configuration */
  lsm303ah_mg_reset_set(&lsm303_dev_ctx, PROPERTY_ENABLE);
  // do
  // {
  //   lsm303ah_mg_reset_get(&lsm303_dev_ctx, &rst);
  // } while (rst);
  delay_ms(5);

  /* Enable Block Data Update */
  lsm303ah_mg_block_data_update_set(&lsm303_dev_ctx, PROPERTY_ENABLE);
  /* Set / Reset magnetic sensor mode */
  lsm303ah_mg_set_rst_mode_set(&lsm303_dev_ctx, LSM303AH_MG_SENS_OFF_CANC_EVERY_ODR);
  /* Enable temperature compensation on mag sensor */
  lsm303ah_mg_offset_temp_comp_set(&lsm303_dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  lsm303ah_mg_data_rate_set(&lsm303_dev_ctx, LSM303AH_MG_ODR_100Hz);
  /* Set magnetometer in continuous mode */
  lsm303ah_mg_operating_mode_set(&lsm303_dev_ctx, LSM303AH_MG_CONTINUOUS_MODE);
  return 0;
}

uint8_t lsm303ah_getData(int16_t *accel_raw, int16_t *magnetic_raw)
{
  uint8_t result = 0;
  /* Read output only if new value is available */
  lsm303ah_reg_t reg;
  lsm303_dev_ctx.handle = (void *)(LSM303AH_I2C_ADD_XL);
  lsm303ah_xl_status_reg_get(&lsm303_dev_ctx, &reg.status_a);
  if (reg.status_a.drdy)
  {
    /* Read acceleration data */
    memset(accel_raw, 0x00, 3 * sizeof(int16_t));
    lsm303ah_acceleration_raw_get(&lsm303_dev_ctx, accel_raw);
  }
  else
  {
    result |= (1 << 0);
  }

  lsm303_dev_ctx.handle = (void *)(LSM303AH_I2C_ADD_MG);
  lsm303ah_mg_status_get(&lsm303_dev_ctx, &reg.status_reg_m);
  if (reg.status_reg_m.zyxda)
  {
    /* Read magnetic field data */
    memset(magnetic_raw, 0x00, 3 * sizeof(int16_t));
    lsm303ah_magnetic_raw_get(&lsm303_dev_ctx, magnetic_raw);
  }
  else
  {
    result |= (1 << 1);
  }
  return result;
}

uint8_t imu_init()
{
  uint8_t result = 0;
#if DRIVE_TYPE == 0
  result = lsm6ds3tr_init();
  if (result != 0)
    return result;
  result = lsm303ah_init();
#else
  Lsm6d3s_Configuration();
  Lsm303c_Configuration();
#endif
  return result;
}

void imu_convertData()
{
  lsm6ds3tr_getData(lsm6ds3tr_gyroRaw, lsm6ds3tr_accelRaw, &lsm6ds3tr_temperatureRaw);
  lsm6ds3tr_angular_rate_mdps[0] = lsm6ds3tr_c_from_fs1000dps_to_mdps(lsm6ds3tr_gyroRaw[0]);
  lsm6ds3tr_angular_rate_mdps[1] = lsm6ds3tr_c_from_fs1000dps_to_mdps(lsm6ds3tr_gyroRaw[1]);
  lsm6ds3tr_angular_rate_mdps[2] = lsm6ds3tr_c_from_fs1000dps_to_mdps(lsm6ds3tr_gyroRaw[2]);
  lsm6ds3tr_acceleration_mg[0] = lsm6ds3tr_c_from_fs8g_to_mg(lsm6ds3tr_accelRaw[0]);
  lsm6ds3tr_acceleration_mg[1] = lsm6ds3tr_c_from_fs8g_to_mg(lsm6ds3tr_accelRaw[1]);
  lsm6ds3tr_acceleration_mg[2] = lsm6ds3tr_c_from_fs8g_to_mg(lsm6ds3tr_accelRaw[2]);
  lsm6ds3tr_temperature_degC = lsm6ds3tr_c_from_lsb_to_celsius(lsm6ds3tr_temperatureRaw);

  lsm303ah_getData(lsm303ah_accelRaw, lsm303ah_magneticRaw);
  lsm303ah_acceleration_mg[0] = lsm303ah_from_fs8g_to_mg(lsm303ah_accelRaw[0]);
  lsm303ah_acceleration_mg[1] = lsm303ah_from_fs8g_to_mg(lsm303ah_accelRaw[1]);
  lsm303ah_acceleration_mg[2] = lsm303ah_from_fs8g_to_mg(lsm303ah_accelRaw[2]);
  lsm303ah_magnetic_mG[0] = lsm303ah_from_lsb_to_mgauss(lsm303ah_magneticRaw[0]);
  lsm303ah_magnetic_mG[1] = lsm303ah_from_lsb_to_mgauss(lsm303ah_magneticRaw[1]);
  lsm303ah_magnetic_mG[2] = lsm303ah_from_lsb_to_mgauss(lsm303ah_magneticRaw[2]);

  (void)lsm6ds3tr_angular_rate_mdps;
  (void)lsm6ds3tr_acceleration_mg;
  (void)lsm6ds3tr_temperature_degC;
  (void)lsm303ah_acceleration_mg;
  (void)lsm303ah_magnetic_mG;
}

uint8_t imu_getData(int16_t *imuData)
{
  uint8_t result = 0;
#if DRIVE_TYPE == 0
  result = lsm6ds3tr_getData(&imuData[0], &imuData[3], &lsm6ds3tr_temperatureRaw);
  if (result != 0)
    return result;
  result = lsm303ah_getData(lsm303ah_accelRaw, &imuData[6]);
#else
  uint8_t gryo_data[6] = {0};
  uint8_t acc_data[6] = {0};
  uint8_t mag_data[6] = {0};

  //gyro read
  if ((Lsm6d3s_ReadByte(LSM6DS3TR_C_ACC_GYRO_STATUS_REG) & 0x02) != 0)
  {
    Lsm6d3s_ReadLenByte(LSM6DS3TR_C_ACC_GYRO_OUTX_L_G, gryo_data, 6);
    imuData[0] = (gryo_data[1] << 8) | (gryo_data[0]);
    imuData[1] = (gryo_data[3] << 8) | (gryo_data[2]);
    imuData[2] = (gryo_data[5] << 8) | (gryo_data[4]);
  }

  //acc read
  if ((Lsm6d3s_ReadByte(LSM6DS3TR_C_ACC_GYRO_STATUS_REG) & 0x01) != 0)
  {
    Lsm6d3s_ReadLenByte(LSM6DS3TR_C_ACC_GYRO_OUTX_L_XL, acc_data, 6);

    imuData[3] = (acc_data[1] << 8) | (acc_data[0]);
    imuData[4] = (acc_data[3] << 8) | (acc_data[2]);
    imuData[5] = (acc_data[5] << 8) | (acc_data[4]);
  }

  //mag read
  if (LSM303C_MAG_R_NewXYZData(0, (LSM303C_MAG_ZYXDA_t *)&flag_LSM303C_MAG_XYZDA) != 0)
  {
    LSM303C_MAG_Get_Magnetic(0, mag_data);
    imuData[6] = (mag_data[1] << 8) | (mag_data[0]);
    imuData[7] = (mag_data[3] << 8) | (mag_data[2]);
    imuData[8] = (mag_data[5] << 8) | (mag_data[4]);
  }
#endif
  return result;
}

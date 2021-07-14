#include "MS5525DSO.h"

#define MS5525DSO_CMD_RESET     ((uint8_t)0x1e)
#define MS5525DSO_CMD_BASE_PROM ((uint8_t)0xa0)
#define MS5525DSO_CMD_CONVERT   ((uint8_t)0x40)

const uint8_t MS5525DSO::_Q_coeff[pp_MAXPART][6] =
{
  { 15, 17, 7, 5, 7, 21 }, // pp001DS
  { 14, 16, 8, 6, 7, 22 }, // pp002GS
  { 16, 18, 6, 4, 7, 22 }, // pp002DS
  { 16, 17, 6, 5, 7, 21 }, // pp005GS
  { 17, 19, 5, 3, 7, 22 }, // pp005DS
  { 16, 17, 6, 5, 7, 22 }, // pp015GS
  { 16, 17, 6, 5, 7, 22 }, // pp015AS
  { 17, 19, 5, 3, 7, 22 }, // pp015DS
  { 17, 18, 5, 4, 7, 22 }, // pp030AS
  { 17, 18, 5, 4, 7, 22 }, // pp030GS
  { 18, 21, 4, 1, 7, 22 }, // pp030DS
};

/* These #defines are only valid inside a class instance method */
#define Q1      (_Q_coeff[_partNum][0])
#define Q2      (_Q_coeff[_partNum][1])
#define Q3      (_Q_coeff[_partNum][2])
#define Q4      (_Q_coeff[_partNum][3])
#define Q5      (_Q_coeff[_partNum][4])
#define Q6      (_Q_coeff[_partNum][5])
#define P_SENS  (_PROM_coeff[0])
#define P_OFF   (_PROM_coeff[1])
#define TC_SENS (_PROM_coeff[2])
#define TC_OFF  (_PROM_coeff[3])
#define T_REF   (_PROM_coeff[4])
#define T_SENS  (_PROM_coeff[5])

MS5525DSO::MS5525DSO(MS5525DSO_part_t partNum, TwoWire *_aWire)
{
  _partNum = partNum;
  setOSR(MS5525DSO_OSR_4096);
  _wire = _aWire;
  _i2c_addr = 0;
}

void MS5525DSO::setOSR(uint8_t osr)
{
  _osr = osr & 0x0e;
  if (_osr > 8) _osr = 8;
}

bool MS5525DSO::begin(uint8_t addr)
{
  if (_wire == NULL) return false;
  _i2c_addr = addr;
  return _begin_common();
}

bool MS5525DSO::_begin_common(void)
{
  bool success = true;
  
  reset();

  // Read PROM calibration parameters 1 through 6
  for (uint8_t i = 1; i <= 6; i++) {
    if (!_read_prom(i, &_PROM_coeff[i - 1])) success = false;
  }
  return success;
}

void MS5525DSO::dumpCoefficients(Stream & s)
{
  s.printf("C1 - Pressure Sensitivity                            : %u\r\n", P_SENS);
  s.printf("C2 - Pressure Offset                                 : %u\r\n", P_OFF);
  s.printf("C3 - Temperature Coefficient of Pressure Sensitivity : %u\r\n", TC_SENS);
  s.printf("C4 - Temperature Coefficient of Pressure Offset      : %u\r\n", TC_OFF);
  s.printf("C5 - Reference Temperature                           : %u\r\n", T_REF);
  s.printf("C6 - Temperature Coefficient of Temperature          : %u\r\n", T_SENS);
}

bool MS5525DSO::readPressureAndTemperature(double * pressure, double * temperature)
{
  uint32_t raw[2];  // index 0 is raw pressure, index 1 is raw temperature

  for (uint8_t i = 0; i < 2; i++) {
    _convert_D(i);

    // Delay the maximum expected time depending on OSR
    switch (_osr) {
    case MS5525DSO_OSR_256:
      delay(1);
      break;
    case MS5525DSO_OSR_512:
      delay(2);
      break;
    case MS5525DSO_OSR_1024:
      delay(3);
      break;
    case MS5525DSO_OSR_2048:
      delay(5);
      break;
    case MS5525DSO_OSR_4096:
    default:
      delay(10);
      break;
    }

    if (!_read_adc(&raw[i])) return false;
  }

  // Difference between actual and reference temperature
  int64_t dT = raw[1] - ((int64_t)T_REF << Q5);

  // Offset at actual temperature
  int64_t off = ((int64_t)P_OFF << Q2) + ((TC_OFF * dT) >> Q4);

  // Sensitivity at actual temperature
  int64_t sens = ((int64_t)P_SENS << Q1) + ((TC_SENS * dT) >> Q3);

  // Temperature compensated pressure
  int64_t tc_press = (((sens * raw[0]) >> 21) - off) >> 15;
  *pressure = tc_press * 0.0001;

  // Calculate cooked temperature if requested
  if (temperature != NULL) {
    *temperature = (2000 + ((dT * T_SENS) >> Q6)) * 0.01;
  }

  return true;
}

bool MS5525DSO::_read_prom(uint8_t i, uint16_t * c)
{
  // Address of PROM parameter to read
  _wire->beginTransmission(_i2c_addr);
  _wire->write(MS5525DSO_CMD_BASE_PROM | (i << 1));
  _wire->endTransmission();

  if (_wire->requestFrom(_i2c_addr, (uint8_t)2) < 2) {
    return false;
  }

  // Read 2 bytes of the coefficient, MSB first
  *c = 0;
  for (auto n = 0; n < 2; n++) {
    *c = (*c << 8) | _wire->read();
  }
  return true;
}

void MS5525DSO::_convert_D(uint8_t d)
{
  _wire->beginTransmission(_i2c_addr);
  _wire->write(MS5525DSO_CMD_CONVERT | ((d & 0x01) << 4) | _osr);
  _wire->endTransmission();
}

bool MS5525DSO::_read_adc(uint32_t * c)
{
  _wire->beginTransmission(_i2c_addr);
  _wire->write((uint8_t)0x00);
  _wire->endTransmission();
  
  if (_wire->requestFrom(_i2c_addr, (uint8_t)3) < 3) {
    return false;
  }

  // Read 3 bytes of the ADC result, MSB first
  *c = 0;
  for (auto n = 0; n < 3; n++) {
    *c = (*c << 8) | _wire->read();
  }

  return true;
}

void MS5525DSO::reset(void)
{
  _wire->beginTransmission(_i2c_addr);
  _wire->write(MS5525DSO_CMD_RESET);
  _wire->endTransmission();

  delay(10);
}

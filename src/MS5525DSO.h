/**
 * Library for the MS5525DSO series pressure sensor from MEAS.
 * 
 * This sensor uses either I2C or SPI to communicate. In I2C mode, this device
 * uses either address 0x76 or 0x77 for communication, at a maximum clock speed
 * of 400KHz. The code will not change the current clock speed, but will use the
 * one currently in effect.
 * 
 * Alex Villacís Lasso (Palosanto Solutions), as part of the OpenVenti project.
 * 
 */
#ifndef __MS5525DSO_H__
#define __MS5525DSO_H__

//#include <SPI.h>
#include <Wire.h>

#define I2C_MS5525DSO_ADDR      0x76  /* Sensor address, CSB pin HIGH */
#define I2C_MS5525DSO_ADDR_ALT  0x77  /* Sensor address, CSB pin LOW */

/* DO NOT reorder or add entries to this model enumeration unless the
 * quotient table is updated as well. */
typedef enum
{
  pp001DS = 0,
  pp002GS,
  pp002DS,
  pp005GS,
  pp005DS,
  pp015GS,
  pp015AS,
  pp015DS,
  pp030AS,
  pp030GS,
  pp030DS,

  pp_MAXPART
} MS5525DSO_part_t;

/* Macros for oversampling ratio settings */
#define MS5525DSO_OSR_256   0
#define MS5525DSO_OSR_512   2
#define MS5525DSO_OSR_1024  4
#define MS5525DSO_OSR_2048  6
#define MS5525DSO_OSR_4096  8

class MS5525DSO
{
private:
  /* Table of coefficients that depend on the part number */
  static const uint8_t _Q_coeff[pp_MAXPART][6];

  /* Table of parameters read from PROM on startup */
  uint16_t _PROM_coeff[6];

  /* Selected part code to be managed */
  MS5525DSO_part_t _partNum;

  /* Oversampling ratio (OSR) selection for ADC reading */
  uint8_t _osr;

  /* I2C interface to use for operation in I2C mode */
  TwoWire * _wire;
  uint8_t _i2c_addr;

  bool _begin_common(void);
  bool _read_prom(uint8_t, uint16_t *);
  void _convert_D(uint8_t);
  bool _read_adc(uint32_t *);
public:
  /* Constructor for sensor in I2C mode. The I2C address will be specified in the begin() method. */
  MS5525DSO(MS5525DSO_part_t partNum, TwoWire *_aWire = &Wire);  

  /* Constructor for sensor in SPI mode. */
  //MS5525DSO(MS5525DSO_part_t partNum, uint8_t cs, SPIClass *_spi = &SPI);

  uint8_t getOSR(void)  { return _osr; }
  void setOSR(uint8_t);

  /* WARNING: the library will *not* call Wire.begin() for you. This should be done before
   * calling the following 3 methods. */
  bool begin(uint8_t addr = I2C_MS5525DSO_ADDR);
  void reset(void);

  /* Pressure returned in psi, temperature returned in degrees Celsius, */
  bool readPressureAndTemperature(double * pressure, double * temperature = NULL);

  /* The following methods are for debugging only */
  void dumpCoefficients(Stream &);
};

#endif

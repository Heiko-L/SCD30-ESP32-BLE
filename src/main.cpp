#include "Arduino.h"
#include "sdkconfig.h"
#include "esp_pm.h"
#include "Sensirion_Gadget_BLE.h"
#include <SensirionI2cScd4x.h>
#include <string>

// *** declarations *** //

enum measurement_type
{
  low_power,
  high_performance,
  idle_single_shot,
  power_cycled_single_shot
};

// Callback functions
void OnForcedRecalibration(std::string value);
void OnIntervalChange(std::string value);
void OnAltitudeChange(std::string value);
void OnTempOffsetChange(std::string value);
void OnASCInitInterval(std::string value);
void OnASCInterval(std::string value);
void OnASCEnable(std::string value);
void OnASCTarget(std::string value);

int16_t StartPeriodicMeasurement(enum measurement_type type);
void PrintError(int16_t error, char *reason = nullptr);

// *** globals *** //

static uint16_t g_persistentTempOffsetTicks;
static uint16_t g_persistentAltitude;
static uint16_t g_persistentSelfCalEnable;
static uint16_t g_persistentSelfCalTarget;
static uint16_t g_persistentAutoCalInitPeriod;
static uint16_t g_persistentAutoCalStandardPeriod;
static uint64_t g_persistentSerial;
static char errorMessage[128];
static int16_t error;
static enum measurement_type measurementType = high_performance;
static int measurementIntervalMs = 5000;
static int64_t lastMeasurementTimeMs = 0;

NimBLELibraryWrapper lib;
SCD4xDataProvider provider(lib, DataType::T_RH_CO2);
SensirionI2cScd4x sensor;

void PrintError(int16_t error, char *reason)
{
  if (reason)
  {
    Serial.print("In ");
    Serial.print(reason);
    Serial.print(" ");
  }
  Serial.print("Error: ");
  errorToString(error, errorMessage, sizeof errorMessage);
  Serial.println(errorMessage);
}

int16_t StartPeriodicMeasurement(enum measurement_type type)
{
  int16_t error;
  switch (type)
  {
  case low_power:
    error = sensor.startLowPowerPeriodicMeasurement();
    break;
  case high_performance:
    error = sensor.startPeriodicMeasurement();
    break;
  default:
    error = 1;
    break;
  }
  return error;
}

void getPersistentData(void)
{
  sensor.getTemperatureOffsetRaw(g_persistentTempOffsetTicks);
  sensor.getSensorAltitude(g_persistentAltitude);
  sensor.getAutomaticSelfCalibrationTarget(g_persistentSelfCalTarget);
  sensor.getAutomaticSelfCalibrationEnabled(g_persistentSelfCalEnable);
  sensor.getAutomaticSelfCalibrationInitialPeriod(g_persistentAutoCalInitPeriod);
  sensor.getAutomaticSelfCalibrationStandardPeriod(g_persistentAutoCalStandardPeriod);
  sensor.getSerialNumber(g_persistentSerial);
}

void printPersistentData(void)
{
  Serial.print("Sensirion SCD4x ID: 0x");
  Serial.println(g_persistentSerial, HEX);
  Serial.print("Temperature Offset: ");
  Serial.print(g_persistentTempOffsetTicks * 175.0 / 65536.0);
  Serial.print("°C (");
  Serial.print(g_persistentTempOffsetTicks);
  Serial.println(" Ticks)");
  Serial.print("Altitude setting: ");
  Serial.print(g_persistentAltitude);
  Serial.println("m above sea level");
  Serial.print("Automatic Self Calibration: ");
  if (g_persistentSelfCalEnable)
  {
    Serial.print(g_persistentSelfCalTarget);
    Serial.println("ppm");
  }
  else
  {
    Serial.println("off");
  }
  Serial.print("Initial Period of ASC: ");
  Serial.print(g_persistentAutoCalInitPeriod);
  Serial.println("h");
  Serial.print("Standard Period of ASC: ");
  Serial.print(g_persistentAutoCalStandardPeriod);
  Serial.println("h");
}

void setup()
{
  Serial.begin(115200);
#if CONFIG_PM_ENABLE
  // Configure dynamic frequency scaling:
  // maximum and minimum frequencies are set in sdkconfig,
  // automatic light sleep is enabled if tickless idle support is enabled.
  esp_pm_config_t pm_config = {
      .max_freq_mhz = 240,
      .min_freq_mhz = 80,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
      .light_sleep_enable = true
#endif
  };
  ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
#endif // CONFIG_PM_ENABLE
  Serial.flush();
  // pinMode(2, OUTPUT);
  // digitalWrite(2, HIGH);
  Serial.println("Hallo!");
  // Initialize the SCD driver
  Wire.begin();
  sensor.begin(Wire, 0x62);
  sensor.stopPeriodicMeasurement();
  getPersistentData();
  printPersistentData();

  // Initialize the GadgetBle Library
  provider.enableMeasurementIntervalCharacteristic(OnIntervalChange);
  provider.enableTempOffsetCharacteristic(OnTempOffsetChange);
  provider.enableAltitudeCharacteristic(OnAltitudeChange);
  provider.enableForcedRecalibrationCharacteristic(OnForcedRecalibration);
  provider.enableASCCharacteristic(OnASCEnable);
  provider.enableASCTargetCharacteristic(OnASCTarget);
  provider.enableASCInitIntervalCharacteristic(OnASCInitInterval);
  provider.enableASCIntervalCharacteristic(OnASCInterval);
  provider.begin();
  provider.setASCInitInterval(g_persistentAutoCalInitPeriod);
  provider.setASCInterval(g_persistentAutoCalStandardPeriod);
  provider.setTempOffset(g_persistentTempOffsetTicks);
  provider.setASCStatus(g_persistentSelfCalEnable);
  provider.setASCTarget(g_persistentSelfCalTarget);
  provider.setMeasurementInterval(measurementIntervalMs);
  provider.setAltitude(g_persistentAltitude);

  Serial.print("Sensirion GadgetBle Lib initialized with deviceId = ");
  Serial.println(provider.getDeviceIdString());

  error = StartPeriodicMeasurement(measurementType);
  if (error)
  {
    PrintError(error, "StartPeriodicMeasurement");
    return;
  }
}

void loop()
{
  static uint16_t co2Concentration = 0.0;
  static float temperature = 0.0;
  static float humidity = 0.0;
  bool dataReadyFlag = 0;
  uint32_t nextDelay = 0;
  if (millis() - lastMeasurementTimeMs >= measurementIntervalMs)
  {
    sensor.getDataReadyStatus(dataReadyFlag);
    if (dataReadyFlag)
    {
      error = sensor.readMeasurement(co2Concentration, temperature,
                                     humidity);
      if (error)
      {
        PrintError(error, "readMeasurementData");
        return;
      }
      lastMeasurementTimeMs = millis();
      provider.writeValueToCurrentSample(
          co2Concentration, SignalType::CO2_PARTS_PER_MILLION);
      provider.writeValueToCurrentSample(
          temperature, SignalType::TEMPERATURE_DEGREES_CELSIUS);
      provider.writeValueToCurrentSample(
          humidity, SignalType::RELATIVE_HUMIDITY_PERCENTAGE);
      provider.commitSample();

      // Provide the sensor values for Tools -> Serial Monitor or Serial
      // Plotter
      Serial.print("CO2[ppm]:");
      Serial.print(co2Concentration);
      Serial.print("\t");
      Serial.print("Temperature[°C]:");
      Serial.print(temperature);
      Serial.print("\t");
      Serial.print("Humidity[%]:");
      Serial.println(humidity);
    }
  }
  else
  {
    nextDelay = 20;
  }
  if (provider.isDownloading())
  {
    provider.handleDownload();
    nextDelay = 3;
  }
  else
  {
    if (nextDelay == 0)
    {
      nextDelay = measurementIntervalMs;
    }
  }
  delay(nextDelay);
}

void OnForcedRecalibration(std::string value)
{
  // co2 level is encoded in lower two bytes, little endian
  // the first two bytes are obfuscation and can be ignored
  // using nRF Connect write characterisic as UINT32 (litle endian)
  // with value = co2reference * 2^16
  uint16_t referenceCO2Level = value[2] | (value[3] << 8);
  uint16_t correctionValue;

  Serial.print("FRC requested with value: ");
  Serial.println(referenceCO2Level);
  error = sensor.stopPeriodicMeasurement();
  if (error)
  {
    PrintError(error);
  }
  delay(500);
  error = sensor.performForcedRecalibration(referenceCO2Level, correctionValue);
  if (error)
  {
    PrintError(error);
  }
  Serial.print("Returned correction value: ");
  Serial.println(correctionValue);
  StartPeriodicMeasurement(measurementType);
}

void OnIntervalChange(std::string value)
{
  uint16_t error;

  // using nRF Connect write characterisic as UINT32 (litle endian)
  uint16_t interval = value[0] | (value[1] << 8);
  Serial.print("Interval requested with value: ");
  Serial.println(interval);
  if (interval <= 5000)
  {
    Serial.println("Enabling high performance mode with 5s sampling period");
    measurementIntervalMs = 5000;
    // g_persistentAutoCalInitPeriod = 48;
    // g_persistentAutoCalStandardPeriod = 168;
    measurementType = high_performance;
  }
  else
  {
    Serial.println("Enabling low power mode with 30s sampling period");
    measurementIntervalMs = 30000;
    // g_persistentAutoCalInitPeriod = 8;
    // g_persistentAutoCalStandardPeriod = 28;
    measurementType = low_power;
  }
  // Serial.println("Stopping Measurements");
  // error = sensor.stopPeriodicMeasurement();
  // if (error) {PrintError(error);}
  // Serial.println("Writing new ASC Values");
  // sensor.setAutomaticSelfCalibrationInitialPeriod(g_persistentAutoCalInitPeriod);
  // sensor.setAutomaticSelfCalibrationStandardPeriod(g_persistentAutoCalStandardPeriod);
  Serial.println("Restarting periodic measurements");
  // provider.sendASCInitInterval(g_persistentAutoCalInitPeriod);
  // provider.sendASCInterval(g_persistentAutoCalStandardPeriod);
  provider.setMeasurementInterval(measurementIntervalMs);
  // sensor.persistSettings();
  StartPeriodicMeasurement(measurementType);
}

void OnAltitudeChange(std::string value)
{
  // using nRF Connect write characterisic as UINT32 (litle endian)
  uint16_t altitude = value[0] | (value[1] << 8);
  Serial.print("Altitude requested with value: ");
  Serial.println(altitude);
  Serial.println("Stopping Measurements");
  error = sensor.stopPeriodicMeasurement();
  if (error)
  {
    PrintError(error);
  }
  Serial.println("Writing new altitude data");
  error = sensor.setSensorAltitude(altitude);
  if (error)
  {
    PrintError(error);
  }
  error = sensor.persistSettings();
  if (error)
  {
    PrintError(error);
  }
  provider.setAltitude(altitude);
  StartPeriodicMeasurement(measurementType);
}

void OnTempOffsetChange(std::string value)
{
  // using nRF Connect write characterisic as UINT32 (litle endian)
  uint16_t tempoffset = value[0] | (value[1] << 8);
  Serial.print("Temp offset requested with value: ");
  Serial.println(tempoffset);
  Serial.println("Stopping Measurements");
  error = sensor.stopPeriodicMeasurement();
  if (error)
  {
    PrintError(error);
  }
  Serial.println("Writing new temperature Offset");
  error = sensor.setTemperatureOffsetRaw(tempoffset);
  if (error)
  {
    PrintError(error);
  }
  error = sensor.persistSettings();
  if (error)
  {
    PrintError(error);
  }
  provider.setTempOffset(tempoffset);
  StartPeriodicMeasurement(measurementType);
}

void OnASCInitInterval(std::string value)
{
  // using nRF Connect write characterisic as UINT32 (litle endian)
  uint16_t tempoffset = value[0] | (value[1] << 8);
  Serial.print("ASC Init Interval requested with value: ");
  Serial.println(tempoffset);
  Serial.println("Stopping Measurements");
  error = sensor.stopPeriodicMeasurement();
  if (error)
  {
    PrintError(error);
  }
  Serial.println("Writing new ASC Initial Interval");
  error = sensor.setAutomaticSelfCalibrationInitialPeriod(tempoffset);
  if (error)
  {
    PrintError(error);
  }
  error = sensor.persistSettings();
  if (error)
  {
    PrintError(error);
  }
  provider.setASCInitInterval(tempoffset);
  StartPeriodicMeasurement(measurementType);
}

void OnASCInterval(std::string value)
{
  // using nRF Connect write characterisic as UINT32 (litle endian)
  uint16_t tempoffset = value[0] | (value[1] << 8);
  Serial.print("ASC Interval requested with value: ");
  Serial.println(tempoffset);
  Serial.println("Stopping Measurements");
  error = sensor.stopPeriodicMeasurement();
  if (error)
  {
    PrintError(error);
  }
  Serial.println("Writing new ASC Interval");
  error = sensor.setAutomaticSelfCalibrationStandardPeriod(tempoffset);
  if (error)
  {
    PrintError(error);
  }
  error = sensor.persistSettings();
  if (error)
  {
    PrintError(error);
  }
  provider.setASCInterval(tempoffset);
  StartPeriodicMeasurement(measurementType);
}

void OnASCTarget(std::string value)
{
  // using nRF Connect write characterisic as UINT32 (litle endian)
  uint16_t tempoffset = value[0] | (value[1] << 8);
  Serial.print("New ASC target: ");
  Serial.println(tempoffset);
  Serial.println("Stopping Measurements");
  error = sensor.stopPeriodicMeasurement();
  if (error)
  {
    PrintError(error);
  }
  Serial.println("Set ASC with target ppm");
  error = sensor.setAutomaticSelfCalibrationTarget(tempoffset);
  if (error)
  {
    PrintError(error);
  }
  error = sensor.persistSettings();
  if (error)
  {
    PrintError(error);
  }
  provider.setASCTarget(tempoffset);
  StartPeriodicMeasurement(measurementType);
}

void OnASCEnable(std::string value)
{
  // using nRF Connect write characterisic as UINT32 (litle endian)
  uint16_t tempoffset = value[0] | (value[1] << 8);
  if (tempoffset)
  {
    tempoffset = 1;
  }
  Serial.print("ASC request: ");
  Serial.println(tempoffset);
  Serial.println("Stopping Measurements");
  error = sensor.stopPeriodicMeasurement();
  if (error)
  {
    PrintError(error);
  }
  Serial.println("Set ASC");
  error = sensor.setAutomaticSelfCalibrationEnabled(tempoffset);
  if (error)
  {
    PrintError(error);
  }
  error = sensor.persistSettings();
  if (error)
  {
    PrintError(error);
  }
  provider.setASCStatus(tempoffset);
  StartPeriodicMeasurement(measurementType);
}

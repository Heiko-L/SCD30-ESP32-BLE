#include "Arduino.h"
#include "sdkconfig.h"
#include "esp_pm.h"
#include "Sensirion_Gadget_BLE.h"
#include <SensirionI2cScd30.h>
#include <string>

// *** declarations *** //

// Callback functions
void OnForcedRecalibration(std::string value);
void OnIntervalChange(std::string value);
void OnAltitudeChange(std::string value);
void OnTempOffsetChange(std::string value);
void OnASCEnable(std::string value);

void PrintError(int16_t error, const char *reason = nullptr);

// *** globals *** //

static uint16_t g_persistentTempOffsetTicks;
static uint16_t g_persistentAltitude;
static uint16_t g_persistentSelfCalEnable;
static uint16_t g_persistentInterval;
static uint16_t g_persistentFRC;
static char errorMessage[128];
static int16_t error;
static int measurementIntervalMs = 5000;
static int64_t lastMeasurementTimeMs = 0;

NimBLELibraryWrapper lib;
SCD4xDataProvider provider(lib, DataType::T_RH_CO2);
SensirionI2cScd30 sensor;

void PrintError(int16_t error, const char *reason)
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

void getPersistentData(void)
{
  sensor.getTemperatureOffset(g_persistentTempOffsetTicks);
  sensor.getAltitudeCompensation(g_persistentAltitude);
  sensor.getAutoCalibrationStatus(g_persistentSelfCalEnable);
  sensor.getForceRecalibrationStatus(g_persistentFRC);
  sensor.getMeasurementInterval(g_persistentInterval);
  measurementIntervalMs = g_persistentInterval * 1000;
}

void printPersistentData(void)
{
  Serial.println("Sensirion SCD31 stored data:");
  Serial.print("Temperature Offset: ");
  Serial.print(g_persistentTempOffsetTicks * 100.0);
  Serial.print("°C (");
  Serial.print(g_persistentTempOffsetTicks);
  Serial.println(" Ticks)");
  Serial.print("Altitude setting: ");
  Serial.print(g_persistentAltitude);
  Serial.println("m above sea level");
  Serial.print("Automatic Self Calibration: ");
  if (g_persistentSelfCalEnable)
  {
    Serial.println("on");
  }
  else
  {
    Serial.println("off");
  }
  Serial.print("Forced Recalibration Value: ");
  Serial.print(g_persistentFRC);
  Serial.println(" ppm");
  Serial.print("Measurement Interval: ");
  Serial.print(g_persistentInterval);
  Serial.println(" s");
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
  provider.begin();
  provider.setTempOffset(g_persistentTempOffsetTicks);
  provider.setASCStatus(g_persistentSelfCalEnable);
  provider.setMeasurementInterval(measurementIntervalMs);
  provider.setAltitude(g_persistentAltitude);

  Serial.print("Sensirion GadgetBle Lib initialized with deviceId = ");
  Serial.println(provider.getDeviceIdString());

  error = sensor.startPeriodicMeasurement(0);
  if (error)
  {
    PrintError(error, "StartPeriodicMeasurement");
    return;
  }
}

void loop()
{
  static float co2Concentration = 0.0;
  static float temperature = 0.0;
  static float humidity = 0.0;
  uint16_t dataReadyFlag = 0;
  uint32_t nextDelay = 0;
  if (millis() - lastMeasurementTimeMs >= measurementIntervalMs)
  {
    sensor.getDataReady(dataReadyFlag);
    if (dataReadyFlag)
    {
      error = sensor.readMeasurementData(co2Concentration, temperature,
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
  error = sensor.forceRecalibration(referenceCO2Level);
  if (error)
  {
    PrintError(error);
  }
  error = sensor.startPeriodicMeasurement(0);
  if (error)
  {
    PrintError(error);
  }
}

void OnIntervalChange(std::string value)
{
  uint16_t error;

  // using nRF Connect write characterisic as UINT32 (litle endian)
  uint16_t interval = value[0] | (value[1] << 8);
  Serial.print("Interval requested with value: ");
  Serial.println(interval);
  if (interval < 2000)
  {
    interval = 2000;
  }
  Serial.println("Restarting periodic measurements");
  error = sensor.stopPeriodicMeasurement();
  if (error)
  {
    PrintError(error);
  }
  g_persistentInterval = interval / 1000;
  measurementIntervalMs = g_persistentInterval * 1000;
  provider.setMeasurementInterval(measurementIntervalMs);
  error = sensor.setMeasurementInterval(g_persistentInterval);
  if (error)
  {
    PrintError(error);
  }
  error = sensor.startPeriodicMeasurement(0);
  if (error)
  {
    PrintError(error);
  }
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
  error = sensor.setAltitudeCompensation(altitude);
  if (error)
  {
    PrintError(error);
  }
  provider.setAltitude(altitude);
  error = sensor.startPeriodicMeasurement(0);
  if (error)
  {
    PrintError(error);
  }
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
  error = sensor.setTemperatureOffset(tempoffset);
  if (error)
  {
    PrintError(error);
  }
  provider.setTempOffset(tempoffset);
  error = sensor.startPeriodicMeasurement(0);
  if (error)
  {
    PrintError(error);
  }
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
  error = sensor.activateAutoCalibration(tempoffset);
  if (error)
  {
    PrintError(error);
  }
  provider.setASCStatus(tempoffset);
  error = sensor.startPeriodicMeasurement(0);
  if (error)
  {
    PrintError(error);
  }
}

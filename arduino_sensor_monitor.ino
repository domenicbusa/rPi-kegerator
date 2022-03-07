// 03MAR2022 - D. Busa - 1.0 - initial release
// 07MAR2022 - D. Busa - 1.1 - added reading of temperature sensors
// PURPOSE : measure current from YHDC current transducer and send JSON payload over serial to raspberry pi
// NOTES
//      this only measures apparent power, no input taken for line voltage. it is assumed constant by VOLTAGE global var

#include <Adafruit_ADS1X15.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>

// Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */

/*  ==================================
 *  =========== GLOBAL VARS ==========
 *  ==================================
 */
// CT sensor
const float FACTOR = 20; //20A/1V - type of current transducer
const float multiplier = 1.000F;    /* ADS1015 @ +/- 2.048V gain (12-bit results), 1 bit = 1 mV */
const float VOLTAGE = 120.0F; // line voltage wire on current transducer

// Temperature sensors
String temp_names[] {"intTemp","extTemp","compTemp"}; // sensor names
OneWire ds18x20[] = { 0, D3, D4 }; // sensor pins
const int oneWireCount = sizeof(ds18x20)/sizeof(OneWire);
DallasTemperature sensor[oneWireCount];

// Define size of JSON object for serial communication
StaticJsonDocument<384> doc; // JSON object

/*  ==================================
 *  =========== SETUP ================
 *  ==================================
 */
void setup(void)
{
  Serial.begin(9600);
  
  // Set up A/D converter for CT measurements
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  //ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
  // Set up Temperature Sensors - Start up the OneWire library on all defined bus-wires
  DeviceAddress deviceAddress;
  for (int i = 0; i < oneWireCount; i++) {;
    sensor[i].setOneWire(&ds18x20[i]);
    sensor[i].begin();
    if (sensor[i].getAddress(deviceAddress, 0)) sensor[i].setResolution(deviceAddress, 12);
  }
}

/*  ==================================
 *  =========== MAIN LOOP ============
 *  ==================================
 */
// TODO : create a request/reply architecture

void loop(void)
{
  // Read CT - measure current and calculate power
  float currentRms = getCurrent();
  float powerAvg = VOLTAGE * currentRms;

  // Read Temperature sensors
  float tempsF[oneWireCount]; // initialize array
  getTemperatures(tempsF); // get temperature from each sensor on oneWire buses

  // recreate array in JSON doc
  JsonArray array = doc.to<JsonArray>();
  // add sensor dicts to JSON array
  add_ctSensor_to_json(array, "ct0", currentRms, VOLTAGE, powerAvg);
  add_tempSensors_to_json(array, temp_names, tempsF);

  // Generate the minified JSON and send it to the Serial port.
  serializeJson(doc, Serial);
  // Start a new line
  Serial.println();

  // clear JSON contents for new data
  doc.clear();
  
  delay(1000);
}

/*  ==================================
 *  =========== FUNCTIONS ============
 *  ==================================
 */
 
void printMeasure(String prefix, float value, String postfix)
{
  Serial.print(prefix);
  Serial.print(value, 3);
  Serial.println(postfix);
}

// Read current from CT, calculate Irms and PAvg
float getCurrent()
{
  float ct_voltage;
  float current;
  float sum = 0;
  long time = millis();
  int counter = 0;

  while (millis() - time < 1000)
  {
    ct_voltage = ads.readADC_Differential_0_1() * multiplier; // millivolts
    ct_voltage /= 1000; // volts
    current = ct_voltage * FACTOR; // convert measure volts to current
    sum += sq(current);
    counter += 1; // increment counter
  }
  current = sqrt(sum / counter); // root of the squared mean
  
  return(current);
}

// Read temperature from temperature sensors
void getTemperatures(float (& tempArray) [oneWireCount])
{
  // Request temperatures to all devices
  for (int i = 0; i < oneWireCount; i++) {
    sensor[i].requestTemperatures();
  }
  // Read temperatures
  for (int i = 0; i < oneWireCount; i++) {
    float tempF = sensor[i].getTempFByIndex(0);
    tempArray[i] = tempF;
  }
}

// Adds CT sensor information to JSON object
JsonArray add_ctSensor_to_json(JsonArray &_array, String _name, float _Irms, float _Vrms, float _Pavg)
{
  JsonObject obj = _array.createNestedObject();
  obj["name"] = _name;
  JsonObject objVals = obj.createNestedObject("vals");
  objVals["Irms"] = _Irms;
  objVals["Vrms"] = _Vrms;
  objVals["Pavg"] = _Pavg;

  return(_array);
}

// Adds temperature sensor information to JSON object
JsonArray add_tempSensors_to_json(JsonArray &_array, String _names[oneWireCount], float _tempsF[oneWireCount])
{
  for (int i = 0; i < oneWireCount; i++) {
    JsonObject obj = _array.createNestedObject();
    obj["name"] = _names[i];
    JsonObject objVals = obj.createNestedObject("vals");
    objVals["tempF"] = _tempsF[i];
  }

  return(_array);
}

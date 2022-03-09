// 03MAR2022 - D. Busa - 1.0 - initial release
// 07MAR2022 - D. Busa - 1.1 - added reading of temperature sensors and output via JSON serial
// 07MAR2022 - D. Busa - 1.2 - added serial request response architecture
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
OneWire ds18x20[] = { D3, D4, D5 }; // sensor pins
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
    Serial.println("ERROR : Failed to initialize ADS.");
    while (1);
  }
  // Set up Temperature Sensors - Start up the OneWire library on all defined bus-wires
  DeviceAddress deviceAddress;
  for (int i = 0; i < oneWireCount; i++) {;
    sensor[i].setOneWire(&ds18x20[i]);
    sensor[i].begin();
    if (sensor[i].getAddress(deviceAddress, 0)) sensor[i].setResolution(deviceAddress, 12);
  }
  Serial.println("SUCCESS : Setup complete");
  jsonMsg_to_serial("SUCCESS", "setup void complete success");
}

/*  ==================================
 *  =========== MAIN LOOP ============
 *  ==================================
 */
// TODO : create a request/reply architecture

void loop(void)
{

  if (Serial.available() > 0) {
    String request = Serial.readStringUntil('\n');
    // send serial response on rcv'd request
    jsonMsg_to_serial("REQUEST_RCVD", request);

    // determine request type
    int request_code = -1; // TODO add logic to convert rcv'd request to code
    if (request == "data") {
      request_code = 0;
    }
    if (request == "reboot"){
      request_code = -99;
    }
    // initialize variables
    float currentRms;
    float powerAvg;
    float tempsF[oneWireCount]; // initialize array
    JsonArray array;
    
    switch (request_code) {
      case 0: // request == "data"
        // Read CT - measure current and calculate power
        currentRms = getCurrent();
        powerAvg = VOLTAGE * currentRms;      
        // Read Temperature sensors
        getTemperatures(tempsF); // get temperature from each sensor on oneWire buses

        // clear JSON contents for new data
        doc.clear();
        // recreate array in JSON doc
        array = doc.to<JsonArray>();
        // add sensor dicts to JSON array
        add_ctSensor_to_json(array, "ct0", currentRms, VOLTAGE, powerAvg);
        add_tempSensors_to_json(array, temp_names, tempsF);
      
        // Generate the minified JSON, send it to the Serial port, clear payload
        sendJsonSerial(); // TODO: add confirmation reply?
        break;
        
      case -99: // request == "reboot" --> reset arduino
        // json response
        jsonMsg_to_serial("INFO", "reboot");
        // reboot arduino
        reboot();
        break;
        
      default:
        // do nothing
        jsonMsg_to_serial("ERROR", "invalid request");
        break;
    }
    
  }
  // do nothing
}

/*  ==================================
 *  =========== FUNCTIONS ============
 *  ==================================
 */

void(* resetFunc) (void) = 0;

void printMeasure(String prefix, float value, String postfix)
{
  Serial.print(prefix);
  Serial.print(value, 3);
  Serial.println(postfix);
}

void sendJsonSerial(){
  // Generate the minified JSON and send it to the Serial port.
  serializeJson(doc, Serial);
  // Start a new line
  Serial.println();
  // clear JSON payload from memory
  doc.clear();
}

void reboot() {
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1) {}
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

// Adds string message information to JSON object, primarily error reporting
void jsonMsg_to_serial(String _name, String _msg)
{
  // clear any contents in JSON object
  doc.clear();
  // define JSON object
  JsonObject obj = doc.createNestedObject();
  obj["name"] = _name;
  obj["msg"] = _msg;
  
  // send JSON message over serial
  sendJsonSerial();
}

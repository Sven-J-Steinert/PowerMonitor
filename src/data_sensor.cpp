#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "secrets.h"


const char* server = "192.168.0.4";
uint16_t serverPort = 65432;
uint32_t old_time;

const char* ssid = SECRET_SSID;
const char* password = SECRET_PASS;
const char* host = "PowerMonitor_ESP";

#include <ADS1115_WE.h> 
#include <RunningMedian.h>

#include<Wire.h>

int x = 100;

RunningMedian buffer_A0 = RunningMedian(x);

RunningMedian samples = RunningMedian(250);  // for calibration in setup
int ref_offset;

int A0_flat = 0;
int i = 0;
int U_sensorValue = 0;

// 16-bit ADC

#define I2C_ADDRESS 0x48

/* There are several ways to create your ADS1115_WE object:
 * ADS1115_WE adc = ADS1115_WE()             -> uses Wire / I2C Address = 0x48
 * ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS)  -> uses Wire / I2C_ADDRESS
 * ADS1115_WE adc = ADS1115_WE(&wire2)       -> uses the TwoWire object wire2 / I2C_ADDRESS
 * ADS1115_WE adc = ADS1115_WE(&wire2, I2C_ADDRESS) -> all together
 * Successfully tested with two I2C busses on an ESP32
 */
ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);


// server
WiFiClient client;

int readChannel(ADS1115_MUX channel) {
  int value = 0.0;
  adc.setCompareChannels(channel);
  value = adc.getRawResult(); // alternative: getResult_mV for Millivolt , getResult_V
  return value;
}

void TCP_send(float v1, float v2, float v3) {
  Serial.print("sending data to server");
  String json_string = "{\"Phase1\":" + String(v1,4) + ",\"Phase2\":" + String(v2,4) + ",\"Phase3\":" + String(v3,4) + "}";
  //String json_string = sprintf("{\"Phase1\": %d, \"Phase2\": %d, \"Phase3\": %d}",v1,v2,v3);
  
  int str_len = json_string.length() + 1; 
  
  // Prepare the character array (the buffer) 
  char char_array[str_len];
  
  // Copy it over 
  json_string.toCharArray(char_array, str_len);
  client.printf(char_array);
  client.flush();
  Serial.println(char_array);
  //Serial.println("closing connection");
  //client.stop();
}

void setup() {
  pinMode(A0, INPUT);
  Wire.begin();
  Serial.begin(115200);

  Serial.println("Booting");
  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid, password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
    Serial.println("Retrying connection...");
  }
  
  ArduinoOTA.setHostname(host);
  ArduinoOTA.onStart([]() { 
    //during upgrade  
  });

  ArduinoOTA.onEnd([]() { 
    // after upgrade
  });

  ArduinoOTA.onError([](ota_error_t error) {
    (void)error;
    ESP.restart();
  });

  /* setup the OTA server */
  ArduinoOTA.begin();
  //Serial.println("Ready");



  // setup 16-bit ADC
  
  
  if(!adc.init()){
    Serial.println("ADS1115 not connected!");
  }

  /* Set the voltage range of the ADC to adjust the gain
   * Please note that you must not apply more than VDD + 0.3V to the input pins!
   * 
   * ADS1115_RANGE_6144  ->  +/- 6144 mV
   * ADS1115_RANGE_4096  ->  +/- 4096 mV
   * ADS1115_RANGE_2048  ->  +/- 2048 mV (default)
   * ADS1115_RANGE_1024  ->  +/- 1024 mV
   * ADS1115_RANGE_0512  ->  +/- 512 mV
   * ADS1115_RANGE_0256  ->  +/- 256 mV
   */
  adc.setVoltageRange_mV(ADS1115_RANGE_6144); //comment line/change parameter to change range

  /* Set the inputs to be compared
   *  
   *  ADS1115_COMP_0_1    ->  compares 0 with 1 (default)
   *  ADS1115_COMP_0_3    ->  compares 0 with 3
   *  ADS1115_COMP_1_3    ->  compares 1 with 3
   *  ADS1115_COMP_2_3    ->  compares 2 with 3
   *  ADS1115_COMP_0_GND  ->  compares 0 with GND
   *  ADS1115_COMP_1_GND  ->  compares 1 with GND
   *  ADS1115_COMP_2_GND  ->  compares 2 with GND
   *  ADS1115_COMP_3_GND  ->  compares 3 with GND
   */
  adc.setCompareChannels(ADS1115_COMP_0_GND); //comment line/change parameter to change channel

  /* Set number of conversions after which the alert pin asserts
   * - or you can disable the alert 
   *  
   *  ADS1115_ASSERT_AFTER_1  -> after 1 conversion
   *  ADS1115_ASSERT_AFTER_2  -> after 2 conversions
   *  ADS1115_ASSERT_AFTER_4  -> after 4 conversions
   *  ADS1115_DISABLE_ALERT   -> disable comparator / alert pin (default) 
   */
  //adc.setAlertPinMode(ADS1115_ASSERT_AFTER_1); //uncomment if you want to change the default

  /* Set the conversion rate in SPS (samples per second)
   * Options should be self-explaining: 
   * 
   *  ADS1115_8_SPS 
   *  ADS1115_16_SPS  
   *  ADS1115_32_SPS 
   *  ADS1115_64_SPS  
   *  ADS1115_128_SPS (default)
   *  ADS1115_250_SPS 
   *  ADS1115_475_SPS 
   *  ADS1115_860_SPS 
   */
  adc.setConvRate(ADS1115_860_SPS); //uncomment if you want to change the default

  /* Set continuous or single shot mode:
   * 
   *  ADS1115_CONTINUOUS  ->  continuous mode
   *  ADS1115_SINGLE     ->  single shot mode (default)
   */
  adc.setMeasureMode(ADS1115_CONTINUOUS); //comment line/change parameter to change mode

   /* Choose maximum limit or maximum and minimum alert limit (window) in Volt - alert pin will 
   *  assert when measured values are beyond the maximum limit or outside the window 
   *  Upper limit first: setAlertLimit_V(MODE, maximum, minimum)
   *  In max limit mode the minimum value is the limit where the alert pin assertion will be  
   *  cleared (if not latched)  
   * 
   *  ADS1115_MAX_LIMIT
   *  ADS1115_WINDOW
   * 
   */
  //adc.setAlertModeAndLimit_V(ADS1115_MAX_LIMIT, 3.0, 1.5); //uncomment if you want to change the default
  
  /* Enable or disable latch. If latch is enabled the alert pin will assert until the
   * conversion register is read (getResult functions). If disabled the alert pin assertion will be
   * cleared with next value within limits. 
   *  
   *  ADS1115_LATCH_DISABLED (default)
   *  ADS1115_LATCH_ENABLED
   */
  //adc.setAlertLatch(ADS1115_LATCH_ENABLED); //uncomment if you want to change the default

  /* Sets the alert pin polarity if active:
   *  
   * ADS1115_ACT_LOW  ->  active low (default)   
   * ADS1115_ACT_HIGH ->  active high
   */
  //adc.setAlertPol(ADS1115_ACT_LOW); //uncomment if you want to change the default
 
  /* With this function the alert pin will assert, when a conversion is ready.
   * In order to deactivate, use the setAlertLimit_V function  
   */
  //adc.setAlertPinToConversionReady(); //uncomment if you want to change the default

  //Serial.println("ADS1115 Example Sketch - Continuous Mode");
  //Serial.println("All values in volts");
  //Serial.println();


  long ref_sum = 0;
    for (int n=0; n <= 1000; n++){
      samples.add(readChannel(ADS1115_COMP_3_GND));
    }
  ref_offset = samples.getMedian();


  // connect to server
  Serial.printf("connecting to %s %d \n", server, serverPort);
  if (!client.connect(server, serverPort)) {
    Serial.println("connection failed");
    delay(5000);
    return;
  }
}

void loop() {
  yield();
  ArduinoOTA.handle();

  U_sensorValue = analogRead(A0);
  int sensorData = readChannel(ADS1115_COMP_0_GND);
  
  Serial.print(U_sensorValue);
  Serial.print(" ");
  Serial.print(sensorData);
  Serial.println(" ");
  //int A0_avg = buffer_A0.reading(sensorData);
  
  int ref = readChannel(ADS1115_COMP_3_GND) - ref_offset;
  //int ref_avg = buffer_ref.reading(ref);

  /*
  buffer_A0.add(int(sensorData - ref));
  i++ ;
  if (i == x){
    A0_flat = buffer_A0.getMedian();
    i=0;
  }
  */


  //Serial.print(sensorData);
  //Serial.print(" ");
  
  //Serial.print(ref);
  //Serial.print(" ");
  
  //Serial.print(A0_avg);
  //Serial.print(" ");

  //Serial.print(ref);
  //Serial.print(" ");

  //Serial.print(int(sensorData - ref));
  //Serial.print(" ");

  // PRINT HERE
  //Serial.println(A0_flat);

  //Serial.print(sensorMovingAvg);
  //Serial.print(" ");
  
  
  //Serial.println(send_data);
  

  //Serial.print(" A1 ");
  //voltage = readChannel(ADS1115_COMP_1_GND);
  //Serial.print(voltage);
  
  //Serial.print(" A2 ");
  //voltage = readChannel(ADS1115_COMP_2_GND);
  //Serial.println(voltage);


  //delay(200);

  /*
  if (millis() - old_time >= 1000) {
    old_time=millis();
    TCP_send(float(U_sensorValue),float(A0_flat/10000),float(0));
  }
  */
}






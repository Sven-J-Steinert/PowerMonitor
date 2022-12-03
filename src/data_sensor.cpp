#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ADS1115_WE.h> 
#include <RunningMedian.h>
#include <Wire.h>
#include "secrets.h"

const char* server = "192.168.0.4";
uint16_t serverPort = 65432;
uint32_t old_time;

const char* ssid = SECRET_SSID;
const char* password = SECRET_PASS;
const char* host = "PowerMonitor_ESP";

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

WiFiClient client;

int A0_buffer = 0;
int A0_value = 0;
int ADS_value = 0;

int convert_Voltage(int value)
{
  // input range
  int min_value = 200;
  int max_value = 800;
  
  // output range [mv]
  int min_out = -325 * 1000;
  int max_out = 325 * 1000;

  int mV = map(value,min_value,max_value,min_out,max_out);
  return mV;
}

int convert_Current(int value)
{
  // input range
  int min_value = 0; // sadly only using half of the int16 range
  int max_value = 32767;
  
  // output range [mA]
  int min_out =-12768;
  int max_out = 12768;

  // depends on:

  // 1.5 mA / bit
  // ADS1115_RANGE_6144  ->  +/- 6144 mV

  //  29152 mA = 6144 mV
  //      20 A = 5000 mV
  //       0 A = 2500 mV
  //     -20 A =    0 mV
  // -29152 mA =-1144 mV

  // ADS1115_RANGE_4096  ->  +/- 4096 mV

  //  12768 mA = 4096 mV
  //       0 A = 2500 mV
  // -12768 mA =  904 mV

  int mA = map(value,min_value,max_value,min_out,max_out);
  return mA;
}

void initTimer();
void initInterrupt();

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
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Booting");

  pinMode(A0, INPUT);
  Wire.begin();

  Serial.print("Connecting to WiFi... ");
  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid, password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
    Serial.println("Retrying connection...");
  }
  Serial.println("connected.");
  

  Serial.print("Starting OTA... ");
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
  Serial.println("done.");


  Serial.print("Starting ADC setup...");

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
  adc.setVoltageRange_mV(ADS1115_RANGE_4096); //comment line/change parameter to change range

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
  adc.setAlertPinMode(ADS1115_ASSERT_AFTER_1); //uncomment if you want to change the default

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
  adc.setAlertLatch(ADS1115_LATCH_ENABLED); //uncomment if you want to change the default

  /* Sets the alert pin polarity if active:
   *  
   * ADS1115_ACT_LOW  ->  active low (default)   
   * ADS1115_ACT_HIGH ->  active high
   */
  adc.setAlertPol(ADS1115_ACT_LOW); //uncomment if you want to change the default
 
  /* With this function the alert pin will assert, when a conversion is ready.
   * In order to deactivate, use the setAlertLimit_V function  
   */
  adc.setAlertPinToConversionReady(); //uncomment if you want to change the default

  //Serial.println("ADS1115 Example Sketch - Continuous Mode");
  //Serial.println("All values in volts");
  //Serial.println();

  Serial.println(" done.");

  Serial.print("Initializing Timer...");
  //initTimer();
  initInterrupt(); // triggers when ADS sets Alert Pin LOW
  Serial.println("done.");

  // connect to server
  Serial.printf("connecting to %s %d \n", server, serverPort);
  if (!client.connect(server, serverPort)) {
    Serial.println("connection failed");
    delay(5000);
    Serial.println("Starting Loop.");
    return;
  }
  Serial.println("Starting Loop.");
}

void loop() {
  yield();
  ArduinoOTA.handle();

}

void IRAM_ATTR timerCall()
{
  Serial.println(analogRead(A0));
}

void initTimer()
{
  timer1_isr_init();
  timer1_attachInterrupt(timerCall);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP); // 5 ticks/us
  timer1_write(5814); // 5814 / 5 ticks per us from TIM_DIV16 == 1162.8 us interval -> 860 SPS
}

void IRAM_ATTR interruptCall()
{
  A0_buffer = analogRead(A0);
  ADS_value = adc.getRawResult();
  
  Serial.print(convert_Current(ADS_value));
  Serial.print(" ");
  Serial.print(convert_Voltage(A0_value));
  Serial.println(" ");
  A0_value = A0_buffer; // delay value 1 step to sync
}

void initInterrupt()
{
  attachInterrupt(digitalPinToInterrupt(D5), interruptCall, FALLING);
}

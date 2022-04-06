#include <TCA9555.h>
#include <Adafruit_LEDBackpack.h>
#include <Adafruit_GrayOLED.h>
#include <gfxfont.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <Wire.h>
#include "driver/i2s.h"
#include "freertos/queue.h"
#include "driver/adc.h"
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "SD.h"  
#include "Audio.h"  // https://github.com/schreibfaul1/ESP32-audioI2S
#include "FS.h"
#include "SPI.h"



Audio audio(true, I2S_DAC_CHANNEL_BOTH_EN);


//i2s configuration 
const i2s_port_t i2s_num = I2S_NUM_0; // i2s port number
i2s_config_t i2s_config = {
     .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
     .sample_rate = 44100,
     .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, /* the DAC module will only take the 8bits from MSB */
     .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
     .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_I2S_MSB,
     .intr_alloc_flags = 0, // default interrupt priority
     .dma_buf_count = 8, // orignally 8
     .dma_buf_len = 64,  // originally 64
     .use_apll = 0
    };
//



//Inputs
#define lightning_selected ADC1_CHANNEL_0  // SENSOR_VP
#define voltage_in ADC1_CHANNEL_5  //  GPIO33
#define ball_in_tube_selected ADC1_CHANNEL_4  // GPIO32
#define plane_selected ADC1_CHANNEL_6  // GPIO34
#define lighthouse_selected ADC1_CHANNEL_7  // GPIO35
#define train_selected ADC1_CHANNEL_3  // SENSOR_VN

//lightning MUX Communication
#define SDA 27 // ADC2_CHANNEL_7  // GPIO27
#define SCL 14 //ADC2_CHANNEL_6  // GPIO14

// SD Card inputs
#define SPI_MOSI 23
#define SPI_MISO 19 //VSPIQ 
#define SPI_SCK 18 //VSPICLK
#define SD_CS 5 //VSPICS0

// Outputs
const int Lighthouse_light = 12;  // GPIO12
//#define fan_starter_SSR ADC2_CHANNEL_4 // GPIO13
#define Audio_out DAC_CHANNEL_1 // GPIO25 DAC_1
const int fan_starter_SSR = 13;


//variables
double output_V;
double whistle_V;

double lightning_V;
int lightning_raw;
bool lightning = false;

double train_V;
int train_raw;
bool train = false;

double plane_V;
int plane_raw;
bool plane = false;

double lighthouse_V;
int lighthouse_raw;
bool lighthouse = false;

double ball_V;
int ball_raw;
bool ball = false;
int check_time = 250;  // time between checks in ms.
double t; // time
int t_reset = 1;
int switch_time = 120;
int delayT = 0;
int ave_num = 0;
double ave_v = 0;
double disp_V = 0;
double Fan_start_V = 0;
double output_V_Raw;
unsigned long lightningT;
int num_bars = 0;
long updateT = millis();
int fan_start = 0; // initializing state of fan

bool soundPlay = false;  // can we play the sound or not?
int soundNum;  // sound selection
bool First = true;  // First loop for initializing a connect to FS before calling audio.loop();
int currentS; // current sound Selection

bool up = true;
int lighthouse_val = 0;

bool setup_lightning = true;
bool setup_train = true;
bool setup_plane = true;
bool setup_lighthouse = true;
bool sound_select = true;

unsigned long switch_soundsT = millis();
unsigned long switchT = 5000;

unsigned long fan_timer = 0;
int output_val = 125;

 // constants
 const double Vmin_lightning = 10; // minimum voltage for lightning sequence to start
 const double Vmin = 5.5;  // minimum voltage that can be boosted by the buck boost board.
 const double Vmax = 16;  // Not a real maximum, just set here for testing.
 int period = 15;  // soft enough to make sure the ball stays on top of the tube, but fast enough to not wait forever.

// Define task handles
TaskHandle_t ball_H = NULL;
TaskHandle_t LHL = NULL;
TaskHandle_t LTL = NULL;
TaskHandle_t volt = NULL;
TaskHandle_t sound = NULL;
TaskHandle_t SW = NULL;

// define the tasks
void sound_choice_task( void *pvParameters );
void lightning_light_task( void *pvParameters );
void ball_task( void *pvParameters );
void lighthouse_light_task( void *pvParameters );
void voltage_task( void *pvParameters );
void switch_sound_task( void *pvParameters );

// defining the bargraph
Adafruit_24bargraph bar = Adafruit_24bargraph();

// defining the lightning mux interface
TCA9555 TCA(0x20);

void SDCardInit()
{        
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    pinMode(SD_CS, OUTPUT); 
    digitalWrite(SD_CS, HIGH); // SD card chips select, must use GPIO 5 (ESP32 SS)
    while(!SD.begin(SD_CS))
    {
        Serial.println("Error talking to SD card!");
    }
}



void setup() {
Serial.begin(115200);
Serial.println("starting ESP32");
delay(1000);

WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable   detector

// starting the mux controls
Wire.begin(SDA, SCL);
TCA.begin();
Wire.setClock(400000);
TCA.pinMode16(OUTPUT);
TCA.write16(0x0000);

// Starting the bargraph
bar.begin(0x70);

// setting up the fan soft starter
ledcSetup(0, 5000, 8);
// attach the channel to the GPIO to be controlled.
ledcAttachPin(fan_starter_SSR , 0);

// setting up the lighthouse light LED
ledcSetup(1, 5000, 8);
//attach the channel to the GPIO to be controlled
ledcAttachPin(Lighthouse_light, 1);

// Starting up the SD Card
SDCardInit();

audio.setVolume(17); // 0...21
if(audio.setFileLoop(true)){
  Serial.println("File set to loop");
}else{
  Serial.println("File not set to loop");
}


// Input pinModes
//pinMode(train_selected, INPUT);
adc1_config_channel_atten(train_selected, ADC_ATTEN_DB_11);
Serial.println("train");
adc1_config_channel_atten(plane_selected, ADC_ATTEN_DB_11);
Serial.println("plane selected");
adc1_config_channel_atten(lightning_selected, ADC_ATTEN_DB_11);
Serial.println("did this happen 2?");
adc1_config_channel_atten(voltage_in, ADC_ATTEN_DB_11);
Serial.println("voltage in setup");
adc1_config_channel_atten(ball_in_tube_selected, ADC_ATTEN_DB_11);
Serial.println("ball in tube setup");

adc1_config_channel_atten(lighthouse_selected, ADC_ATTEN_DB_11);
Serial.println("lighthouse");

fan_timer = millis();
Serial.println("Fan timer setup");

// Setting up tasks
Serial.println("Starting Get Ball Fan Task");
  xTaskCreatePinnedToCore(
    ball_task
    ,  "Ball_Fan_Task"
    ,  8192  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  &ball_H 
    ,  NULL);
vTaskSuspend( ball_H );

Serial.println(" Starting Lighthouse Light Task");
  xTaskCreatePinnedToCore(
    lighthouse_light_task
    ,  "Lighthouse_Light"
    ,  8192  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  &LHL 
    ,  NULL);
vTaskSuspend( LHL );

    Serial.println(" Starting Lightning Light Task");
  xTaskCreatePinnedToCore(
    lightning_light_task
    ,  "Lightning_Light_Task"
    ,  8192  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  &LTL 
    ,  NULL);
vTaskSuspend( LTL );

Serial.println("Starting Voltage Reading Task Task");  // this task always runs
  xTaskCreatePinnedToCore(
    voltage_task
    ,  "volt"
    ,  8192  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  &volt 
    ,  NULL);    

Serial.println("Starting Sound Choice Task");  // This task always runs
xTaskCreatePinnedToCore(
    sound_choice_task
    ,  "Sound_Choice"   // A name just for humans
    ,  8192  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &sound 
    ,  NULL);

}

void loop() {
  if (millis()-updateT > check_time){
    vTaskResume( sound );
    updateT = millis();

  if(soundNum == 1 && currentS != 1){
        Serial.println("switching to thunder");
        currentS = 1;
        audio.connecttoFS(SD, "/thunder.wav");
        
   }
   else if(soundNum == 2 && currentS != 2){
        Serial.println("switching to train");
        currentS = 2;
        audio.connecttoFS(SD, "/train3.wav");
        
   }
   else if(soundNum == 3 && currentS != 3){
        Serial.println("switching to plane");
        currentS = 3;
        audio.connecttoFS(SD, "/plane.wav");
        
   }
   else if(soundNum == 4 && currentS != 4){
      Serial.println("switching to lighthouse");
      currentS = 4;
      audio.connecttoFS(SD, "/light.wav");
   }

    



    
  }
  if (First == true){
      First = false;
      audio.connecttoFS(SD, "/thunder.wav");
    }
  
  if(soundPlay == true){
    audio.loop();
  }

}


// converts the sampled voltage to a LED light bar display
void voltage_task( void *pvParameters ){
  (void) pvParameters;
    for(;;)
    {
       const TickType_t xDelay2 = 250 / portTICK_PERIOD_MS;
        vTaskDelay( xDelay2 );
        
        //Reading Generator Voltage
        for (int i = 0; i < 20; i++){
          output_V_Raw = adc1_get_raw(voltage_in);
          output_V = output_V + output_V_Raw*0.02014;  //  value divided by 4096 multiplied by 3.3V and multiplied by a factor of 25 because of the voltage divider R1 = 49.9k, R2 = 2.1k
        }
        output_V = output_V / 20;
        //Serial.print("Voltage: ");
        //Serial.println(output_V);
        // determine the number of bars
        num_bars = round(output_V * 0.32);
        //Serial.print("num bars:  ");
        //Serial.println(num_bars);
        
  
        // write the number of bars
        for (int k = 0; k < 25; k++){
          //Serial.print("k: ");Serial.println(k);
          if (24 - k >= 24-num_bars){
            bar.setBar(24 -k, LED_RED);  // change second parameter to LED_RED for red, LED_YELLOW for yellow, or LED_GREEN for green LEDs.
          }else{
            bar.setBar(24-k,LED_OFF);
          }
        }
        bar.writeDisplay();
        
      }
}


void sound_choice_task( void *pvParameters ){
  (void) pvParameters;
    for(;;)
    {
      // read all of the sound inputs
      lightning_raw = adc1_get_raw(lightning_selected);
      lightning_V = lightning_raw * 0.00325;  //  value divided by 4096 multiplied by 3.3V and multiplied by a factor of 4.03 because of the voltage divider R1 = 10k, R2 = 3.3k
     // Serial.print("Lightning Voltage: ");
     // Serial.println(lightning_V);
      if (lightning_V >= 8.0){
        lightning = true;
      }else{
        lightning = false;
      }

      train_raw = adc1_get_raw(train_selected);
      train_V = train_raw * 0.00325;  //  value divided by 4096 multiplied by 3.3V and multiplied by a factor of 4.03 because of the voltage divider R1 = 10k, R2 = 3.3k
     // Serial.print("Train Voltage: ");
      //Serial.println(train_V);
      if (train_V >= 8.0){
         train = true;
      }else{
        train = false;
      }
      
      plane_raw = adc1_get_raw(plane_selected);
      plane_V = plane_raw * 0.00325;  //  value divided by 4096 multiplied by 3.3V and multiplied by a factor of 4.03 because of the voltage divider R1 = 10k, R2 = 3.3k
    //  Serial.print("Plane Voltage: ");
      //Serial.println(plane_V);
      if (plane_V >= 8.0){
        plane = true;
      }else{
        plane = false;
      }

      lighthouse_raw = adc1_get_raw(lighthouse_selected);
      lighthouse_V = lighthouse_raw * 0.00325;  //  value divided by 4096 multiplied by 3.3V and multiplied by a factor of 4.03 because of the voltage divider R1 = 10k, R2 = 3.3k
   //   Serial.print("lighthouse Voltage: ");
   //   Serial.println(lighthouse_V);
      if (lighthouse_V >= 8.0){
        lighthouse = true;
      }else{
        lighthouse = false;
      }
      

      // ballfan
      ball_raw = adc1_get_raw(ball_in_tube_selected);
      ball_V = ball_raw * 0.00325;  //  value divided by 4096 multiplied by 3.3V and multiplied by a factor of 4.03 because of the voltage divider R1 = 10k, R2 = 3.3k
      //Serial.print("Ball Voltage: ");
      //Serial.println(ball_V);
      if (ball_V >= 8.0){
        if (ball == false){
            output_val = 125;  // set the start offset.
          }
        ball = true;
        // resume ball fan task
        vTaskResume( ball_H );
      }else{
        ball = false;
        // suspend ball fan task
        vTaskSuspend( ball_H );
        ledcWrite(fan_starter_SSR, 0);
      }
      
     
      if (lighthouse == true){
        // enable lighthouse light
        vTaskResume( LHL );
        if (train == false && plane == false && lightning == false){
           // set all the setups to true for the other tasks
          setup_lightning = true;
          setup_train = true;
          setup_plane = true;
          // disable the other tasks
          Serial.println("Enabling lighthouse sound");
          //vTaskResume( LHS );
          soundNum = 4;
          soundPlay = true;
        }
      }else{
        vTaskSuspend(LHL); 
      }
      if(lightning == true){
        // enable lightning lights
        vTaskResume( LTL );
      }else{
        // disable lightning lights
        vTaskSuspend( LTL );
        TCA.write16(0x0000);
      }
        
        
      if (lightning == true && lighthouse == false && train == false && plane == false){
        // set all the setups to true for the other tasks
       soundNum = 1;  // 1 is for thunder.
       soundPlay = true;
        
      
      } else if(lighthouse == false && train == false && plane == true && lightning == false){
          soundNum = 3;  // 3 is for plane.
          soundPlay = true;
        
      } else if(lighthouse == false && train == true && plane == false && lightning == false){
          soundNum = 2;  // 2 is for train.
          soundPlay = true;
       
      }else if (lighthouse == true && train == false && plane == false && lightning == false){
      // do nothing covered before...
      }else{
        soundPlay = false;
      }
      
      vTaskSuspend( NULL );
    }
}

// Turns on the SSR driver to ramp up the fan.
void ball_task( void *pvParameters ){
  (void) pvParameters;
    for(;;)
    {
        const TickType_t xDelay = 50 / portTICK_PERIOD_MS;
        vTaskDelay( xDelay );
        if (output_val < 255){
          output_val ++;
          //Serial.print("output Value: ");
          //Serial.println(output_val);
          ledcWrite(0 , output_val);
        }
      
    }
}

// lights the lighthouse
void lighthouse_light_task( void *pvParameters ){
  (void) pvParameters;
    for(;;)
    {
      // Program to simulate the lighthouse light.
      const TickType_t xDelay1 = 200 / portTICK_PERIOD_MS;
        vTaskDelay( xDelay1 );
        if(up == true){
          lighthouse_val ++;
        }else{
          lighthouse_val --;
        }
        if (lighthouse_val == 255){
          up = false;
        }else if(lighthouse_val == 0){
          up = true;
        }
        //Serial.print("output Value: ");
        //Serial.println(lighthouse_val);
        ledcWrite(1 , lighthouse_val);
        
    }
}


// Plays a lightning - flashes the LEDs in series
void lightning_light_task( void *pvParameters ){
  (void) pvParameters;
    for(;;)
    {
      const TickType_t xDelay3 = switch_time / portTICK_PERIOD_MS;
      Serial.print("lightning is detected");
      //First run through- LED pin 1
      TCA.digitalWrite(15,HIGH);
        //Serial.println("lightning is happening LED 1");
      vTaskDelay( xDelay3 );
      // Second Run Through- first run through is automatic.
      vTaskDelay( xDelay3 );
      TCA.digitalWrite(14,HIGH);
      TCA.digitalWrite(15,LOW);
        //Serial.println("lightning is happening LED 2");
      vTaskDelay( xDelay3 );
        TCA.digitalWrite(13,HIGH);
        TCA.digitalWrite(14,LOW);
        //Serial.println("lightning is happening LED 3");
      
      vTaskDelay( xDelay3 );
        TCA.digitalWrite(12,HIGH);
        TCA.digitalWrite(13,LOW);
        //Serial.println("lightning is happening LED 4");
      
      vTaskDelay( xDelay3 );
        TCA.digitalWrite(11,HIGH);
        TCA.digitalWrite(12,LOW);
        //Serial.println("lightning is happening LED 5");
      
      vTaskDelay( xDelay3 );
        TCA.digitalWrite(10,HIGH);
        TCA.digitalWrite(11,LOW);
        //Serial.println("lightning is happening LED 6");
      
      vTaskDelay( xDelay3 );
        TCA.digitalWrite(9,HIGH);
        TCA.digitalWrite(10,LOW);
        //Serial.println("lightning is happening LED 7");
      
      vTaskDelay( xDelay3 );
        TCA.digitalWrite(8,HIGH);
        TCA.digitalWrite(9,LOW);
        //Serial.println("lightning is happening LED 8");
      
      vTaskDelay( xDelay3 );
        TCA.digitalWrite(7,HIGH);
        TCA.digitalWrite(8,LOW);
        Serial.println("lightning is happening LED 9");
      
      vTaskDelay( xDelay3 );
        TCA.digitalWrite(6,HIGH);
        TCA.digitalWrite(7,LOW);
        //Serial.println("lightning is happening LED 10");
      
      vTaskDelay( xDelay3 );
        TCA.digitalWrite(5,HIGH);
        TCA.digitalWrite(6,LOW);
        //Serial.println("lightning is happening LED 11");
      
      vTaskDelay( xDelay3 );
        TCA.digitalWrite(4,HIGH);
        TCA.digitalWrite(5,LOW);
        //Serial.println("lightning is happening LED 12");
      
      vTaskDelay( xDelay3 );
        TCA.digitalWrite(3,HIGH);
        TCA.digitalWrite(4,LOW);
        Serial.println("lightning is happening LED 13");
      
      vTaskDelay( xDelay3 );
        TCA.digitalWrite(2,HIGH);
        TCA.digitalWrite(3,LOW);
        Serial.println("lightning is happening LED 14");
      
      vTaskDelay( xDelay3 );
        TCA.digitalWrite(1,HIGH);
        TCA.digitalWrite(2,LOW);
        //Serial.println("lightning is happening LED 15");
      
      vTaskDelay( xDelay3 );
        TCA.digitalWrite(0,HIGH);
        TCA.digitalWrite(1,LOW);
        //Serial.println("lightning is happening LED 16");
      
      vTaskDelay( xDelay3 );
        TCA.digitalWrite(0,LOW);
    
  }
}

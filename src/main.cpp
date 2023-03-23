#include <Arduino.h>
#include <U8g2lib.h>
#include <cmath>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>
#include "ADSR.h"
#include "FILTER.h"
#include "ECHO.h"
// //#include "bsp_dma.h"
//include "POLYPHONY.h"
// //#include <ES_CAN.h>
// 
//testing
//#define DISABLE_THREADS
//#define TEST_SCANKEYS
//#define TEST_DISPLAY
//#define TEST_MODESELECTION
//#define TEST_CANTXTHREAD
//#define TEST_CANRXTHREAD
//#define TEST_SAMPLEISR
//#define TEST_CANTX
//#define TEST_CANRX



uint8_t  OCTAVE_NUM = 4; 
const uint32_t sampleRate = 22000; 


// // define the different modes
#define NORMAL_MODE       0
#define ADSR_MODE         1
#define FILTER_MODE       2
#define ECHO_MODE         3
#define POLYPHONY_MODE    4

#define MODE_MAX          4

// define the variables for mode selecting
int32_t Joystick_Y;
int8_t Actual_mode = 0;
int8_t mode_display = 0;
// The global mutex for mode selection
SemaphoreHandle_t ModeselectionMutex;

//define the task that will be used: 
//#define CAN_TX
//#define CAN_RX
//#define DISPLAY_TASK 
//#define scankeys

//Constants
  const uint32_t interval = 100; //Display update interval
  
//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;
  //generating sound 
  const uint32_t stepSizes [] = {68178701,	72231588,	76528508,	81077269,	85899345,	91006452,	96418111,	102151892,	108227319,	114661960,	121479245,	128702599};
  const uint32_t sin_StepSizes [] = {3,3,3,3,3,4,4,4,4,5,5,5};
  

  //advance feature
  FILTER filter1;
  Echo echo; 
  // POLYPHONY polyphony1;
  ADSR adsr1(0, 0.3, 0.1, 0.2, 0.5);



  volatile uint32_t currentStepSize;
  volatile uint8_t keyArray[7];
  volatile uint8_t current_counter; 
  volatile uint8_t sin_step;

  #define SINE_LUT_SIZE 256

const int32_t sine_wave_array[SINE_LUT_SIZE] =  {0,52701886,105372028,157978697,210490205,262874923,315101294,367137860,418953276,470516330,521795962,572761285,623381597,673626407,723465451,772868705,821806412,870249094,918167571,965532977,1012316783,1058490807,1104027236,1148898639,1193077990,1236538674,1279254514,1321199779,1362349203,1402677998,1442161873,1480777043,1518500249,1555308767,1591180424,1626093615,1660027307,1692961061,1724875039,1755750016,1785567395,1814309215,1841958163,1868497584,1893911493,1918184579,1941302224,1963250500,1984016187,2003586778,2021950482,2039096240,2055013722,2069693340,2083126253,2095304368,2106220350,2115867624,2124240379,2131333570,2137142926,2141664947,2144896908,2146836865,2147483647,2146836865,2144896908,2141664947,2137142926,2131333570,2124240379,2115867624,2106220350,2095304368,2083126253,2069693340,2055013722,2039096240,2021950482,2003586778,1984016187,1963250500,1941302224,1918184579,1893911493,1868497584,1841958163,1814309215,1785567395,1755750016,1724875039,1692961061,1660027307,1626093615,1591180424,1555308767,1518500249,1480777043,1442161873,1402677998,1362349203,1321199779,1279254514,1236538674,1193077990,1148898639,1104027236,1058490807,1012316783,965532977,918167571,870249094,821806412,772868705,723465451,673626407,623381597,572761285,521795962,470516330,418953276,367137860,315101294,262874923,210490205,157978697,105372028,52701886,0,-52701886,-105372028,-157978697,-210490205,-262874923,-315101294,-367137860,-418953276,-470516330,-521795962,-572761285,-623381597,-673626407,-723465451,-772868705,-821806412,-870249094,-918167571,-965532977,-1012316783,-1058490807,-1104027236,-1148898639,-1193077990,-1236538674,-1279254514,-1321199779,-1362349203,-1402677998,-1442161873,-1480777043,-1518500249,-1555308767,-1591180424,-1626093615,-1660027307,-1692961061,-1724875039,-1755750016,-1785567395,-1814309215,-1841958163,-1868497584,-1893911493,-1918184579,-1941302224,-1963250500,-1984016187,-2003586778,-2021950482,-2039096240,-2055013722,-2069693340,-2083126253,-2095304368,-2106220350,-2115867624,-2124240379,-2131333570,-2137142926,-2141664947,-2144896908,-2146836865,-2147483647,-2146836865,-2144896908,-2141664947,-2137142926,-2131333570,-2124240379,-2115867624,-2106220350,-2095304368,-2083126253,-2069693340,-2055013722,-2039096240,-2021950482,-2003586778,-1984016187,-1963250500,-1941302224,-1918184579,-1893911493,-1868497584,-1841958163,-1814309215,-1785567395,-1755750016,-1724875039,-1692961061,-1660027307,-1626093615,-1591180424,-1555308767,-1518500249,-1480777043,-1442161873,-1402677998,-1362349203,-1321199779,-1279254514,-1236538674,-1193077990,-1148898639,-1104027236,-1058490807,-1012316783,-965532977,-918167571,-870249094,-821806412,-772868705,-723465451,-673626407,-623381597,-572761285,-521795962,-470516330,-418953276,-367137860,-315101294,-262874923,-210490205,-157978697,-105372028,-52701886};

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//multithreading global variable 
SemaphoreHandle_t keyArrayMutex;
SemaphoreHandle_t RXMessageMutex; 
SemaphoreHandle_t KnobMutex;
SemaphoreHandle_t RXMutex;


// CAN bus protocol 
uint8_t Key_press[8] = {0}; 
QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;
SemaphoreHandle_t CAN_TX_Semaphore;
boolean west; 
boolean east;



// different waveform
uint8_t WAVEFORM = 0; 
#define SAWTOOTH 0
#define SIN 1
#define RECTANGLE 2
#define TRIG 3
//knob



class knob{
  public: 
  int32_t knobRotation; 
  knob(int32_t max_val,int32_t min_val ){
    m_max_val = max_val; 
    m_min_val = min_val; 
    
  }  
  
  void update_rotation(uint8_t current_state){
    
  
      //interprter knobs
      state  = state << 2; // shift the current as previous
      state   = state | current_state; // introduce the new current 
      state  = state & 0x0F; // make 4 most significant bit equal to 0
      switch(state) {
        case 2: 
        case 13: 
        m_knobRotation += 1; 
        m_lastRotation = 1;
          
        break; 
        
        case  7:
        case 8: 
       m_knobRotation -= 1 ; 
        m_lastRotation = -1; 
      
      
        break; 
        
        case 3:
        case 6: 
        case 12: 
       m_knobRotation +=  m_lastRotation;
       
        break; 
        
        default: 
        
        break; 
}
 if(m_knobRotation < m_min_val ){
   m_knobRotation = m_min_val; 
 }
else if(m_knobRotation > m_max_val){
m_knobRotation = m_max_val; 
  }

 __atomic_store_n(&knobRotation,m_knobRotation , __ATOMIC_RELAXED);
 }
  private:
    int32_t m_max_val; 
    int32_t m_min_val; 
    uint8_t state;
    int8_t  m_lastRotation; 
    int32_t m_knobRotation;

};
knob knob1(8,0); // filter
knob knob3(8 ,0); // volume 
knob knob2(1,0); // wave 
knob knob0(8,0); // high pass
//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}
void setRow(uint8_t rowIdx){

  digitalWrite(RA0_PIN,(rowIdx >> 0) & 0x01); 
  digitalWrite(RA1_PIN,(rowIdx >> 1) & 0x01); 
  digitalWrite(RA2_PIN,(rowIdx >> 2) & 0x01); 
   
}
uint8_t readCols(){
  
  uint8_t  col_val;
  
  col_val |= digitalRead(C0_PIN) << 0;
  col_val |= digitalRead(C1_PIN) << 1;
  col_val |= digitalRead(C2_PIN) << 2;
  col_val |= digitalRead(C3_PIN) << 3;  

  return col_val; 

}

void sampleISR() {
  static uint32_t phaseAcc = 0;
  
  phaseAcc += currentStepSize << (4-Key_press[1]);
  //phaseAcc += polyphony1.do_Polyphony();
  // phaseAcc += ((Actual_mode == POLYPHONY_MODE) ? polyphony1.do_Polyphony() : currentStepSize);
  //uint32_t currentStepSize_1 = currentStepSize;
  //int32_t Vout = (phaseAcc >> 24) - 128;
  //Vout = Vout >> (8 - knob_rotation);
  //Serial.println(currentStepSize);
  //int32_t Vout = adsr1.Do_ADSR(phaseAcc, polyphony1.do_Polyphony());
  int32_t Vout = ((Actual_mode == NORMAL_MODE) || (Actual_mode == POLYPHONY_MODE) || (Actual_mode == FILTER_MODE)) ? ((phaseAcc >> 24) - 128) : adsr1.Do_ADSR(phaseAcc, currentStepSize);
  //check = adsr1.Check(currentStepSize);
  if(Actual_mode == FILTER_MODE)
  {
    Vout = filter1.LowPassFilter(Vout, knob1.knobRotation);
    Vout = filter1.HighPassFilter(Vout, knob0.knobRotation);
  }
  //
  if(Actual_mode == ECHO_MODE){Vout = echo.do_Echo(Vout, currentStepSize);}
  //Vout = echo.do_Echo(Vout, polyphony1.do_Polyphony());
  Vout = Vout >> (8 - knob3.knobRotation);

  //
  if((Actual_mode == NORMAL_MODE) || (Actual_mode == POLYPHONY_MODE) || (Actual_mode == FILTER_MODE)){analogWrite(OUTR_PIN, Vout+128);}
  else{analogWrite(OUTR_PIN, Vout);}
  

  // static uint32_t phase_sin = 0; 
  // static uint8_t sin_i = 0; 
  // static uint32_t phaseAcc = 0;
  // if((east == 1 && west == 0)||(east == 0 && west == 0)){
  // int32_t Vout; 
  // int32_t sin_out; 

  // if(WAVEFORM == 0){
  //   phaseAcc += currentStepSize;
  //   Vout = (phaseAcc >> 24) - 128;
  //   Vout = filter1.LowPassFilter(Vout, knob1.knobRotation);
  //   Vout = filter1.HighPassFilter(Vout, knob0.knobRotation);

    
  // }
  // else if (WAVEFORM == 1){
  //   sin_i += sin_step;
  //   sin_out = sine_wave_array[sin_i];
  //   Vout = (sin_out >> 24); 
  // }
  
  // Vout = (Vout >> (8-knob3.knobRotation)); // modify the volume 
  // analogWrite(OUTR_PIN, Vout  + 128 );
  // }
}

#ifndef CAN_RX
void CAN_RX_ISR (void) {

	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);

}
#endif

#ifndef CAN_TX
void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}
#endif

#ifndef scankeys
void scanKeysTask(void * pvParameters) {
   
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    #ifndef TEST_SCANKEYS
      vTaskDelayUntil( &xLastWakeTime, xFrequency );
    #endif
    uint32_t localCurrentStepSize;
    uint8_t  localKnob3;
    int8_t  lastRotation;
    int8_t  localKnob3rotation;
    uint8_t l_prevkeystate[3];
    uint8_t localsinstep; 
    uint8_t localTxmes[8]  = {0}; 
    uint32_t ID;
    boolean outBits[7] = {0,0,0,1,1,1,1}; 
    boolean l_west; 
    boolean l_east;
    
    
    
    
    //reading input
    for(int i = 0; i < 7 ; i++){//depending on the number of row(not collumn) i need to be change  
      setRow(i);                     //Set row address
      digitalWrite(OUT_PIN,outBits[i]); //Set value to latch in DFF
      digitalWrite(REN_PIN,1);          //Enable selected row
      delayMicroseconds(3);             //Wait for column inputs to stabilise
      keyArray[i] = readCols();         //Read column inputs
      digitalWrite(REN_PIN,0);          //Disable selected row // reading the 4 collum of row i 
    }

      // interprter key 
      for(int i = 0; i < 3 ; i++){
      
        for(int j = 0; j < 4 ; j++){
          if(((keyArray[i] >> j) & 0x01) == 0){//checking if it a key is press which is equivalent to have a bit == 0 
          localCurrentStepSize = stepSizes[i*4+j]; // looking step size in the arra 
          if(((l_prevkeystate[i] >> j) & 0x01) == 1){
              localTxmes[0] = 'P';
                if( l_east  == 0){
                localTxmes[1] = 3;
              }
                            else {
                localTxmes[1] = 4;
              }
              localTxmes[2] = i*4+j;
              xSemaphoreTake(RXMessageMutex, portMAX_DELAY);
              Key_press[0] = localTxmes[0] ; 
              Key_press[1] = localTxmes[1] ; 
              Key_press[2] = localTxmes[2] ; 
              xSemaphoreGive(RXMessageMutex);
              }
            }
          else if(((l_prevkeystate[i] >> j) & 0x01) == 0) {
              localTxmes[0] = 'R';
              if( l_east  == 0){
                localTxmes[1] = 3;
              }
              else {
                localTxmes[1] = 4;
              }

              localTxmes[2] = i*4+j;
              xSemaphoreTake(RXMessageMutex, portMAX_DELAY);
                Key_press[0] = localTxmes[0] ; 
                Key_press[1] = localTxmes[1] ; 
                Key_press[2] = localTxmes[2] ; 
              xSemaphoreGive(RXMessageMutex);
             
              __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
          }
          
        }
        l_prevkeystate[i] =  keyArray[i];
      }

      // 
      l_west = ~(keyArray[5]>>3) & 0x01; 
      l_east = ~(keyArray[6]>>3) & 0x01; 
      
      
      // Serial.print(keyArray[5]);
      //Knob reading 
      uint8_t localKnob3_current = keyArray[3] >> 2;
      uint8_t localKnob2_current = keyArray[3];
      uint8_t localKnob1_current = keyArray[4] >> 2; 
      uint8_t localKnob0_current = keyArray[4];
      
      xSemaphoreTake(KnobMutex, portMAX_DELAY); 
      knob3.update_rotation(localKnob3_current & 0x03); 
      knob2.update_rotation(localKnob2_current & 0x03); 
      knob1.update_rotation(localKnob1_current & 0x03); 
      knob0.update_rotation(localKnob0_current & 0x03);
      west = l_west; 
      east = l_east; 
      WAVEFORM =  knob2.knobRotation;
      xSemaphoreGive(KnobMutex); 
      
      //OUT
      if(l_west == 1){
      xQueueSend( msgOutQ, localTxmes, portMAX_DELAY);
      localTxmes[0] = 'M'; // this is just a Random value different from P or R which is quite important 
      }
   

      // waveform function
      if(localCurrentStepSize != 0){
         __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
        localCurrentStepSize = 0;
      }
      
      
    #ifdef TEST_SCANKEYS
      break;
    #endif
      
      
  }

}
#endif

#ifndef DISPLAY_TASK
void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  static uint32_t count = 0;
  TickType_t xLastWakeTime = xTaskGetTickCount(); 
  while(1){
    #ifndef TEST_DISPLAY
      vTaskDelayUntil( &xLastWakeTime, xFrequency );
    #endif
    if((east == 1 && west == 0)||(east == 0 && west == 0)){
    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"state/octave/key");  // write something to the internal memory
    u8g2.setCursor(100,10);
   
   //key display 
   xSemaphoreTake(RXMessageMutex, portMAX_DELAY);
     u8g2.print((char) Key_press[0]);
     u8g2.print(Key_press[1]);
     u8g2.print(Key_press[2]);
   xSemaphoreGive(RXMessageMutex); 
 
  // wave and volume display
    xSemaphoreTake(KnobMutex, portMAX_DELAY); 
    //knobx
    u8g2.drawStr(2,20,"vol");
    u8g2.setCursor(30,20);
    u8g2.print(knob3.knobRotation,HEX);
   
    
    //can 
    u8g2.drawStr(2,30,"East");
    u8g2.setCursor(50,30);
    u8g2.print(east,HEX);
    u8g2.drawStr(60,30,"West");
    u8g2.setCursor(90,30);
    u8g2.print(west,HEX);
    xSemaphoreGive(KnobMutex);

     xSemaphoreTake(ModeselectionMutex,portMAX_DELAY);
    u8g2.drawStr(40,20,"mode");
    
    switch(mode_display){
      case NORMAL_MODE:
        u8g2.drawStr(80,20,"Normal");
      break; 
      case ADSR_MODE:
        u8g2.drawStr(80,20,"ADSR");
      break; 
      case ECHO_MODE:
        u8g2.drawStr(80,20,"ECHO");
      break; 
      case FILTER_MODE:
        u8g2.drawStr(80,20,"FILTER");
      break; 
      case POLYPHONY_MODE:
         u8g2.drawStr(80,20,"POLY");
      break; 
      default:u8g2.drawStr(80,20,"Normal"); 
     
  

    }
    xSemaphoreGive(ModeselectionMutex);




    u8g2.sendBuffer(); 

    //Toggle LED
     digitalToggle(LED_BUILTIN);
  

  }
  #ifdef TEST_DISPLAY
    break;
  #endif
  }
}
#endif

#ifndef CAN_RX
void decodeTask(void * pvParameters){
  uint32_t localCurrentStepSize ;
  uint8_t l_RXMessage[8] = {0};
  while(1){
    
    xQueueReceive(msgInQ, l_RXMessage, portMAX_DELAY);
    if(east == 1 && west == 0){
    if(l_RXMessage[0] == 'R'){
      localCurrentStepSize = 0;
       __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
       xSemaphoreTake(RXMessageMutex, portMAX_DELAY);
          Key_press[0] = l_RXMessage[0] ; 
          Key_press[1] = l_RXMessage[1] ; 
          Key_press[2] = l_RXMessage[2] ; 
       xSemaphoreGive(RXMessageMutex);
    }
    else if(l_RXMessage[0] == 'P'){
      localCurrentStepSize = stepSizes[l_RXMessage[2]];
      xSemaphoreTake(RXMessageMutex, portMAX_DELAY);
        Key_press[0] = l_RXMessage[0] ; 
        Key_press[1] = l_RXMessage[1] ; 
        Key_press[2] = l_RXMessage[2] ; 
      xSemaphoreGive(RXMessageMutex);
      // 
    }
    if(localCurrentStepSize != 0){
      __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
      localCurrentStepSize = 0;
    }
    }
    #ifdef TEST_CANRXTHREAD
      break;
    #endif


  }
}
#endif

#ifndef CAN_TX
void CAN_TX_Task (void * pvParameters) {
	uint8_t msgOut[8];
	while (1) {
	  
    xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);

		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
		CAN_TX(0x123, msgOut);
    
    #ifdef TEST_CANTXTHREAD
    xSemaphoreGive(CAN_TX_Semaphore);
      break;
    #endif
	}
}
#endif

void modeSelection(void *pvParameters)
{
  const TickType_t xFrequency = 250 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1){
    #ifndef TEST_MODESELECTION
      vTaskDelayUntil( &xLastWakeTime, xFrequency );
    #endif
    int8_t local_Actual_mode = Actual_mode;
    xSemaphoreTake(ModeselectionMutex, portMAX_DELAY);
    Joystick_Y = (int32_t)analogRead(JOYY_PIN);
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    uint8_t Joystick_click = (keyArray[5]&0x4)>>2;
    xSemaphoreGive(keyArrayMutex);
    if(Joystick_Y > 780)
    {
      mode_display += 1;
      if(mode_display > MODE_MAX){ mode_display = NORMAL_MODE; }
      if(Joystick_click == 1){ local_Actual_mode += 1; }
      if(local_Actual_mode > MODE_MAX){ local_Actual_mode = NORMAL_MODE; }
    }
    else if(Joystick_Y < 200)
    {
      mode_display -= 1;
      if(mode_display < 0 ){ mode_display = NORMAL_MODE; }
      if(Joystick_click == 1){ local_Actual_mode -= 1; }
      if(local_Actual_mode < 0 ){ local_Actual_mode = NORMAL_MODE; }
    }
    else{mode_display = mode_display; local_Actual_mode = local_Actual_mode;}
    xSemaphoreGive(ModeselectionMutex);
    __atomic_store_n(&Actual_mode, local_Actual_mode, __ATOMIC_RELAXED);

    #ifdef TEST_MODESELECTION
      break;
    #endif


  }
}

void setup() {
  // put your setup code here, to run once:

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");
  
  
  // interupt 
   #ifndef scankeys
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();
  #endif 
  //instatiate CAN
  msgInQ = xQueueCreate(36,8);
  
  msgOutQ = xQueueCreate(36,8);
 
  CAN_Init(false);
  setCANFilter(0x123,0x7ff);
  
  #ifndef CAN_RX
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  #endif 

  #ifndef CAN_TX
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  #endif

   

  CAN_Start();
  

  //multithreading
    //multithreading for scankey
KnobMutex = xSemaphoreCreateMutex();
 ModeselectionMutex = xSemaphoreCreateMutex();
keyArrayMutex = xSemaphoreCreateMutex();
RXMessageMutex = xSemaphoreCreateMutex();
CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

#ifndef DISABLE_THREADS
  TaskHandle_t scanKeysHandle = NULL;
  #ifndef scankeys  
  xTaskCreate(
  scanKeysTask,		/* Function that implements the task */
  "scanKeys",		/* Text name for the task */
  64,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  2,			/* Task priority */
  &scanKeysHandle );  /* Pointer to store the task handle */
  #endif 


    //multithreading for display task
  TaskHandle_t displayUpdateHandle = NULL;
  #ifndef DISPLAY_TASK
  xTaskCreate(
  displayUpdateTask,		/* Function that implements the task */
  "displayupdate",		/* Text name for the task */
  256,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  1,			/* Task priority */
  &displayUpdateHandle); /* Pointer to store the task handle */
  #endif 

  TaskHandle_t decodeDataHandle = NULL;
  #ifndef CAN_RX
  xTaskCreate(
    decodeTask, 
    "decodeRX", 
    256, 
    NULL, 
    3, //temporary 
    &decodeDataHandle);
  #endif

  TaskHandle_t CantxHandle = NULL;
  #ifndef CAN_TX
  xTaskCreate(
    CAN_TX_Task, 
    "CANTX", 
    256, 
    NULL, 
    3, //temporary 
    & CantxHandle );
  #endif 

  TaskHandle_t modeSelectionHandle = NULL;
    xTaskCreate(
        modeSelection,     /* Function that implements the task */
        "modeselection",       /* Text name for the task */
        20,               /* Stack size in words, not bytes */
        NULL,             /* Parameter passed into the task */
        1,                /* Task priority */
        &modeSelectionHandle);
#endif

#ifdef TEST_SCANKEYS

  uint32_t startTime = micros();
  for (int iter = 0; iter < 32; iter++) {
  scanKeysTask(NULL);
  }
  Serial.println(micros() - startTime);
  Serial.println((micros() - startTime)/32);
  while (1);
#endif

#ifdef TEST_DISPLAY

  uint32_t startTime = micros();
  for (int iter = 0; iter < 32; iter++) {
  displayUpdateTask(NULL);
  }
  Serial.println(micros() - startTime);
  Serial.println((micros() - startTime)/32);
  while (1);

#endif

#ifdef TEST_MODESELECTION

  uint32_t startTime = micros();
  for (int iter = 0; iter < 32; iter++) {
  modeSelection(NULL);
  }
  Serial.println(micros() - startTime);
  Serial.println((micros() - startTime)/32);
  while (1);

#endif

#ifdef TEST_SAMPLEISR

  uint32_t startTime = micros();
  for (int iter = 0; iter < 320; iter++) {
  sampleISR();
  }
  Serial.println(micros() - startTime);
  Serial.println((micros() - startTime)/320);
  while (1);

#endif

#ifdef TEST_CANTXTHREAD
  uint8_t msg[8]={'A',1,2,3,4,5,6,7};
  for(int iter=0;iter<384;iter++){
    xQueueSend(msgOutQ,msg,NULL);
  }
  uint32_t startTime=micros();
  
  for (int iter = 0; iter < 384; iter++) {
    CAN_TX_Task(NULL);
  }
  Serial.println(micros() - startTime);
  Serial.println((micros() - startTime)/384);
  while (1);

#endif

#ifdef TEST_CANRXTHREAD
  uint8_t msg[8]={'A',1,2,3,4,5,6,7};
  for(int iter=0;iter<384;iter++){
    xQueueSend(msgInQ,msg,NULL);
  }

   uint32_t startTime=micros();
  for (int iter = 0; iter < 384; iter++) {
    CAN_TX_Task(NULL);
  }
  Serial.println(micros() - startTime);
  Serial.println((micros() - startTime)/384);
  while (1);

#endif
#ifndef DISABLE_THREADS
  vTaskStartScheduler();
#endif



}


// void print_binary_V2(uint8_t decimal){
 
//  unsigned char* binary = reinterpret_cast<unsigned char*>(&decimal);
//  unsigned char mask = 0x80;

//   // Print the binary representation
//   Serial.print("col representation: ");
//   for (int i = 0; i < sizeof(decimal); i++) {
//     for (int j = 0; j < 8; j++) {
      
//       Serial.print((binary[i] & mask) ? 1 : 0);
//       mask >>= 1;
//     } 
//     mask = 0x80;
//   }
//   Serial.println();
// }
// // generate sin wave 
void loop() {}

    
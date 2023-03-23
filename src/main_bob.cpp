// #include <Arduino.h>
// #include <U8g2lib.h>
// #include <STM32FreeRTOS.h>
// #include "ADSR.h"
// #include "FILTER.h"
// #include "ECHO.h"
// //#include "bsp_dma.h"
// #include "POLYPHONY.h"
// //#include <ES_CAN.h>

// using namespace std;

// // define the different modes
// #define NORMAL_MODE       0
// #define ADSR_MODE         1
// #define FILTER_MODE        2
// #define ECHO_MODE         3
// #define POLYPHONY_MODE    4

// #define MODE_MAX          4

// // define the variables for mode selecting
// int32_t Joystick_Y;
// int8_t Actual_mode = 0;
// int8_t mode_display = 0;
// // The global mutex for mode selection
// SemaphoreHandle_t ModeselectionMutex;



// // Constants
// const uint32_t interval = 100; // Display update interval

// // Pin definitions
// // Row select and enable
// const int RA0_PIN = D3;
// const int RA1_PIN = D6;
// const int RA2_PIN = D12;
// const int REN_PIN = A5;

// // Matrix input and output
// const int C0_PIN = A2;
// const int C1_PIN = D9;
// const int C2_PIN = A6;
// const int C3_PIN = D1;
// const int OUT_PIN = D11;

// // Audio analogue out
// const int OUTL_PIN = A4;
// const int OUTR_PIN = A3;

// // Joystick analogue in
// const int JOYY_PIN = A0;
// const int JOYX_PIN = A1;

// // Output multiplexer bits
// const int DEN_BIT = 3;
// const int DRST_BIT = 4;
// const int HKOW_BIT = 5;
// const int HKOE_BIT = 6;
// // generating sound

// const uint32_t sampleRate = 22000;
// const double semitoneFactor = std::pow(2, 1.0 / 12);


// // CAN bus
// volatile uint8_t TX_Message[8] = {0};

// uint32_t phaseAcc_1 = 0;



// constexpr std::array<uint32_t, 12> calc_stepSizes()
// {
//   std::array<uint32_t, 12> stepSizes{};

//   double frequencies[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//   frequencies[9] = 440.0; // A4

//   for (int i = 10; i < 12; i++)
//   {
//     frequencies[i] = frequencies[i - 1] * semitoneFactor;
//   }
//   for (int i = 8; i >= 0; i--)
//   {
//     frequencies[i] = frequencies[i + 1] / semitoneFactor;
//   }
//   for (int i = 0; i < 12; i++)
//   {
//     double frequency = frequencies[i];
//     stepSizes[i] = static_cast<uint32_t>((1ULL << 32) * frequency / sampleRate);
//   }
//   return stepSizes;
// }

// const std::array<uint32_t, 12> stepSizes = calc_stepSizes();
// const std::array<const char *, 12> keyNote = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

// volatile uint32_t currentStepSize;
// volatile uint8_t keyArray[7];
// //volatile int8_t knob_rotation;

// volatile uint8_t check;

// SemaphoreHandle_t keyArrayMutex;

// // Display driver object
// U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

// class Knob {
//   private:
//     int8_t m_max_val; 
//     int8_t m_min_val; 
//     int8_t m_knobRotation;
//     uint8_t localKnob3_previous;
//     int rotation_variable;
    

//   public:
//     Knob(int8_t max_val, int8_t min_val) : m_knobRotation(knob_rotation), localKnob3_previous(0), rotation_variable(0) 
//     {
//       m_max_val = max_val; 
//       m_min_val = min_val; 
//     }

//     volatile int8_t knob_rotation;
  
//     void update(uint8_t localKnob3_current) {
//       uint8_t rotation = localKnob3_previous << 2 | localKnob3_current;
      
//       switch(rotation) {
//         case 0x1:
//           rotation_variable = 1;
//           break;
//         case 0x4:
//           rotation_variable = -1;
//           break;
//         case 0xB:
//           rotation_variable = -1;
//           break;
//         case 0xE:
//           rotation_variable = 1;
//           break;
//         default:
//           rotation_variable = 0;
//       }
      
//       m_knobRotation += rotation_variable;
      
//       if(m_knobRotation > m_max_val) {
//         m_knobRotation = m_max_val;
//       }
//       else if(m_knobRotation < m_min_val) {
//         m_knobRotation = m_min_val;
//       }
      
//       localKnob3_previous = localKnob3_current;
      
//       __atomic_store_n(&knob_rotation, m_knobRotation, __ATOMIC_RELAXED);
//     }
    
//     uint8_t getRotation() {
//       return m_knobRotation;
//     }
// };

// Knob knob3(8,0);
// Knob knob2(8,0);
// Knob knob1(8,0);

// // Instantiate ADSR class
// ADSR adsr1(0, 0.3, 0.1, 0.2, 0.5);

// // Instantiate FILTER class
// FILTER filter1;
// // Instantiate ECHO class
// Echo echo;

// // Instatiate POLYPHONY class
// POLYPHONY polyphony1;

// // Joystick 

// //multithreading global variable 
// //SemaphoreHandle_t keyArrayMutex;
// SemaphoreHandle_t RXMessageMutex; 
// //SemaphoreHandle_t KnobMutex;
// SemaphoreHandle_t RXMutex;

// // CAN bus protocol 
// uint8_t Key_press[8] = {0}; 
// QueueHandle_t msgInQ;
// QueueHandle_t msgOutQ;
// SemaphoreHandle_t CAN_TX_Semaphore;
// boolean west; 
// boolean east;
// //#define SAMPLE_BUFFER_SIZE 2000
// //int32_t sampleBuffer0[SAMPLE_BUFFER_SIZE*2];
// //int32_t *sampleBuffer1 = sampleBuffer0 + SAMPLE_BUFFER_SIZE;
// // Function to set outputs using key matrix
// void setOutMuxBit(const uint8_t bitIdx, const bool value)
// {
//   digitalWrite(REN_PIN, LOW);
//   digitalWrite(RA0_PIN, bitIdx & 0x01);
//   digitalWrite(RA1_PIN, bitIdx & 0x02);
//   digitalWrite(RA2_PIN, bitIdx & 0x04);
//   digitalWrite(OUT_PIN, value);
//   digitalWrite(REN_PIN, HIGH);
//   delayMicroseconds(2);
//   digitalWrite(REN_PIN, LOW);
// }
// void setRow(uint8_t rowIdx)
// {
//   digitalWrite(REN_PIN, LOW);

//   digitalWrite(RA0_PIN, rowIdx & 0x01);
//   digitalWrite(RA1_PIN, rowIdx & 0x02);
//   digitalWrite(RA2_PIN, rowIdx & 0x04);

//   digitalWrite(REN_PIN, HIGH);
// }

// uint8_t readCols(uint8_t row)
// {
//   setRow(row);
//   delayMicroseconds(3);

//   uint8_t col0 = digitalRead(C0_PIN) ? 1 : 0;
//   uint8_t col1 = digitalRead(C1_PIN) ? 2 : 0;
//   uint8_t col2 = digitalRead(C2_PIN) ? 4 : 0;
//   uint8_t col3 = digitalRead(C3_PIN) ? 8 : 0;

//   return col0 | col1 | col2 | col3;
// }

// const char *mapKeys(uint32_t keys)
// {
//   const char *localkeyNote = "0";
//   for (int i = 0; i < 12; i++)
//   {
//     localkeyNote = ((keys & 0x1 << i) > 0) ? localkeyNote : keyNote[i];
//   }
//   return localkeyNote;
// }

// uint32_t mapStepsize(uint32_t keys)
// {
//   uint32_t localStepSize = 0;
//   for (int i = 0; i < 12; i++)
//   {
//     localStepSize = ((keys & 0x1 << i) > 0) ? localStepSize : stepSizes[i];
//   }
//   return localStepSize;
// }


// // CAN BUS
// /*void CAN_RX_ISR (void) {

// 	uint8_t RX_Message_ISR[8];
// 	uint32_t ID;
// 	CAN_RX(ID, RX_Message_ISR);
// 	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);

// }
// void CAN_TX_ISR (void) {
// 	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
// }*/

// void sampleISR()
// {
//   static uint32_t phaseAcc = 0;
//   //phaseAcc += currentStepSize;
//   //phaseAcc += polyphony1.do_Polyphony();
//   phaseAcc += ((Actual_mode == POLYPHONY_MODE) ? polyphony1.do_Polyphony() : currentStepSize);
//   //uint32_t currentStepSize_1 = currentStepSize;
//   //int32_t Vout = (phaseAcc >> 24) - 128;
//   //Vout = Vout >> (8 - knob_rotation);
//   //Serial.println(currentStepSize);
//   //int32_t Vout = adsr1.Do_ADSR(phaseAcc, polyphony1.do_Polyphony());
//   int32_t Vout = ((Actual_mode == NORMAL_MODE) || (Actual_mode == POLYPHONY_MODE) || (Actual_mode == FILTER_MODE)) ? ((phaseAcc >> 24) - 128) : adsr1.Do_ADSR(phaseAcc, currentStepSize);
//   //check = adsr1.Check(currentStepSize);
//   if(Actual_mode == FILTER_MODE)
//   {
//     Vout = filter1.LowPassFilter(Vout, knob2.knob_rotation);
//     Vout = filter1.HighPassFilter(Vout, knob1.knob_rotation);
//   }
//   //
//   if(Actual_mode == ECHO_MODE){Vout = echo.do_Echo(Vout, currentStepSize);}
//   //Vout = echo.do_Echo(Vout, polyphony1.do_Polyphony());
//   Vout = Vout >> (8 - knob3.knob_rotation);

//   //
//   if((Actual_mode == NORMAL_MODE) || (Actual_mode == POLYPHONY_MODE) || (Actual_mode == FILTER_MODE)){analogWrite(OUTR_PIN, Vout+128);}
//   else{analogWrite(OUTR_PIN, Vout);}
  
// }

// void scanKeysTask(void *pvParameters)
// {

//   const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
//   TickType_t xLastWakeTime = xTaskGetTickCount();
  

//   static uint32_t localKnob3;
  

//   while (1)
//   { 
    
//     vTaskDelayUntil( &xLastWakeTime, xFrequency );
//     // reading input
//     for (int i = 0; i < 6; i++)
//     {                            // depending on the number of row(not collumn) i need to be change
//       xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
//       keyArray[i] = readCols(i); // reading the 4 collum of row i
//       xSemaphoreGive(keyArrayMutex);
//     }
//     uint8_t localKnob3_current = keyArray[3]&0x3;
//     uint8_t localKnob2_current = (keyArray[3]&0xC)>>2;
//     uint8_t localKnob1_current = keyArray[4]&0x3;
//     knob3.update(localKnob3_current);
//     knob2.update(localKnob2_current);
//     knob1.update(localKnob1_current);

//     uint32_t keys = keyArray[2]<<8 | keyArray[1]<<4 | keyArray[0];
//     polyphony1.Polyphony_mapStepsize(keys);
    
//     //int32_t X = (int32_t)analogRead(JOYX_PIN);
//     uint32_t localCurrentStepSize=mapStepsize(keys);
//     //phaseAcc_1 += localCurrentStepSize;
//     //Serial.println(localCurrentStepSize);
//     //Serial.println(currentStepSize);
//     Serial.println(Joystick_Y);
//     __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
//     //__atomic_store_n(&currentStepSize, phaseAcc_1, __ATOMIC_RELAXED);
//   }
// }

// void modeSelection(void *pvParameters)
// {
//   const TickType_t xFrequency = 250 / portTICK_PERIOD_MS;
//   TickType_t xLastWakeTime = xTaskGetTickCount();

//   while(1){
//     vTaskDelayUntil( &xLastWakeTime, xFrequency );
//     int8_t local_Actual_mode = Actual_mode;
//     xSemaphoreTake(ModeselectionMutex, portMAX_DELAY);
//     Joystick_Y = (int32_t)analogRead(JOYY_PIN);
//     xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
//     uint8_t Joystick_click = (keyArray[5]&0x4)>>2;
//     xSemaphoreGive(keyArrayMutex);
//     if(Joystick_Y > 780)
//     {
//       mode_display += 1;
//       if(mode_display > MODE_MAX){ mode_display = NORMAL_MODE; }
//       if(Joystick_click == 1){ local_Actual_mode += 1; }
//       if(local_Actual_mode > MODE_MAX){ local_Actual_mode = NORMAL_MODE; }
//     }
//     else if(Joystick_Y < 200)
//     {
//       mode_display -= 1;
//       if(mode_display < 0 ){ mode_display = NORMAL_MODE; }
//       if(Joystick_click == 1){ local_Actual_mode -= 1; }
//       if(local_Actual_mode < 0 ){ local_Actual_mode = NORMAL_MODE; }
//     }
//     else{mode_display = mode_display; local_Actual_mode = local_Actual_mode;}
//     xSemaphoreGive(ModeselectionMutex);
//     __atomic_store_n(&Actual_mode, local_Actual_mode, __ATOMIC_RELAXED);


//   }
// }

// void displayUpdateTask(void *pvParameters)
// {
//   const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
//   TickType_t xLastWakeTime = xTaskGetTickCount();
  
//   static uint32_t localKnob3;

//   while (1)
//   { 
//     //#ifndef TEST_DISPLAY
//     vTaskDelayUntil(&xLastWakeTime, xFrequency);
//     //#endif

//     xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
//     uint32_t keys = keyArray[2]<<8 | keyArray[1]<<4 | keyArray[0];
//     xSemaphoreGive(keyArrayMutex);

    
//     /*u8g2.clearBuffer();         // clear the internal memory
//     u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
//     u8g2.drawStr(2,10,"Helllo World!");  // write something to the internal memory
//     u8g2.setCursor(2,20);
//     u8g2.print(knob3.getRotation(),HEX);
//     u8g2.setCursor(10,20);
//     u8g2.print(knob2.getRotation(),HEX);
//     u8g2.setCursor(18,20);
//     u8g2.print(knob1.getRotation(),HEX);
//     u8g2.sendBuffer();          // transfer internal memory to the display*/
//     xSemaphoreTake(ModeselectionMutex, portMAX_DELAY);
//     switch(mode_display){
//       case NORMAL_MODE:
//             u8g2.clearBuffer();         // clear the internal memory
//             u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
//             u8g2.drawStr(2,10,"Normal Mode");  // write something to the internal memory
//             //u8g2.setCursor(2,20);
//             u8g2.drawStr(2, 20, "----------------------------");
//             u8g2.drawStr(2, 30,"VOICE: ");
//             u8g2.setCursor(45, 30);
//             u8g2.print(knob3.getRotation(),HEX);
//             /*u8g2.setCursor(10,20);
//             u8g2.print(knob2.getRotation(),HEX);
//             u8g2.setCursor(18,20);
//             u8g2.print(knob1.getRotation(),HEX);*/
//             u8g2.sendBuffer();
//             break;
//       case ADSR_MODE:
//             u8g2.clearBuffer();         // clear the internal memory
//             u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
//             u8g2.drawStr(2,10,"ADSR Mode");  // write something to the internal memory
//             //u8g2.setCursor(2,20);
//             u8g2.drawStr(2, 20, "----------------------------");
//             u8g2.drawStr(2, 30,"VOICE: ");
//             u8g2.setCursor(45, 30);
//             u8g2.print(knob3.getRotation(),HEX);
//             /*u8g2.setCursor(10,20);
//             u8g2.print(knob2.getRotation(),HEX);
//             u8g2.setCursor(18,20);
//             u8g2.print(knob1.getRotation(),HEX);*/
//             u8g2.sendBuffer();
//             break;
//       case FILTER_MODE:
//             u8g2.clearBuffer();         // clear the internal memory
//             u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
//             u8g2.drawStr(2,10,"Filter Mode");  // write something to the internal memory
//             //u8g2.setCursor(2,20);
//             u8g2.drawStr(2, 20, "----------------------------");
//             u8g2.drawStr(2, 30,"VOICE: ");
//             u8g2.setCursor(45, 30);
//             u8g2.print(knob3.getRotation(),HEX);
//             //u8g2.setCursor(50,30);
//             u8g2.drawStr(55, 30,"LP: ");
//             u8g2.setCursor(75, 30);
//             u8g2.print(knob2.getRotation(),HEX);
//             u8g2.drawStr(85, 30, "HP: ");
//             u8g2.setCursor(110,30);
//             u8g2.print(knob1.getRotation(),HEX);
//             u8g2.sendBuffer();
//             break;
//       case ECHO_MODE:
//             u8g2.clearBuffer();         // clear the internal memory
//             u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
//             u8g2.drawStr(2,10,"Echo Mode");  // write something to the internal memory
//            // u8g2.setCursor(2,20);
//             u8g2.drawStr(2, 20, "----------------------------");
//             u8g2.drawStr(2, 30,"VOICE: ");
//             u8g2.setCursor(45, 30);
//             u8g2.print(knob3.getRotation(),HEX);
//             /*u8g2.setCursor(10,20);
//             u8g2.print(knob2.getRotation(),HEX);
//             u8g2.setCursor(18,20);
//             u8g2.print(knob1.getRotation(),HEX);*/
//             u8g2.sendBuffer();
//             break;
//       case POLYPHONY_MODE:
//             u8g2.clearBuffer();         // clear the internal memory
//             u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
//             u8g2.drawStr(2,10,"Polyphony Mode");  // write something to the internal memory
//             u8g2.drawStr(2, 20, "----------------------------");
//             u8g2.drawStr(2, 30,"VOICE: ");
//             u8g2.setCursor(45, 30);
//             //u8g2.setCursor(2,20);
//             u8g2.print(knob3.getRotation(),HEX);
//             /*u8g2.setCursor(10,20);
//             u8g2.print(knob2.getRotation(),HEX);
//             u8g2.setCursor(18,20);
//             u8g2.print(knob1.getRotation(),HEX);*/
//             u8g2.sendBuffer();
//             break;
//       default:
//             u8g2.clearBuffer();         // clear the internal memory
//             u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
//             u8g2.drawStr(2,10,"Normal Mode");  // write something to the internal memory
//             u8g2.drawStr(2, 20, "----------------------------");
//             u8g2.drawStr(2, 30,"VOICE: ");
//             u8g2.setCursor(45, 30);
//            // u8g2.setCursor(2,20);
//             u8g2.print(knob3.getRotation(),HEX);
//             /*u8g2.setCursor(10,20);
//             u8g2.print(knob2.getRotation(),HEX);
//             u8g2.setCursor(18,20);
//             u8g2.print(knob1.getRotation(),HEX);*/
//             u8g2.sendBuffer();
//             break;
//     }
//     xSemaphoreGive(ModeselectionMutex);
 
//     digitalToggle(LED_BUILTIN);
//     //#ifdef TEST_DISPLAY
//       //break;
//     //#endif
//   }
// }

// // Decode task for CAN
// void decodeTask(void * pvParameters){
//   uint32_t localCurrentStepSize ;
//   uint8_t l_RXMessage[8] = {0};
//   while(1){
    
//     xQueueReceive(msgInQ, l_RXMessage, portMAX_DELAY);
//     if(east == 1 && west == 0){
//     if(l_RXMessage[0] == 'R'){
//       localCurrentStepSize = 0;
//        __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
//        xSemaphoreTake(RXMessageMutex, portMAX_DELAY);
//           Key_press[0] = l_RXMessage[0] ; 
//           Key_press[1] = l_RXMessage[1] ; 
//           Key_press[2] = l_RXMessage[2] ; 
//        xSemaphoreGive(RXMessageMutex);
//     }
//     else if(l_RXMessage[0] == 'P'){
//       localCurrentStepSize = stepSizes[l_RXMessage[2]];
//       xSemaphoreTake(RXMessageMutex, portMAX_DELAY);
//         Key_press[0] = l_RXMessage[0] ; 
//         Key_press[1] = l_RXMessage[1] ; 
//         Key_press[2] = l_RXMessage[2] ; 
//       xSemaphoreGive(RXMessageMutex);
//       // 
//     }
//     if(localCurrentStepSize != 0){
//       __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
//       localCurrentStepSize = 0;
//     }
//     }


//   }
// }


// /*void CAN_TX_Task (void * pvParameters) {
// 	uint8_t msgOut[8];
// 	while (1) {
// 	xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
// 		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
// 		CAN_TX(0x123, msgOut);
// 	}
// }*/


// void setup()
// {
//   // put your setup code here, to run once:

//   // Set pin directions
//   pinMode(RA0_PIN, OUTPUT);
//   pinMode(RA1_PIN, OUTPUT);
//   pinMode(RA2_PIN, OUTPUT);
//   pinMode(REN_PIN, OUTPUT);
//   pinMode(OUT_PIN, OUTPUT);
//   pinMode(OUTL_PIN, OUTPUT);
//   pinMode(OUTR_PIN, OUTPUT);
//   pinMode(LED_BUILTIN, OUTPUT);

//   pinMode(C0_PIN, INPUT);
//   pinMode(C1_PIN, INPUT);
//   pinMode(C2_PIN, INPUT);
//   pinMode(C3_PIN, INPUT);
//   pinMode(JOYX_PIN, INPUT);
//   pinMode(JOYY_PIN, INPUT);

//   // Initialise display
//   setOutMuxBit(DRST_BIT, LOW); // Assert display logic reset
//   delayMicroseconds(2);
//   setOutMuxBit(DRST_BIT, HIGH); // Release display logic reset
//   u8g2.begin();
//   setOutMuxBit(DEN_BIT, HIGH); // Enable display power supply

//   // Initialise UART
//   Serial.begin(9600);
//   Serial.println("Hello World");
  

//   // interupt
//   TIM_TypeDef *Instance = TIM1;
//   HardwareTimer *sampleTimer = new HardwareTimer(Instance);
//   sampleTimer->setOverflow(22000, HERTZ_FORMAT);
//   sampleTimer->attachInterrupt(sampleISR);
//   sampleTimer->resume();

//   //instatiate CAN
//   msgInQ = xQueueCreate(36,8);
//   msgOutQ = xQueueCreate(36,8);
 
//   /*CAN_Init(false);
//   setCANFilter(0x123,0x7ff);
//   CAN_RegisterRX_ISR(CAN_RX_ISR);
//   CAN_RegisterTX_ISR(CAN_TX_ISR);
//   CAN_Start();*/


//   keyArrayMutex = xSemaphoreCreateMutex();
//   RXMessageMutex = xSemaphoreCreateMutex();

//   CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

//   ModeselectionMutex = xSemaphoreCreateMutex();

//   // Initialize DMA
//   //DMA_Configuration();

//   // multithreading
//   // multithreading for scankey
//   TaskHandle_t scanKeysHandle = NULL;
//   xTaskCreate(
//       scanKeysTask,     /* Function that implements the task */
//       "scanKeys",       /* Text name for the task */
//       64,               /* Stack size in words, not bytes */
//       NULL,             /* Parameter passed into the task */
//       3,                /* Task priority */
//       &scanKeysHandle); /* Pointer to store the task handle */

  
//   // multithreading for modeselection
//   TaskHandle_t modeSelectionHandle = NULL;
//   xTaskCreate(
//       modeSelection,     /* Function that implements the task */
//       "modeselection",       /* Text name for the task */
//       20,               /* Stack size in words, not bytes */
//       NULL,             /* Parameter passed into the task */
//       1,                /* Task priority */
//       &modeSelectionHandle);

//   // multithreading for display task
//   TaskHandle_t displayUpdateHandle = NULL;
//   xTaskCreate(
//       displayUpdateTask,     /* Function that implements the task */
//       "displayupdate",       /* Text name for the task */
//       256,                   /* Stack size in words, not bytes */
//       NULL,                  /* Parameter passed into the task */
//       2,                     /* Task priority */
//       &displayUpdateHandle); /* Pointer to store the task handle */
  
//   TaskHandle_t decodeDataHandle = NULL;
//   xTaskCreate(
//       decodeTask, 
//       "decodeTX", 
//       256, 
//       NULL, 
//       3, //temporary 
//       &decodeDataHandle);


//   /*TaskHandle_t CantxHandle = NULL;
//   xTaskCreate(
//       CAN_TX_Task, 
//       "decodeTX", 
//       256, 
//       NULL, 
//       3, //temporary 
//       & CantxHandle );*/

  
//   vTaskStartScheduler();
// }
// void loop() {}
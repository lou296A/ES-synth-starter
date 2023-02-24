#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>

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
  volatile uint32_t currentStepSize;
  volatile uint8_t keyArray[7];
//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

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
uint8_t readCols(uint8_t  row){
   
 
  setRow(row); 
  uint8_t  col_val;
  digitalWrite(REN_PIN,HIGH);
  col_val |= digitalRead(C0_PIN) << 0;
  col_val |= digitalRead(C1_PIN) << 1;
  col_val |= digitalRead(C2_PIN) << 2;
  col_val |= digitalRead(C3_PIN) << 3;  
  digitalWrite(REN_PIN,LOW);
  return col_val; 

}
void sampleISR() {
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  analogWrite(OUTR_PIN, Vout + 128);
}
void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
   vTaskDelayUntil( &xLastWakeTime, xFrequency );
   uint32_t localCurrentStepSize;
   for(int i = 0; i < 3 ; i++){
      keyArray[i] = readCols(i);
      delayMicroseconds(3);
      }
    for(int i = 0; i < 3 ; i++){
      Serial.println(keyArray[i]); 
      for(int j = 0; j < 4 ; j++){
        if(((keyArray[i] >> j) & 0x01) == 0){
          localCurrentStepSize = stepSizes[i*4+j];
        }
    
      
      }
    }
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
    Serial.print("Stepsize:"); 
    Serial.println(currentStepSize); 
  }

}

void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 1000/portTICK_PERIOD_MS;
   static uint32_t count = 0;
  TickType_t xLastWakeTime = xTaskGetTickCount(); 
  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"Helllo World!");  // write something to the internal memory
    u8g2.setCursor(2,20);
    u8g2.print(count++);
    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);
  

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
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();
  
  
  //multithreading
    //multithreading for scankey
TaskHandle_t scanKeysHandle = NULL;
xTaskCreate(
scanKeysTask,		/* Function that implements the task */
"scanKeys",		/* Text name for the task */
64,      		/* Stack size in words, not bytes */
NULL,			/* Parameter passed into the task */
2,			/* Task priority */
&scanKeysHandle );  /* Pointer to store the task handle */
  
  //multithreading for display task
TaskHandle_t displayUpdateHandle = NULL;
xTaskCreate(
displayUpdateTask,		/* Function that implements the task */
"displayupdate",		/* Text name for the task */
256,      		/* Stack size in words, not bytes */
NULL,			/* Parameter passed into the task */
1,			/* Task priority */
&displayUpdateHandle);  /* Pointer to store the task handle */
vTaskStartScheduler();
}
void print_binary_V2(uint8_t decimal){
 
 unsigned char* binary = reinterpret_cast<unsigned char*>(&decimal);
 unsigned char mask = 0x80;

  // Print the binary representation
  Serial.print("col representation: ");
  for (int i = 0; i < sizeof(decimal); i++) {
    for (int j = 0; j < 8; j++) {
      
      Serial.print((binary[i] & mask) ? 1 : 0);
      mask >>= 1;
    }
    mask = 0x80;
  }
  Serial.println();
}
void loop() {}
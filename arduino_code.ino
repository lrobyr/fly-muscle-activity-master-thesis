// Define the pin connected to the optocoupler signal
#define CAMERA_PIN 11
#define RINGLIGHT_PIN 9
#define BLUESTIMLIGHT_PIN 3
#define REDSTIMLIGHT_PIN 10

// Define the pin connected to the 5V exposure signal
#define EXPOSURE_PIN 6 // Pin for the exposure signal

volatile bool lightOn = false;

// Global state
volatile uint16_t tickTarget = 0;          
// Number of 0.1ms intervals
volatile uint16_t tickCounter = 0;

int STIMLIGHTIMAGING_PIN;
int OPTOLIGHT_PIN;

// Initialise a fps variable to store the framerate
int fps;

// exposure time in microsecondsOPTOLIGHT_PIN
unsigned long exp_time; 

// Initialise a duration variable to store the experiment duration
unsigned long duration;

// Variable to store the interval between image captures
int CAPTURE_INTERVAL;

// Frame counter to keep track of even and odd frames
unsigned long frameCounter = 0;

bool toggle1 = false;

bool alternate = true;
bool opto = false;

int off1_dur = 0;
int on1_dur = 0;
int off2_dur = 0;
unsigned long cycleCounter = 0;
unsigned long n_off1_cycles = 0;
unsigned long n_on1_cycles = 0;
unsigned long n_off2_cycles = 0;
enum OptoState {
  OFF1,
  ON1,
  OFF2
};

// Initial state to ON1 in case it is not used
OptoState currOptoState = ON1;  

unsigned long readInteger() {
  unsigned long ret = 0;
  while (true) {
    while (Serial.available() < 1) {
      ; // Wait for data to be available
    }
    char newChar = Serial.read();
    if (newChar == '*') {
      break; // Stop reading when '*' is encountered
    }
    if (newChar != '*' && newChar > 47) {
      int newDigit = newChar - '0';
      ret = ret * 10;
      ret = ret + newDigit;
    }
  }
  return ret;
}

bool readBoolean() {
  while (true) {
    while (Serial.available() < 1) {
      ; // Wait for data to be available
    }
    char newChar = Serial.read();

    // Accept '0' or '1' as boolean values
    if (newChar == '0') {
      return false;
    } else if (newChar == '1') {
      return true;
    }
  }
}

void setTimerInterrupts(int fps)
{  
  // Set the prescaler e.g the speed at witch the counter increments
  // For FPS going from 31 to 2000
  long int prescale_8_CMR    = (16000000 / (2*8*fps)) -1; 
  
  // For FPS going from 1 to 30
  long int prescale_1024_CMR = (16000000 / (2*1024*fps)) -1;  
  

  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register
  if (prescale_8_CMR > 256 && prescale_8_CMR < 65536) 
  {
    OCR1A  = prescale_8_CMR; // (must be <65536)
  }
  else
  {
    OCR1A  = prescale_1024_CMR;
  }
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);

  // set prescaler 
  if (prescale_8_CMR > 256 && prescale_8_CMR < 65536) 
  {
    // Set CS11 bit for 8 prescaler
    TCCR1B |= (1 << CS11);  
  }
  else
  {
    // Set CS10 and CS12 bits for 1024 prescaler
    TCCR1B |= (1 << CS12) | (1 << CS10); 
  }
 
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
}

void setupTimer2ForLightPulse(unsigned long microseconds) {
  // Clamp microsecond input
  if (microseconds < 100) microseconds = 100;
  if (microseconds > 1000000) microseconds = 1000000;

  // Each tick is 0.1ms (100 µs)
  tickTarget = microseconds / 100;
  tickCounter = 0;

  // Reset Timer2 registers
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  TIMSK2 = 0;

  // CTC Mode with OCR2A match
  OCR2A = 25; // 25 ticks × 4 µs = 100 µs = 0.1 ms
  TCCR2A |= (1 << WGM21);      // CTC mode
  TCCR2B |= (1 << CS22);       // Prescaler = 64
  TIMSK2 |= (1 << OCIE2A);     // Enable Compare Match A interrupt
}

void setup() {

  // Set the camera pin as output
  pinMode(CAMERA_PIN, OUTPUT);
  pinMode(RINGLIGHT_PIN, OUTPUT);

  // Set the exposure pin as input
  pinMode(EXPOSURE_PIN, INPUT);  

  // Initialize serial communication at 9600 baud
  Serial.begin(9600);

  // Send acknowledgment message
  Serial.println("Arduino Ready");
}

ISR(TIMER1_COMPA_vect)
{
  // timer1 interrupt interrupts twice per 1/fps (see /2 in setup)
  // Generates squares with period 1/fps and on dur 1/fps/2
  if (toggle1)
  {
    digitalWrite(CAMERA_PIN,HIGH);
    toggle1 = 0;
    frameCounter++;
    cycleCounter++;
    lightOn = false;
  }
  else
  {
    digitalWrite(CAMERA_PIN,LOW);
    toggle1 = 1;
  }
}

ISR(TIMER2_COMPA_vect) {
  tickCounter++;

  if (tickCounter >= tickTarget) {
    // End of light pulse
    digitalWrite(RINGLIGHT_PIN, LOW);
    digitalWrite(STIMLIGHTIMAGING_PIN, LOW);
    TIMSK2 &= ~(1 << OCIE2A); // Disable interrupt
    TCCR2B = 0;
  }
}

void loop() {
// Check if there's data available on the serial port
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');

    if (command == "setup") {
      Serial.println("Waiting for fps");
      fps = readInteger();
      Serial.print("Received updated fps value from Python: ");
      Serial.println(fps);

      duration = readInteger();         
      Serial.print("Received updated duration value from Python: ");
      Serial.println(duration);

      exp_time = readInteger();
      Serial.print("Received updated exp_time value from Python: ");
      Serial.println(exp_time);
      
      alternate = readBoolean();
      if (alternate){
        Serial.println("Muscle and kinematic frames captured in alternation");
      }else{
        Serial.println("Muscle frames only!");
      }
      opto = readBoolean();
      if (opto){
        Serial.println("Opto experiment:");
        currOptoState = OFF1;

        off1_dur = readInteger();         
        Serial.print("OFF1 duration: ");
        Serial.print(off1_dur);
        n_off1_cycles = off1_dur*fps;
        Serial.print(" correspoding number of cycles: ");
        Serial.println(n_off1_cycles);

        on1_dur = readInteger();         
        Serial.print("ON1 duration: ");
        Serial.println(on1_dur);
        n_on1_cycles = on1_dur*fps;
        Serial.print(" correspoding number of cycles: ");
        Serial.println(n_on1_cycles);

        off2_dur = readInteger();         
        Serial.print("OFF2 duration: ");
        Serial.println(off2_dur);
        n_off2_cycles = off2_dur*fps;

        STIMLIGHTIMAGING_PIN = REDSTIMLIGHT_PIN;
        OPTOLIGHT_PIN = BLUESTIMLIGHT_PIN;

      }else{
        STIMLIGHTIMAGING_PIN = BLUESTIMLIGHT_PIN;
        OPTOLIGHT_PIN = REDSTIMLIGHT_PIN;
        Serial.println("Opto OFF");
      }

      pinMode(STIMLIGHTIMAGING_PIN, OUTPUT);
      pinMode(OPTOLIGHT_PIN, OUTPUT);

      Serial.println("Ready to setup interrupts");
      
      //Wait fro the program to be ready
      while (true) {
        if (Serial.available()) {
          String command = Serial.readStringUntil('\n');
          if (command == "launch") {
            break;
          }
        }
      }
      
      // Temporarily disable interrupts
      noInterrupts();
      
      // Set up timer interrupts with the received fps value
      setTimerInterrupts(fps);

      // Re-enable interrupts
      interrupts();

      // Capture images for the specified duration
      unsigned long startTime = millis();
      
      while (millis() - startTime < duration * 1000) {
        // Check what LED to turn on
        if (digitalRead(EXPOSURE_PIN) == HIGH && !lightOn) {  
          lightOn = true;

          if (frameCounter % 2 == 0 && alternate) {
            digitalWrite(RINGLIGHT_PIN, HIGH);
            digitalWrite(STIMLIGHTIMAGING_PIN, LOW);
            
          } else {
            digitalWrite(RINGLIGHT_PIN, LOW);  
            digitalWrite(STIMLIGHTIMAGING_PIN, HIGH);
          }
            // Start short light pulse
            setupTimer2ForLightPulse(exp_time);  
        }

        // If imaging + optostimulation imaging light ON, 
        // opto light ON / OFF according to GUI commands
        if (opto  && (currOptoState == ON1)){
          digitalWrite(OPTOLIGHT_PIN, HIGH);

        } else {
          digitalWrite(OPTOLIGHT_PIN, LOW);
        }

        if (opto){
          if (currOptoState == OFF1){
            if (cycleCounter > n_off1_cycles){
              currOptoState = ON1;
              cycleCounter = 0;
            }
           } else if (currOptoState == ON1){
            if (cycleCounter > n_on1_cycles){
              currOptoState = OFF2;
              cycleCounter = 0;             
            }
          } else if (currOptoState == OFF2){
            if (cycleCounter > n_off2_cycles){
              currOptoState = OFF1;
              cycleCounter = 0;            
            }
          }
        }
        
        if (Serial.available()) {
          String command = Serial.readStringUntil('\n');
          if (command == "stop"){
            break;
          }
        }
      }
      // After the duration has passed, stop capturing images
      TIMSK1 &= ~(1 << OCIE1A);
      digitalWrite(CAMERA_PIN, LOW);
      digitalWrite(RINGLIGHT_PIN, LOW);
      digitalWrite(STIMLIGHTIMAGING_PIN, LOW);
      digitalWrite(OPTOLIGHT_PIN, LOW);
      
      delay(500);
      // Send "done" message to Python script
      Serial.print("Done number red frames: ");
      Serial.println(frameCounter);
    }
  }
}
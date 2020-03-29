#include <SoftwareSerial.h>

/* General defines */
#define CPU_CLK_FREQ    16000000    /* Microcontroller clock frequency - 16 Mhz crystal */
#define PRESCALER       64          /* Prescaler value */
#define CPQN            24          /* Pulses per quarter note - MIDI smallest unit of time, 24 is stardard */
#define MIN_SEC         60          /* Seconds in a minute */
#define PAUSE_BLINK_MS  300         /* Play led blinking time in ms when in pause mode */
#define CLK_SRC_INT     1           /* Clock source selection - internal clock */
#define CLK_SRC_EXT     2           /* Clock source selection - external clock */

/* Pin mapping */
#define RX_PIN          2           /* Serial line RX pin */
#define TX_PIN          3           /* Serial line TX pin */
#define CLK_SRC_PIN     4           /* Pin number associated to clock source selection switch - digital output */
#define PLAY_BTN_PIN    5           /* Pin number associated to play button - digital output */
#define STOP_BTN_PIN    8           /* Pin number associated to stop button - digital output */
#define PLAY_LED_PIN    11          /* Pin number associated to led signalling play mode - digital output */
#define EXT_LED_PIN     12          /* Pin number associated to led signalling external clock source - digital output */
#define INT_LED_PIN     13          /* Pin number associated to led signalling internal clock source - digital output */
#define TEMPO_PIN       A0          /* Pin number associated to tempo selection - analog output */

/* MIDI messages */
#define MIDI_START      0xFA        /* MIDI start message */
#define MIDI_CONTINUE   0xFB        /* MIDI continue message */
#define MIDI_STOP       0xFC        /* MIDI stop message */
#define MIDI_CLOCK      0xF8        /* MIDI clock message */
#define MIDI_NOTE_ON    0x90        /* MIDI note on message */

/* General variables */
byte data;                          /* Serial line data byte */
byte pitch;                         /* Pitch byte */
byte velocity;                      /* Velocity byte */
int source = CLK_SRC_INT;           /* Clock source value - internal clock by default */
int bpm = 0;                        /* BPM value */
int currentPqn = 1;                 /* Current pqn value */       
long prevMillis = 0;                /* Previous millisecond value */
boolean playFlag = false;           /* Flag indicating if in play mode */
boolean pauseFlag = false;          /* Flag indicating if in pause mode */

/* Leds */
int ledState = LOW;
int playLedState = HIGH; 

/* Buttons */
int numPressed = 0;                 /* Counter of toggles on play button */
int playCurState = LOW;             /* Play button current state - low state by default */
int playPrevState = LOW;            /* Play button previous state - low state by default */
int stopCurState = LOW;             /* Stop button current state - low state by default */
int stopPrevState = LOW;            /* Stop button previous state - low state by default */
int sourceCurState = LOW;           /* Source button current state - low state by default */
int sourcePrevState = LOW;          /* Source button previous state - low state by default */

/* Potentiometer */
int tempoVal = 0;                   /* Tempo potentiometer value*/

/* Serial line initialization */
SoftwareSerial midiSerial(RX_PIN, TX_PIN);


/* 
 *  Timer1 interrupt service routine
 *  Generates pulse wave of frequency depending on potentiometer
 */
 ISR(TIMER1_COMPA_vect) 
 {
   
  if (tempoVal != bpm) {
     updateBPM();  // BPM must be update at next interrupt to avoid jitter
  }

  /* If in play mode, send MIDI clock and manage LED blinking */
  if (playFlag == true) {
    sendMessages();  
    
  }
  /* Else if in pause mode, onyl manage play led blinking */
  else if (pauseFlag == true) {
    blinkPlayLed();
  }
}

/* 
 *  Read BPM value
 */

void readBPM(){
  /* Read tempo value from analog pin */
  tempoVal = analogRead(TEMPO_PIN);

  /* Map potentiometer value from minimum bpm (60) to maximum bpm (200) */
  tempoVal = map(tempoVal, 0, 1023, 60, 200);
}

/* 
 *  Update BPM value by writing the compare match register
 *  compare match register = [ Arduino clock speed / (prescaler * desired interrupt frequency) ] - 1
 *  
 *  midi clock = 24 pulses per quarter note (quarte note = BPM)
 *  clock (ms) = 60 / (120 * 24)
 */
void updateBPM(){
  bpm = tempoVal;
  OCR1A = (MIN_SEC * CPU_CLK_FREQ / PRESCALER) / (bpm * CPQN) - 1;
}

/* 
 *  Counting 24 cpqn to manage LED and MIDI sync message
 */
void sendMessages()
{   

  /* Set pin to blink in relation to selected clock source */ 
  int ledPin;
  if (source == CLK_SRC_INT) {
    ledPin = INT_LED_PIN;
  }
  else if (source == CLK_SRC_EXT) {
    ledPin = EXT_LED_PIN;
  }

  if (currentPqn < CPQN) {
    if (currentPqn == 12) {
      /* Blink selected clock source led */
      blinkLed(ledPin);
    }

    /* Increment pqn counter */
    currentPqn++;

    /* Send midi clock signal */
    midiSerial.write(MIDI_CLOCK); 
  }
  else if (currentPqn == CPQN) {
    /* Reset pqn counter */
    currentPqn = 1;

    /* Blink selected clock source led */
    blinkLed(ledPin);

    /* Send midi clock signal */
    midiSerial.write(MIDI_CLOCK);
  }
}

/* 
 *  Blink given led by inverting its current status
 */
void blinkLed(int pin){
  
  if (ledState == LOW){
      ledState = HIGH;
  }
  else{
      ledState = LOW;
  }
    /* Set the LED state */
    digitalWrite(pin, ledState);
}

/* 
 *  Blink play led without using delay() function
 */
void blinkPlayLed(){
 
  unsigned long curMillis = millis();
 
  if(curMillis - prevMillis > PAUSE_BLINK_MS) {
    /* Store the millisecond of last LED blinking */
    prevMillis = curMillis;   

    /* If the state of the LED is LOW set it HIGH and vice-versa */
    if (playLedState == LOW) {
      playLedState = HIGH;
    }
    else {
      playLedState = LOW;
    }
    
    /* Set the LED state */
    digitalWrite(PLAY_LED_PIN, playLedState);
  }
  
}

/* 
 *  Send MIDI messages for playing a note
 */
void playNote(int cmd, int pitch, int velocity) {
  midiSerial.write(cmd);
  midiSerial.write(pitch);
  midiSerial.write(velocity);
}

void setup() 
{
    
  Serial.begin(9600);
  
  /* Set MIDI baud rate */
  midiSerial.begin(31250);

  /* Set pin directions */
  pinMode(INT_LED_PIN,OUTPUT);
  pinMode(EXT_LED_PIN,OUTPUT);
  pinMode(PLAY_LED_PIN,OUTPUT);
  pinMode(PLAY_BTN_PIN,INPUT);
  pinMode(STOP_BTN_PIN,INPUT);

  /* Set initial state of internal and external source selection led, default mode is internal source */
  digitalWrite(INT_LED_PIN, HIGH); 
  digitalWrite(EXT_LED_PIN, LOW);

  /* Set initial state of play led, default state is OFF */
  digitalWrite(PLAY_LED_PIN, playLedState);  

  /* Disable interrupts */
  cli();

  /* Set timer1 interrupt at 48Hz */
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  
  /* Set compare match register for 48hz increments  ---> 120 BPM */
  OCR1A = 5207;// = (16*10^6) / (48*64) - 1 (must be <65536)
  /* Turn on CTC mode */
  TCCR1B |= (1 << WGM12);
  /* Set CS11 and CS10 bits for 64 prescaler */
  TCCR1B |= (1 << CS11) | (1 << CS10);  
  /* Enable timer compare interrupt */
  TIMSK1 |= (1 << OCIE1A);

  /* Enable interrupts*/
  sei();

}

void loop() 
{
  /* Read selected clock source state */
  sourceCurState = digitalRead(CLK_SRC_PIN);

  /* If source button state changes */
  if ((sourceCurState == HIGH) && (sourcePrevState == LOW)) {

    if (source == CLK_SRC_INT) {
      /* Update source selection */ 
      source = CLK_SRC_EXT;
      /* Set internal clock source led state to LOW */
      digitalWrite(INT_LED_PIN, LOW); 
      /* Set external clock source led state to LOW */
      digitalWrite(EXT_LED_PIN, HIGH);
    }
    else if (source == CLK_SRC_EXT) {
      /* Update source selection */ 
      source = CLK_SRC_INT;
      /* Set external clock source led state to LOW */
      digitalWrite(EXT_LED_PIN, LOW);
      /* Set internal clock source led state to HIGH */
      digitalWrite(INT_LED_PIN, HIGH);
    } 
  }

  /* Store new clock source state as previous */
  sourcePrevState = sourceCurState;

  /* If MIDI info is available at the connector */
  if ((midiSerial.available() > 0) && (source == CLK_SRC_EXT)) { 

    /* Disable timer compare interrupt to avoid double clock info */
    TIMSK1 &= ~((1<<OCIE1A));    
  
    /* Read serial data */
    data = midiSerial.read();
    
    /* Process input MIDI messages */
    switch (data){
    case MIDI_START:
    case MIDI_CONTINUE:
      playFlag = true;
      midiSerial.write(data);
    break;
    case MIDI_STOP:
      playFlag = false;
      midiSerial.write(data);
    break;
    case MIDI_CLOCK:
      if (playFlag == true) {
        sendMessages();
      }
    break;
    /* If serial data reads a MIDI NOTE_ON message, wait for the note to be switched ON */
    case MIDI_NOTE_ON:
      if (playFlag == true) {
        if (pitch != 0) {
          pitch = midiSerial.read();
          velocity = midiSerial.read();
          /* Play a note with given pitch and velocity */
          playNote(MIDI_NOTE_ON, pitch, velocity);
        }
//        else {
//          velocity = data;
//          playNote(MIDI_NOTE_ON, pitch, velocity);
//          pitch = 0;
//          velocity = 0;
//        }
      }
    break;
    /* If serial data reads a MIDI NOTE_OFF message, wait for the note to be switched OFF */
//    case MIDI_NOTE_OFF:
//      if ((playFlag == true) && (pitch == 0)) {
//        pitch = data; 
//        velocity = 0;
//        playNote(MIDI_NOTE_OFF, pitch, velocity);
//        pitch = 0;
//      }
//     break;
    default:
    break;
    }
  }
  else if (source == CLK_SRC_INT) {

    /* Set timer interrupts */
    TIMSK1 |= (1 << OCIE1A); 
    /* Disable interrupts */
    cli();
    /* Read current BPM value */
    readBPM();
    /* Enable interrupts */
    sei();

    /* Read play and stop buton states */
    playCurState = digitalRead(PLAY_BTN_PIN);
    stopCurState = digitalRead(STOP_BTN_PIN);

//  /* If stop button is pressed */
//  if ((stopCurState == HIGH) && (stopPrevState == LOW)) {
//    midiSerial.write(MIDI_STOP);  
//    playFlag = 0;  //reset play flag
//    pauseFlag = 0;  //reset pause flag
//    numPressed = 0; //reset number of times play button has been pressed
//  }

    /* If play button is pressed first time */
    if ((playCurState == HIGH) && (playPrevState == LOW)) {
      if (numPressed == 0) {
        /* Send MIDI start message */
        midiSerial.write(MIDI_START);
        playFlag = true;
        numPressed = 1;
      }
      else if (numPressed == 1) {
        /* Send MIDI stop message */
        midiSerial.write(MIDI_STOP);
        pauseFlag = true; 
        playFlag = false;
        numPressed = 2;

        /* When in pause keep led on */
        digitalWrite(INT_LED_PIN, LOW); 
      }
      else if (numPressed == 2) {
        //digitalWrite(playLedPin, LOW);
        /* Send MIDI continue message */
        midiSerial.write(MIDI_CONTINUE);  
        pauseFlag = false;  
        playFlag = true;
        
        /* Set counter to 1 in order to toggle between play/pause states */
        numPressed = 1; 
      }
    }

    /* Store new play and stop button states as previous */
    playPrevState = playCurState;
    stopPrevState = stopCurState;
  }
}

/*
 * Multiple Button Handling using 1ms timer interrupt.
 * Detects both long/short press and dual button press
 */

const int buttonPin_1 = 2;     // the number of the pushbutton pin
const int buttonPin_2 = 3;     // the number of the pushbutton pin
const int buttonPin_3 = 4;     // the number of the pushbutton pin

const int ledPin =  13;      // the number of the LED pin

// variables will change:
int buttonState_1 = 0;         // variable for reading the pushbutton status
int buttonState_2 = 0;         // variable for reading the pushbutton status
int buttonState_3 = 0;         // variable for reading the pushbutton status

volatile unsigned int button_1_press_cnt = 0;
volatile unsigned int button_2_press_cnt = 0;
volatile unsigned int button_3_press_cnt = 0;

volatile unsigned int mstick = 0;
volatile unsigned int run_time = 0;
volatile unsigned int msec_tick = 0;
volatile unsigned int sec_tick = 0;
volatile unsigned int min_tick = 0;
volatile unsigned int hrs_tick = 0;
volatile unsigned int day_tick = 0;

#define BUTTON_TIME_LIMIT     20
#define BUTTON_SHORT_PRESS    500
#define BUTTON_LONG_PRESS     2000

typedef enum BUTTON_EVENT_E
{
   BUTTON_EVENT_NO_PRESS,
   BUTTON_EVENT_1_SHORT_PRESS,
   BUTTON_EVENT_2_SHORT_PRESS,
   BUTTON_EVENT_3_SHORT_PRESS,
   BUTTON_EVENT_1_LONG_PRESS,
   BUTTON_EVENT_2_LONG_PRESS,
   BUTTON_EVENT_3_LONG_PRESS,
   BUTTON_EVENT_1_3DUAL_PRESS,
} BUTTON_EVENT;

//This is the interrupt request
ISR(TIMER0_COMPA_vect)
{
  mstick++;
  msec_tick++;

  if(msec_tick>1000)
  {
    run_time++;
    sec_tick++;
    msec_tick=0;
    //Serial.println(sec_tick);
  }
  
  if(sec_tick > 60)
  {
    min_tick++;
    sec_tick=0;
  }

  if(min_tick > 60)
  {
    hrs_tick++;
    min_tick=0;
  }
  
  if(hrs_tick >= 24)
  {
    day_tick++;
    hrs_tick = 0;
  }

  if(HIGH == digitalRead(buttonPin_1))
  {
    button_1_press_cnt++;
  }
  if(HIGH == digitalRead(buttonPin_2))
  {
    button_2_press_cnt++;
  }
  if(HIGH == digitalRead(buttonPin_3))
  {
    button_3_press_cnt++;
  }
}

unsigned int msticks(void)
{
   return mstick;
}

void msDelay(unsigned int msec)
{
  unsigned int current = msticks();
  while(msticks() < (current + msec));
}

void setup()
{
  // put your setup code here, to run once:
   pinMode(LED_BUILTIN, OUTPUT);

  // initialize the pushbutton pin as an input:
   pinMode(buttonPin_1, INPUT);
   pinMode(buttonPin_2, INPUT);
   pinMode(buttonPin_3, INPUT);

  //-------- Timer Set : 1ms ---------
  TCCR0A=(1<<WGM01);    //Set the CTC mode   
  OCR0A=0xF9; //Value for ORC0A for 1ms 
  
  TIMSK0|=(1<<OCIE0A);   //Set the interrupt request
  sei(); //Enable interrupt
  
  TCCR0B|=(1<<CS01);    //Set the prescale 1/64 clock
  TCCR0B|=(1<<CS00);

  //------ Serial Init ------
  Serial.begin(9600);
}

BUTTON_EVENT GetButtonEvent(void)
{
  BUTTON_EVENT button_event = BUTTON_EVENT_NO_PRESS;
  if((HIGH != digitalRead(buttonPin_1)) && (BUTTON_TIME_LIMIT < button_1_press_cnt) && (button_1_press_cnt < BUTTON_SHORT_PRESS))
  {
    button_1_press_cnt = 0;
    button_event = BUTTON_EVENT_1_SHORT_PRESS;
  }
  if((HIGH != digitalRead(buttonPin_2)) && (BUTTON_TIME_LIMIT < button_2_press_cnt) && (button_2_press_cnt < BUTTON_SHORT_PRESS))
  {
    button_2_press_cnt = 0;
    button_event = BUTTON_EVENT_2_SHORT_PRESS;    
  }
  if((HIGH != digitalRead(buttonPin_3)) && (BUTTON_TIME_LIMIT < button_3_press_cnt) && (button_3_press_cnt < BUTTON_SHORT_PRESS))
  {
    button_3_press_cnt = 0;
    button_event = BUTTON_EVENT_3_SHORT_PRESS;
  }
  if((HIGH != digitalRead(buttonPin_1)) && button_1_press_cnt > BUTTON_SHORT_PRESS)
  {
    button_1_press_cnt = 0;
    button_event = BUTTON_EVENT_1_LONG_PRESS;
  }
  if((HIGH != digitalRead(buttonPin_2)) && button_2_press_cnt > BUTTON_SHORT_PRESS)
  {
    button_2_press_cnt = 0;
    button_event = BUTTON_EVENT_2_LONG_PRESS;    
  }
  if((HIGH != digitalRead(buttonPin_3)) && button_3_press_cnt > BUTTON_SHORT_PRESS)
  {
    button_3_press_cnt = 0;
    button_event = BUTTON_EVENT_3_LONG_PRESS;
  }
  
  if(button_event == BUTTON_EVENT_1_SHORT_PRESS || button_event == BUTTON_EVENT_1_LONG_PRESS)
  {
    if(BUTTON_TIME_LIMIT < button_3_press_cnt)
    {
       button_1_press_cnt = 0;
       button_3_press_cnt = 0;
       button_event = BUTTON_EVENT_1_3DUAL_PRESS;
    }
  }
  else if(button_event == BUTTON_EVENT_3_SHORT_PRESS || button_event == BUTTON_EVENT_3_LONG_PRESS)
  {
    if(BUTTON_TIME_LIMIT < button_1_press_cnt)
    {
       button_1_press_cnt = 0;
       button_3_press_cnt = 0;
       button_event = BUTTON_EVENT_1_3DUAL_PRESS;
    }
  }
  return button_event;
}

void loop()
{
  BUTTON_EVENT button_event = GetButtonEvent();
  if(button_event != BUTTON_EVENT_NO_PRESS)
  {
    switch(button_event)
    {
      case BUTTON_EVENT_1_SHORT_PRESS:
            Serial.println("BUTTON_EVENT_1_SHORT_PRESS");
      break;
      case BUTTON_EVENT_2_SHORT_PRESS:
            Serial.println("BUTTON_EVENT_2_SHORT_PRESS");
      break;
      case BUTTON_EVENT_3_SHORT_PRESS:
            Serial.println("BUTTON_EVENT_3_SHORT_PRESS");
      break;
      case BUTTON_EVENT_1_LONG_PRESS:
            Serial.println("BUTTON_EVENT_1_LONG_PRESS");
      break;
      case BUTTON_EVENT_2_LONG_PRESS:
            Serial.println("BUTTON_EVENT_2_LONG_PRESS");
      break;
      case BUTTON_EVENT_3_LONG_PRESS:
            Serial.println("BUTTON_EVENT_3_LONG_PRESS");
      break;
      case BUTTON_EVENT_1_3DUAL_PRESS:
            Serial.println("BUTTON_EVENT_1_3DUAL_PRESS");
      break;
      default:
            Serial.println("Detected Event : Invalid");
      break;
    }
  }
}

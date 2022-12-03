#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// Assign pins of ATTiny841, make sure it runs at 16 MHz internal oscillator

//DigitalPin Assignments
const unsigned int pwm1 = PIN_PA5; //PWM out on OC1A/PCINT5/Pin8
const unsigned int pwm2 = PIN_PA4; //PWM out on OC1B/PCINT4/Pin9
const unsigned int pwm3 = PIN_PA3; //PWM out on OC2B/PCINT3/Pin10


//Analog Pin Assignments
const unsigned int rate = PIN_PA1; //This will be an analog read of speed pot 1 on ADC0/PCINT0/Pin13
const unsigned int depth = PIN_PA0; // This will be an analog read of the depth pot


// Initialize UI stuff
const unsigned int checkInterval = 200; // Only check the pots/switches ever so many ms
unsigned long lastInterval;
float currDepth; // This is the value of the depth pot

float rateTime; // Current time value compared to the previous time to see if we need to move to the next table entry.
unsigned int prevRateVal; // Used to determine if we have changed rate pot enough that tap tempo should no longer be used
unsigned long rateStep; // The amount of time between each pwm step
unsigned long lastTime; // Used for keeping track of whether we move to the next entry in our sineTable or not
unsigned long maxTime = 2e6; // Max time of 2s
unsigned long minTime = 50e3; // Min time of 100 ms


// Initialize bookkeeping stuff
const uint8_t tableLength = 255; //Number of entries in our tables below
uint8_t inx1 = 0; //Index to read out of the table for PWM1, start at the beginning
uint8_t inx2 = 85; //Index to read out of the table for PWM2, start 120 degrees phase shift from PWM1
uint8_t inx3 = 170; //Index to read out of the table for PWM3, start 240 degrees phase shift from PWM1
int dutyCycle1; //PWM Duty cycle for PWM1
int dutyCycle2; //PWM Duty cycle for PWM2
int dutyCycle3; //PWM Duty cycle for PWM3


// Create a table for the PWM wave. Values are for 8 bit PWM (max value 255).
// Put it in flash memory so that it doesn't eat up our dynamic SRAM
const uint16_t waveTable[] PROGMEM = {512,525,537,550,562,575,587,600,612,625,637,649,661,673,685,697,709,720,732,743,754,765,776,787,797,808,818,828,838,848,857,866,875,884,892,901,909,917,924,932,939,946,952,959,965,970,976,981,986,991,995,999,1003,1006,1009,1012,1015,1017,1019,1020,1022,1023,1024,1024,1024,1024,1023,1022,1021,1020,1018,1016,1013,1011,1008,1004,1001,997,993,988,984,978,973,967,962,955,949,942,935,928,921,913,905,897,
888,880,871,862,852,843,833,823,813,803,792,782,771,760,749,737,726,714,703,691,679,667,655,643,631,618,606,594,581,569,556,544,531,518,506,493,480,468,455,443,430,418,406,393,381,369,357,345,333,321,310,298,287,275,264,253,242,232,221,211,201,191,181,172,162,153,144,136,127,119,111,103,96,89,82,75,69,62,57,51,46,40,36,31,27,23,20,16,13,11,8,6,4,3,2,1,0,0,0,0,1,2,4,5,7,9,12,15,18,21,25,29,33,38,43,48,54,59,65,72,78,85,92,100,107,115,123,
132,140,149,158,167,176,186,196,206,216,227,237,248,259,270,281,292,304,315,327,339,351,363,375,387,399,412,424,437,449,462,474,487,499,512};

//Wave tables that can be copied/pasted into waveTable to change the LFO waveform
/*
//Sine
const uint16_t sineTable[] PROGMEM = {512,525,537,550,562,575,587,600,612,625,637,649,661,673,685,697,709,720,732,743,754,765,776,787,797,808,818,828,838,848,857,866,875,884,892,901,909,917,924,932,939,946,952,959,965,970,976,981,986,991,995,999,1003,1006,1009,1012,1015,1017,1019,1020,1022,1023,1024,1024,1024,1024,1023,1022,1021,1020,1018,1016,1013,1011,1008,1004,1001,997,993,988,984,978,973,967,962,955,949,942,935,928,921,913,905,897,
888,880,871,862,852,843,833,823,813,803,792,782,771,760,749,737,726,714,703,691,679,667,655,643,631,618,606,594,581,569,556,544,531,518,506,493,480,468,455,443,430,418,406,393,381,369,357,345,333,321,310,298,287,275,264,253,242,232,221,211,201,191,181,172,162,153,144,136,127,119,111,103,96,89,82,75,69,62,57,51,46,40,36,31,27,23,20,16,13,11,8,6,4,3,2,1,0,0,0,0,1,2,4,5,7,9,12,15,18,21,25,29,33,38,43,48,54,59,65,72,78,85,92,100,107,115,123,
132,140,149,158,167,176,186,196,206,216,227,237,248,259,270,281,292,304,315,327,339,351,363,375,387,399,412,424,437,449,462,474,487,499,512};

const uint8_t risingSawTable[] PROGMEM = {0,4,8,12,16,20,24,28,32,36,40,44,48,52,56,60,64,68,72,77,81,85,89,93,97,101,105,109,113,117,121,125,129,133,137,141,145,149,153,157,161,165,169,173,177,181,185,189,193,197,201,205,209,213,217,222,226,230,234,238,242,246,250,254,258,262,266,270,274,278,282,286,290,294,298,302,306,310,314,318,322,326,330,334,338,342,346,350,354,358,362,367,371,375,379,383,387,391,395,399,403,407,411,415,419,423,427,
431,435,439,443,447,451,455,459,463,467,471,475,479,483,487,491,495,499,503,507,512,516,520,524,528,532,536,540,544,548,552,556,560,564,568,572,576,580,584,588,592,596,600,604,608,612,616,620,624,628,632,636,640,644,648,652,656,661,665,669,673,677,681,685,689,693,697,701,705,709,713,717,721,725,729,733,737,741,745,749,753,757,761,765,769,773,777,781,785,789,793,797,801,806,810,814,818,822,826,830,834,838,842,846,850,854,858,862,866,870,874,
878,882,886,890,894,898,902,906,910,914,918,922,926,930,934,938,942,946,951,955,959,963,967,971,975,979,983,987,991,995,999,1003,1007,1011,1015,1019,1023};


const uint8_t fallingSawTable[] PROGMEM = {1023,1019,1015,1011,1007,1003,999,995,991,987,983,979,975,971,967,963,959,955,951,946,942,938,934,930,926,922,918,914,910,906,902,898,894,890,886,882,878,874,870,866,862,858,854,850,846,842,838,834,830,826,822,818,814,810,806,801,797,793,789,785,781,777,773,769,765,761,757,753,749,745,741,737,733,729,725,721,717,713,709,705,701,697,693,689,685,681,677,673,669,665,661,656,652,648,644,640,636,632,
628,624,620,616,612,608,604,600,596,592,588,584,580,576,572,568,564,560,556,552,548,544,540,536,532,528,524,520,516,512,507,503,499,495,491,487,483,479,475,471,467,463,459,455,451,447,443,439,435,431,427,423,419,415,411,407,403,399,395,391,387,383,379,375,371,367,362,358,354,350,346,342,338,334,330,326,322,318,314,310,306,302,298,294,290,286,282,278,274,270,266,262,258,254,250,246,242,238,234,230,226,222,217,213,209,205,201,197,193,189,
185,181,177,173,169,165,161,157,153,149,145,141,137,133,129,125,121,117,113,109,105,101,97,93,89,85,81,77,72,68,64,60,56,52,48,44,40,36,32,28,24,20,16,12,8,4,0};


const uint8_t squareTable[] PROGMEM = {255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


const uint16_t triangleTable[] PROGMEM = {512,520,528,536,544,553,561,569,577,585,593,601,609,617,626,634,642,650,658,666,674,682,690,699,707,715,723,731,739,747,755,763,772,780,788,796,804,812,820,828,836,845,853,861,869,877,885,893,901,909,918,926,934,942,950,958,966,974,982,991,999,1007,1015,1023,1015,1007,999,991,983,975,967,959,951,943,935,927,919,911,903,895,887,879,871,863,855,847,839,831,823,815,807,799,791,783,775,767,759,751,
743,735,727,719,711,703,695,687,679,671,663,655,647,639,631,623,615,607,599,591,583,575,567,559,551,543,535,527,519,511,504,496,488,480,472,464,456,448,440,432,424,416,408,400,392,384,376,368,360,352,344,336,328,320,312,304,296,288,280,272,264,256,248,240,232,224,216,208,200,192,184,176,168,160,152,144,136,128,120,112,104,96,88,80,72,64,56,48,40,32,24,16,8,0,8,16,24,32,40,48,56,64,72,80,88,96,104,112,120,128,136,144,152,160,168,176,184,
192,200,208,216,224,232,240,248,256,264,272,280,288,296,304,312,320,328,336,344,352,360,368,376,384,392,400,408,416,424,432,440,448,456,464,472,480,488,496,504,512};
*/

// This table converts a linear taper speed pot to a logarithmically spaced array of pulse times. This is true logarithmic as opposed to a "log" pot and sounds much nicer
const int logTable[] PROGMEM = {1023,787,699,647,607,575,551,527,507,491,475,463,451,439,427,419,407,399,391,383,375,367,363,355,351,343,339,331,327,323,315,311,307,303,299,295,291,287,283,279,275,271,267,263,259,259,255,251,
247,247,243,239,235,235,231,227,227,223,219,219,215,215,211,207,207,203,203,199,199,195,191,191,187,187,183,183,179,179,179,175,175,171,171,167,167,163,163,163,159,159,155,155,151,151,151,147,147,147,143,143,139,139,139,135,
135,135,131,131,131,127,127,127,123,123,123,119,119,119,115,115,115,111,111,111,107,107,107,107,103,103,103,99,99,99,99,95,95,95,95,91,91,91,91,87,87,87,83,83,83,83,83,79,79,79,79,75,75,75,75,71,71,71,71,67,67,67,67,67,63,63,
63,63,59,59,59,59,59,55,55,55,55,55,51,51,51,51,51,47,47,47,47,47,43,43,43,43,43,39,39,39,39,39,39,35,35,35,35,35,31,31,31,31,31,31,27,27,27,27,27,27,23,23,23,23,23,23,19,19,19,19,19,19,15,15,15,15,15,15,15,11,11,11,11,11,11,
7,7,7,7,7,7,7,3,3,3,3};

void setup() {

  //Define pin modes
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(pwm3, OUTPUT);
  pinMode(rate, INPUT);
  pinMode(depth, INPUT);

  // Initialize pin states
  digitalWrite(pwm1,LOW);
  digitalWrite(pwm2, LOW);
  digitalWrite(pwm3, LOW);


  lastTime = micros(); // Get an initial value

  //Set up the timer outputs to the correct pins. Don't touch timer 0 so that millis() works correctly
  //Pin 8 -> PWM2 -> TOCC4 (make timer OC2B)
  //Pin 9 -> PWM1 -> TOCC3 (make timer OC2A)
  //Pin 10 -> PWM3 -> TOCC2 (make timer OC1B)
  TOCPMSA0 = (1<<TOCC2S0) | (1<<TOCC3S1); //This sets TOCC1 throught TOCC3
  TOCPMSA1 = (1<<TOCC4S1); //This sets TOCC4 to be OC2B
  TOCPMCOE = (1<<TOCC2OE) | (1<<TOCC3OE) | (1<<TOCC4OE); //Enable the time/output compare on TOCC1-4
  
  //Disable interrupts on timers
  TIMSK1 = 0;
  TIMSK2 = 0;
  
  //Set up 16 bit timers so that we can use PWM
  //PWM is 10-bit fast, 0x03FF TOP, no prescaler
  //Set to Fast, 10-bit PWM, max value of 1024
  TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11) | (1<<WGM10); //TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10) | _BV(WGM11);
  TCCR1B = (1<<WGM12) | (1<<CS10); //No prescaler,  | _BV(WGM12)
  TCCR1C = 0b00000000; //Set to zero when in PWM mode
  TCCR2A = (1<<COM2A1) | (1<<COM2B1) | (1<<WGM21) | (1<<WGM20); //TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20) | _BV(WGM21); //Set to Fast, 10-bit PWM, max value of 1024
  TCCR2B = (1<<WGM22) | (1<<CS20); //No prescaler,  | _BV(WGM22)
  TCCR2C = 0b00000000; //Set to zero when in PWM mode

  // Get initial UI state
  prevRateVal = 2000; // Set to a value impossible for the rate pot to reach to force an update on initialization
  checkRate();
  checkDepth();
  lastInterval = millis();
}

void loop() {
  // Check the UI stuff if it has been at least checkInterval ms since last check
  if (millis() - lastInterval >= checkInterval) {
    checkRate();
    checkDepth();
    lastInterval = millis();
  }

  // Update the PWM output
  updatePWM();
}




//Update PWM outputs based on above settings
void updatePWM() {
  //Compare current time to last time we updated for each PWM out
  if ((micros() - lastTime) > rateStep) {
    //We have met the time threshold for PWM, so go to the next value in the table
    float tableVal1 = pgm_read_word(waveTable + inx1);
    float tableVal2 = pgm_read_word(waveTable + inx2);
    float tableVal3 = pgm_read_word(waveTable + inx3);
    dutyCycle1 = round(tableVal1 * currDepth/1023);
    dutyCycle2 = round(tableVal2 * currDepth/1023);
    dutyCycle3 = round(tableVal3 * currDepth/1023);  
    
    // Set PWM's
    OCR2B = dutyCycle1;
    OCR2A = dutyCycle2;
    OCR1B = dutyCycle3;
    inx1 += 1; //Increment the read index for PWM
    inx2 += 1; //Increment the read index for PWM
    inx3 += 1; //Increment the read index for PWM
    
    // Go back to the beginning of the table if we have gotten to the end
    if (inx1 == tableLength) {
      inx1 = 0;
    }
    if (inx2 == tableLength) {
      inx2 = 0;
    }
    if (inx3 == tableLength) {
      inx3 = 0;
    }
    
    lastTime = micros();
  }//End of if ((currTime - lastTime) > rateStep)

}// end of updatePWM()





//Check what the rate pot is reading
void checkRate () {
  float logTableVal = 0.0;
  
  //Calculate out the required time steps to reach before we go to the next step in the table.
  int rateVal = analogRead(rate);

  // Only make a change if we have turned the knob a little. This is to prevent stomping on the time determined by tap tempo
  if (abs(rateVal - prevRateVal) >= 5) {
    //Find the closest value in the log table so we can use linear pots for logarithmic time spacing
    int logInx = round(rateVal/4);
    logTableVal = pgm_read_word(logTable + logInx);
    rateTime = maxTime * logTableVal/1023;

  
    // Ensure that the time isn't too short
    if (rateTime < minTime) {
      rateTime = minTime;
    }
    
    prevRateVal = rateVal; // Keep track of the knob position
    rateStep = round(rateTime/tableLength);
  }
}// End checkRate()


void checkDepth() {
  currDepth = analogRead(depth);
}






// Some helpful debug functions
/*
  void blinkLED(int numBlinks, int duration) {
  for (int i = 0; i < numBlinks; i++) {
    digitalWrite(pwm2, HIGH);
    delay(duration);
    digitalWrite(pwm2, LOW);
    delay(duration);
  }
  }

  void blinkDebugLED(int numBlinks, int duration) {
  for (int i = 0; i < numBlinks; i++) {
    digitalWrite(encSW, HIGH);
    delay(duration);
    digitalWrite(encSW, LOW);
    delay(duration);
  }
  }
  */

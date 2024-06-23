#include <Arduino.h>
#include "Adafruit_VL53L0X.h"
 
//Distance sensor
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Rotary encoder variables
#define ENC_A 51
#define ENC_B 50

unsigned long _lastIncReadTime = micros(); 
unsigned long _lastDecReadTime = micros(); 
int _pauseLength = 25000;
int _increment = 100;

// Variables for PID
double kp = 0.2;
double ki = 0.1;
double kd = 0.3;
double dt = 0.1;

double Actual = 0;
double SetPoint = 0;
double Error = 0;
double ErrorSum = 0;
double ErrorDiv = 0;
double PrevError = 0;
double StuurActie = 0;

volatile int counter = 100;

// Rotary encoder read function
void read_encoder() {
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent
 
  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(ENC_A)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(ENC_B)) old_AB |= 0x01; // Add current state of pin B
  
  encval += enc_states[( old_AB & 0x0f )];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) {        // Four steps forward
    int changevalue = _increment;
    counter = counter + changevalue;              // Update counter
    encval = 0;
  }
  else if( encval < -3 ) {        // Four steps backward
    int changevalue = -_increment;
    counter = counter + changevalue;              // Update counter
    encval = 0;
  }
} 

// Exponential moving average filter for the distance sensor
template <uint8_t K, class uint_t = uint16_t>
class EMA {
  public:
    /// Update the filter with the given input and return the filtered output.
    uint_t operator()(uint_t input) {
        state += input;
        uint_t output = (state + half) >> K;
        state -= output;
        return output;
    }

    static_assert(
        uint_t(0) < uint_t(-1),  // Check that `uint_t` is an unsigned type
        "The `uint_t` type should be an unsigned integer, otherwise, "
        "the division using bit shifts is invalid.");

    /// Fixed point representation of one half, used for rounding.
    constexpr static uint_t half = uint_t{1} << (K - 1);

  private:
    uint_t state = 0;
};
 
void setup() {  
    Serial.begin(115200);
    
    // wait until serial port opens for native USB devices
    while (! Serial) {
    delay(1);
    }
    
    //Check if the sensor is operational as the whole system won't work if it isn't
    Serial.println("Adafruit VL53L0X test");
    delay(100);
    if (!lox.begin(0x29,true)) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
    }
    // power
    Serial.println(F("VL53L0X is operational\n\n"));

    //Set up the pwm pin
    pinMode(9, OUTPUT);
    // Set encoder pins and attach interrupts
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A), read_encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), read_encoder, CHANGE);
}
 
const unsigned long interval = 10000; // 10000 µs = 100 Hz

void loop() {
    // Change the setpoint based on the rotary encoder
    if(counter != SetPoint){
        SetPoint = counter;
    }

    // Setup measuring on the distance sensor
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

    //Debug prints
    // if (measure.RangeStatus != 4) { // phase failures have incorrect data
    //     static EMA<2> filter;
    //     static unsigned long prevMicros = micros() - interval;
    //     if (micros() - prevMicros >= interval) {
    //         int rawValue = measure.RangeMilliMeter;
    //         int filteredValue = filter(rawValue);
    //         Serial.print(rawValue);
    //         Serial.print('\t');
    //         Serial.print(filteredValue);
    //         Serial.print('\t');
    //         Serial.print(lastCounter);
    //         Serial.print('\t');
    //         prevMicros += interval;
    //     }
    // } else {
    //     Serial.println(" out of range ");
    // }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 

    // Filter distance measurements
    static EMA<2> filter;
    Actual = filter(measure.RangeMilliMeter);

    // More debug prints
    // Serial.print(PrevError);
    // Serial.print('\t');

    //Serial.println(SetPoint);

    // Pid controller
    // I know it's supposed to be setpoints - actual but since the sensor is at the top I flipped them
    Error = Actual - SetPoint;
    ErrorSum = ErrorSum + Error * dt;
    ErrorDiv = (Error - PrevError) / dt;
    StuurActie = (kp * Error) + (ki * ErrorSum) + (kd * ErrorDiv);
    PrevError = Error;

    // Even more debug prints
    // Serial.print(Actual);
    // Serial.print('\t');

    // Serial.print(Error);
    // Serial.print('\t');

    // Serial.print(ErrorSum);
    // Serial.print('\t');

    // Serial.print(ErrorDiv);
    // Serial.print('\t');


    // Serial.print(StuurActie);
    // Serial.print('\t');

    // Map the stuuractie to the pwm values as the stuuractie can be much larger than the pwm signal allows.
    int newval = map(constrain(StuurActie,-500,500),-500,500,0,255); // map input 

    // Serial.println(newval);
    // Serial.print('\t');
    //Serial.println(lastCounter);
    
    // Write pwm signal to pwm pin to control the fan
    analogWrite(9,newval);
}
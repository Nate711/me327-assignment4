// Pin declares
int MOTOR_PWM_PIN = 5; // PWM output pin for motor 1
int MOTOR_DIR_PIN = 8; // direction output pin for motor 1
int POSITION_SENSOR_PIN = A2; // input pin for MR sensor

const double COUNTS_PER_ROTATION = 980;

// Position tracking variables
boolean flipped = false;        // Whether we've flipped
int raw_position = 0;           // current raw reading from MR sensor
int last_raw_position = 0;      // last raw reading from MR sensor
int last_last_raw_position = 0; // last last raw reading from MR sensor
int flips = 0;             // keeps track of the number of flips over the 180deg mark
int raw_difference = 0;
int last_raw_difference = 0;
int raw_offset = 0;
int last_raw_offset = 0;
const int FLIP_THRESHOLD = 700;     // threshold to determine whether or not a flip over the 180 degree mark occurred

void initialize_mr_sensor();
void initialize_motor();
double read_mr_sensor();
void command_motor(double Tp);
void set_pwm_frequency(int pin, int divisor);

void initialize_mr_sensor() {
  // Input pins
  pinMode(POSITION_SENSOR_PIN, INPUT); // set MR sensor pin to be an input
  
  // Initialize position valiables
  last_last_raw_position = analogRead(POSITION_SENSOR_PIN);
  last_raw_position = analogRead(POSITION_SENSOR_PIN);
  flips = 0;
}

void initialize_motor() {
  // Set PWM frequency 
  set_pwm_frequency(MOTOR_PWM_PIN, 1); 
  
  // Output pins
  pinMode(MOTOR_PWM_PIN, OUTPUT);  // PWM pin for motor A
  pinMode(MOTOR_DIR_PIN, OUTPUT);  // dir pin for motor A
  
  // Initialize motor 
  analogWrite(MOTOR_PWM_PIN, 0);     // set to not be spinning (0/255)
  digitalWrite(MOTOR_DIR_PIN, LOW);  // set direction
}

double read_mr_sensor() {
  // Get voltage output by MR sensor
  raw_position = analogRead(POSITION_SENSOR_PIN);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  raw_difference = raw_position - last_raw_position;              //difference btwn current raw position and last raw position
  last_raw_difference = raw_position - last_last_raw_position;    //difference btwn current raw position and last last raw position
  raw_offset = abs(raw_difference);
  last_raw_offset = abs(last_raw_difference);
  
  // Update position record-keeping vairables
  last_last_raw_position = last_raw_position;
  last_raw_position = raw_position;
  
  // Keep track of flips over 180 degrees
  if((last_raw_offset > FLIP_THRESHOLD) && (!flipped)) {  // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(last_raw_difference > 0) {                         // check to see which direction the drive wheel was turning
      flips--;                                            // cw rotation 
    } else {                                              
      flips++;                                            // ccw rotation
    }
    flipped = true;                                       // set boolean so that the next time through the loop won't trigger a flip
  } else {                                                // anytime no flip has occurred
    flipped = false;
  }
  return raw_position + flips * COUNTS_PER_ROTATION;
}

void command_motor(double Tp) {
  // Determine correct direction for motor torque
  if(Tp > 0) { 
    digitalWrite(MOTOR_DIR_PIN, HIGH);
  } else {
    digitalWrite(MOTOR_DIR_PIN, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  double duty = sqrt(abs(Tp)/0.0183);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {            
    duty = 1;
  } else if (duty < 0) { 
    duty = 0;
  }  
  unsigned int output = (int)(duty* 255);     // convert duty cycle to output signal
  analogWrite(MOTOR_PWM_PIN, output);                // output the signal
}

// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// --------------------------------------------------------------
void set_pwm_frequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

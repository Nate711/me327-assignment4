//--------------------------------------------------------------------------
// Code to test basic Hapkit functionality (sensing and force output)
// Updated by Allison Okamura 1.17.2018, Nathan Kau 9.9.2022
//--------------------------------------------------------------------------

// Includes
#include <math.h>
#include "helpers.h"

double calculate_handle_position(double updatedPos) {
  // Define kinematic parameters you may need
  //  double rh = 0.075; // handle radius [m]
  //  double m = 0.0128; // [deg/pos]
  //  double b = - 8.7113; // [deg]
  //  double ts = m * updatedPos + b;
  //  xh = rh * (ts * 3.14159 / 180); // handle position [m]
  //  Serial.println(xh);
  return 0.0;
}

double calculate_pulley_torque(double force) {
  //  double rs = 0.073152;   // sector radius [m]
  //  double rp = 0.004191;   // pulley radius [m]
  //  return rp / rs * rh * force; // [Nm]
  return 0.0;
}

double specify_force() {
  return 0.0;
}

void setup() 
{
  // Set up serial communication. MAKE SURE YOUR SERIAL MONITOR IS SET TO 115200 BAUD
  Serial.begin(115200);
  
  initialize_mr_sensor();
  initialize_motor();
}

void loop()
{
  // Compute position in counts (do not change)
  double updated_position = read_mr_sensor();

  // Compute position in meters
  double handle_position = calculate_handle_position(updated_position);
  
  // Assign a motor output force in Newtons
  // ADD YOUR CODE HERE
  double force = specify_force();
  double pulley_torque = calculate_pulley_torque(force);
  
  // Force output (do not change) 
  command_motor(pulley_torque);
  
}

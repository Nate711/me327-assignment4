//--------------------------------------------------------------------------
// Code to test basic Hapkit functionality (sensing and force output)
// Updated by Allison Okamura 1.17.2018, Nathan Kau 9.9.2022
//--------------------------------------------------------------------------

// Includes
#include <math.h>
#include "helpers.h"

/*
 * calculate_handle_position
 * 
 * Calculates the position of the hapkit handle given the sensor reading
 * 
 * Args:
 *  updated_position: Position reading from sensor
 * Return:
 *  Handle position in meters
 */
double calculate_handle_position(double updated_position) {
  // STUDENT CODE HERE

  // TODO: remove answer code
  // Define kinematic parameters you may need
  //  double rh = 0.075; // handle radius [m]
  //  double m = 0.0128; // [deg/pos]
  //  double b = - 8.7113; // [deg]
  //  double ts = m * updated_position + b;
  //  xh = rh * (ts * 3.14159 / 180); // handle position [m]
  //  Serial.println(xh);
  return 0.0;
}

/*
 * calculate_pulley_torque
 * 
 * Calculates the pulley torque necessary to achieve the desired force
 * 
 * Args:
 *  force: Desired force in Newtons
 * Return:
 *  Motor torque (in Newton-meters) to achieve desired force.
 */
double calculate_pulley_torque(double force) {
  // STUDENT CODE HERE
  
  // TODO: remove answer code
  //  double rs = 0.073152;   // sector radius [m]
  //  double rp = 0.004191;   // pulley radius [m]
  //  return rp / rs * rh * force; // [Nm]
  return 0.0;
}

/*
 * student_specified_force
 * 
 * Specifies a desired force to render on the hapkit
 * 
 * Args:
 *  None
 * Return:
 *  Desired force in Newtons
 */
double student_specified_force() {
  // STUDENT CODE HERE
  return 0.0;
}

void setup() 
{
  // Set up serial communication. MAKE SURE YOUR SERIAL MONITOR IS SET TO 115200 BAUD
  Serial.begin(115200);
  
  initialize_mr_sensor();
  initialize_motor();
}

int count=0;
void loop()
{
  // Compute position in counts
  double updated_position = read_mr_sensor();
  if(count % 10 == 0) Serial.println(updated_position);
  count++;

  // Compute position in meters
  double handle_position = calculate_handle_position(updated_position);
  
  // Assign a motor output force in Newtons
  double force = student_specified_force();
  double pulley_torque = calculate_pulley_torque(force);
  
  // Force output
  command_motor(pulley_torque);  
}

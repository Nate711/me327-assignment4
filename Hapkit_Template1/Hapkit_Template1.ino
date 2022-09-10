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
  return 0.0;
}

/*
 * student_specified_force
 * 
 * Specifies a desired force to render on the hapkit
 * 
 * Args:
 *  handle_position: Handle position in meters
 *  handle_velocity: Approximate handle velocity in meters/second
 * Return:
 *  Desired force in Newtons
 */
double student_specified_force(double handle_position, double handle_velocity) {
  // STUDENT CODE HERE
  return 0.0;
}

void setup() 
{
  // Set up serial communication. MAKE SURE YOUR SERIAL MONITOR IS SET TO 115200 BAUD
  Serial.begin(115200);
  
  initialize_mr_sensor();
  initialize_motor();
  initialize_loop_checker();
}

int count = 0;
void loop()
{
  // Compute position in counts
  double updated_position = read_mr_sensor();

  // Compute position [m] and velocity [m/s]
  double handle_position = calculate_handle_position(updated_position);
  double smoothed_velocity = calculate_smoothed_velocity(handle_position, /*DT=*/0.001);

  // Print out handle position and velocity every 10 loops
  if(count % 10 == 0) {
    Serial.print(handle_position, 3);
    Serial.print(" ");
    Serial.println(smoothed_velocity, 3);
  }
  count++;
  
  // Assign a motor output force [N]
  double force = student_specified_force(handle_position, smoothed_velocity);
  double pulley_torque = calculate_pulley_torque(force);
  
  // Command the motor
  command_motor(pulley_torque);

  // Check if your loop speed is too slow, and if so, print an error message.
  check_loop_speed();
}

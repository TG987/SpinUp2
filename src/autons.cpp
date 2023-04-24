#include "main.h"

/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////

const int DRIVE_SPEED = 110; // This is 110/127 (around 87% of max speed).  We don't suggest making this 127.
                             // If this is 127 and the robot tries to heading correct, it's only correcting by
                             // making one side slower.  When this is 87%, it's correcting by making one side
                             // faster and one side slower, giving better heading correction.
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

///
// Constants
///

// It's best practice to tune constants when the robot is empty and with heavier game objects, or with lifts up vs down.
// If the objects are light or the cog doesn't change much, then there isn't a concern here.

void default_constants()
{
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.35, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.28, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void exit_condition_defaults()
{
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 400, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}
void roller_optical(bool side = false) { 
  //we use two opticals so this checks which one to use through the tiernary operator
  float rollerColor = side ? optical_sensorRight.get_hue() : optical_sensorLeft.get_hue();
  //while the roller color is not red

  while((rollerColor > 50)) {
    //run the intake
    intake_indexer_mtr.move(250);
    //move backwards while running the intake
    chassis.set_drive_pid(-1, DRIVE_SPEED);
      chassis.wait_drive();
      //update the roller state
     rollerColor = side ? optical_sensorRight.get_hue() : optical_sensorLeft.get_hue();
     //wait to not overuse resources
     pros::delay(10);
  }
  //stop the intake
  intake_indexer_mtr.move(0);
}
void intakeSpin(int timeSpin, bool dir = false, int speed = 600)
{
  intake_indexer_mtr.tare_position();
  dir ? timeSpin * -1 : timeSpin;
  intake_indexer_mtr.move_absolute(timeSpin, speed);
  while (!((intake_indexer_mtr.get_position() < timeSpin + 50) && (intake_indexer_mtr.get_position() > timeSpin - 50)))
  {
    // Continue running this loop as long as the motor is not within +-5 units of its goal
    pros::delay(2);
  }
}
void align(){
  //find current value of distance sensor
double current = distance_sensor.get();

//if we are not already perfected aligned
  while (current > 95) {
  current = distance_sensor.get();
  chassis.set_tank(-30, -30);
  pros::lcd::print(3, "Distance: %d\n", current);
  }
    chassis.set_tank(0, 0);
 
}
void index_one(){
  int time_millis = pros::millis() + 300;
  while(line_tracker.get_value() > 2750 && pros::millis() < time_millis){
    intake_indexer_mtr.move(-100);
  }
  intake_indexer_mtr.move(0);
  
}
void Near_Side()
{

    flywheel.move_velocity(600);
   chassis.set_drive_pid(-3, DRIVE_SPEED);
  chassis.wait_drive();
 intakeSpin(625);
  intake_indexer_mtr.move_velocity(600);
   intakeLift.set_value(true);
  chassis.set_drive_pid(15.75, DRIVE_SPEED);
   chassis.wait_drive();
   chassis.set_swing_pid(ez::RIGHT_SWING, -14, SWING_SPEED);
  chassis.wait_drive();
    pros::delay(800);
  index_one();
  pros::delay(450);
 index_one();
    intake_indexer_mtr.move_velocity(400);
    intakeLift.set_value(false);
       pros::delay(2100);
   index_one();
  pros::delay(450);
index_one();
  pros::delay(450);
index_one();
  pros::delay(150);
    chassis.set_drive_pid(-10.75, DRIVE_SPEED);
     chassis.wait_drive();
          chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();
    intake_indexer_mtr.move_velocity(400);
      intakeLift.set_value(true);
          chassis.set_drive_pid(20.5, DRIVE_SPEED);
     chassis.wait_drive();
        intakeLift.set_value(false);
          pros::delay(1400);
          chassis.set_turn_pid(-24, TURN_SPEED);
  chassis.wait_drive();
      index_one();
  pros::delay(450);
 index_one();
  pros::delay(450);
index_one();
    flywheel.move_velocity(580);
      intake_indexer_mtr.move_velocity(600);
       chassis.set_drive_pid(9.5, DRIVE_SPEED);
     chassis.wait_drive();
         chassis.set_turn_pid(-22.5, TURN_SPEED);
  chassis.wait_drive();
         pros::delay(900);
index_one();
   pros::delay(450);
 index_one();

}

void AWP()
{
  // The first parameter is target degrees
  // The second parameter is max speed the robot will drive at
  flywheel.move_velocity(600);
  chassis.set_drive_pid(-3, DRIVE_SPEED);
  chassis.wait_drive();
  intakeSpin(625);  
  chassis.set_swing_pid(ez::LEFT_SWING, 45, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(27, DRIVE_SPEED, true);
  chassis.wait_until(22);
    intake_indexer_mtr.move_velocity(600);
  chassis.set_max_speed(40);
  chassis.wait_drive();
  chassis.set_max_speed(100);
  pros::delay(400);
  chassis.set_turn_pid(-25.5, TURN_SPEED);
  chassis.wait_drive();
  intake_indexer_mtr.move(0);
  ////////////////////////////////////
  // First Shots Fired
  //////////////////////////////
index_one();
  pros::delay(400);
index_one();
  pros::delay(400);
index_one();
  pros::delay(100);

  intake_indexer_mtr.move_velocity(600);
   flywheel.move_velocity(600);
  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(36, DRIVE_SPEED, true);
  chassis.wait_drive();
  pros::delay(600);

  chassis.set_turn_pid(-47, TURN_SPEED);
  chassis.wait_drive();
  intake_indexer_mtr.move(0);
index_one();
  pros::delay(400);
index_one();
  pros::delay(400);
index_one();
  pros::delay(100);
  intake_indexer_mtr.move_velocity(600);
    flywheel.move_velocity(600);
  chassis.set_turn_pid(43, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(67, DRIVE_SPEED, true);
  chassis.wait_drive();
  chassis.set_turn_pid(-83, TURN_SPEED);
  chassis.wait_drive();
index_one();
  pros::delay(800);
index_one();
pros::delay(100);
  align();
  intakeSpin(625);  
}

// Wait Until and Changing Max Speed
///
void Far_Side()
{ 
  flywheel.move_velocity(600);
     intake_indexer_mtr.move_velocity(600);
   chassis.set_drive_pid(28, DRIVE_SPEED);
  chassis.wait_drive();
   chassis.set_drive_pid(-8, DRIVE_SPEED);
  chassis.wait_drive();
   chassis.set_turn_pid(-66.5, TURN_SPEED);
     chassis.wait_drive();
        intake_indexer_mtr.move(0);
     align();
  
    intakeSpin(625);
    chassis.set_drive_pid(9, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(-58, TURN_SPEED);
     chassis.wait_drive();
      intakeLift.set_value(true);
          chassis.set_drive_pid(8, DRIVE_SPEED);
  chassis.wait_drive();
   index_one();
   pros::delay(450);
    index_one();
    pros::delay(450);
     index_one();
          intake_indexer_mtr.move_velocity(600);
         intakeLift.set_value(false);
  pros::delay(2000);
    index_one();
   pros::delay(450);
    index_one();
    pros::delay(450);
     index_one();
         pros::delay(100);
         intake_indexer_mtr.move_velocity(600);
  chassis.set_drive_pid(-12.5, DRIVE_SPEED);
  chassis.wait_drive();
   chassis.set_turn_pid(-110, TURN_SPEED);
     chassis.wait_drive();
           intake_indexer_mtr.move_velocity(600);
    chassis.set_drive_pid(63, 70, true);
  chassis.wait_drive();
   chassis.set_turn_pid(-25, TURN_SPEED);
     chassis.wait_drive();
   index_one();
   pros::delay(400);
    index_one();
    pros::delay(400);
     index_one();
}

///
// Auto that tests everything
///
void Skills(){
  optical_sensorLeft.set_led_pwm(100);
  optical_sensorRight.set_led_pwm(100);
  flywheel.move_velocity(470);
  angleChanger.set_value(true);
  align();
roller_optical();
  chassis.set_swing_pid(ez::RIGHT_SWING, -45, SWING_SPEED);
  chassis.wait_drive();
  intake_indexer_mtr.move_velocity(600);
  chassis.set_drive_pid(30, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(89, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(-17, DRIVE_SPEED);
  chassis.wait_drive();
  intake_indexer_mtr.move_velocity(0);
  align();
 roller_optical();
  chassis.set_swing_pid(ez::RIGHT_SWING, -3, SWING_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(104, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(5, TURN_SPEED);
  chassis.wait_drive();
  intake_indexer_mtr.tare_position();
  ////////////////////////////////////
  // First Shots Fired
  //////////////////////////////
  intakeSpin(-1800, true, 430);
   pros::delay(150);
    chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();
     intake_indexer_mtr.move_velocity(600);
   chassis.set_drive_pid(-20, DRIVE_SPEED);
  chassis.wait_drive();
   chassis.set_turn_pid(135, TURN_SPEED);
  chassis.wait_drive();
  flywheel.move_velocity(480);
  angleChanger.set_value(false);
  chassis.set_drive_pid(53, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();
  intake_indexer_mtr.move_velocity(600);
  chassis.set_drive_pid(57, DRIVE_SPEED);
  chassis.wait_drive();
       pros::delay(525);
  chassis.set_turn_pid(-49, TURN_SPEED);
  chassis.wait_drive();
  intake_indexer_mtr.move_velocity(0);
    intakeSpin(-350, true);
  intake_indexer_mtr.tare_position();
  pros::delay(400);
  intakeSpin(-450, true);
  intake_indexer_mtr.tare_position();
  pros::delay(400);
  intakeSpin(-1200, true);
  pros::delay(100);

    flywheel.move_velocity(470);
  angleChanger.set_value(true);
    chassis.set_drive_pid(5, DRIVE_SPEED);
  chassis.wait_drive();

    chassis.set_turn_pid(49, TURN_SPEED);
  chassis.wait_drive();
    chassis.set_drive_pid(55, 100);
  chassis.wait_drive();
    intake_indexer_mtr.move_velocity(600);
    chassis.set_drive_pid(54, 50);
  chassis.wait_drive();
  
  chassis.set_turn_pid(180, TURN_SPEED);
  chassis.wait_drive();
  
   chassis.set_drive_pid(-5, DRIVE_SPEED);
  chassis.wait_drive();
  intake_indexer_mtr.move(0);
  align();
 roller_optical();
  chassis.set_swing_pid(ez::LEFT_SWING, 274, SWING_SPEED);
  chassis.wait_drive();
    chassis.set_drive_pid(82, DRIVE_SPEED);
  chassis.wait_drive();
    chassis.set_turn_pid(253, TURN_SPEED);
  chassis.wait_drive();
    intakeSpin(-1800, true, 430);
  pros::delay(150);
   chassis.set_turn_pid(270, TURN_SPEED);
  chassis.wait_drive();
      chassis.set_drive_pid(-20, DRIVE_SPEED);
  chassis.wait_drive();
    chassis.set_turn_pid(135, TURN_SPEED);
  chassis.wait_drive();
     chassis.set_drive_pid(49, DRIVE_SPEED);
  chassis.wait_drive();
   chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();

   intake_indexer_mtr.move_velocity(600);
   chassis.set_drive_pid(72, 80);
  chassis.wait_drive();
    chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();

  align();
  roller_optical(true);
    chassis.set_swing_pid(ez::RIGHT_SWING, -184.5, SWING_SPEED);
  chassis.wait_drive();

    chassis.set_drive_pid(91.5, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(-173, TURN_SPEED);
  chassis.wait_drive();
  intake_indexer_mtr.tare_position();

  intakeSpin(-1800, true, 430);
   
  pros::delay(150);
   chassis.set_turn_pid(-180, TURN_SPEED);
  chassis.wait_drive();
     intake_indexer_mtr.move_velocity(600);
   chassis.set_drive_pid(-25, DRIVE_SPEED);
  chassis.wait_drive();
   chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();
  flywheel.move_velocity(480);
  angleChanger.set_value(false);
  chassis.set_drive_pid(53.5, DRIVE_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(-135, TURN_SPEED);
  chassis.wait_drive();
  intake_indexer_mtr.move_velocity(600);
  chassis.set_drive_pid(50, DRIVE_SPEED);
  chassis.wait_drive();
    pros::delay(400);
  chassis.set_turn_pid(-223, TURN_SPEED);
  chassis.wait_drive();

  intake_indexer_mtr.move_velocity(0);
    intakeSpin(-350, true);
  pros::delay(350);
  intakeSpin(-450, true);
  pros::delay(350);
  intakeSpin(-800, true);
  pros::delay(100);
    flywheel.move_velocity(510);
    chassis.set_turn_pid(-135, TURN_SPEED);
  chassis.wait_drive();
 
    chassis.set_drive_pid(55, 90);
  chassis.wait_drive();
     intake_indexer_mtr.move_velocity(600);
    chassis.set_drive_pid(42, 50);
  chassis.wait_drive();
  pros::delay(600);
      chassis.set_turn_pid(-271.5, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_drive_pid(5, DRIVE_SPEED);
  chassis.wait_drive();
    intakeSpin(-350, true);
  pros::delay(350);
  intakeSpin(-450, true);
  pros::delay(350);
  intakeSpin(-1200, true);
    flywheel.move_velocity(540);
        intake_indexer_mtr.move_velocity(0);
  chassis.set_turn_pid(-54, TURN_SPEED);
  chassis.wait_drive();
     chassis.set_drive_pid(43, DRIVE_SPEED);
     chassis.wait_drive();
    intake_indexer_mtr.move_velocity(600);
    chassis.set_drive_pid(30, 60);
  chassis.wait_drive();
    pros::delay(600);
  chassis.set_turn_pid(-3, TURN_SPEED);
  chassis.wait_drive();
  pros::delay(350);
     intakeSpin(-350, true);
  pros::delay(350);
  intakeSpin(-450, true);
  pros::delay(350);
  intakeSpin(-600, true);
    pros::delay(100);
      chassis.set_drive_pid(-50, DRIVE_SPEED);
  chassis.wait_drive();
    chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();
    chassis.set_drive_pid(3, DRIVE_SPEED);
  chassis.wait_drive();
 expansion.set_value(true);
 
}

void combining_movements()
{
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::RIGHT_SWING, -45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();
}
void swing_example(){
   optical_sensorLeft.set_led_pwm(100);
  optical_sensorRight.set_led_pwm(100);
  align();
  roller_optical();
}
///
// Interference example
///
void tug(int attempts)
{
  for (int i = 0; i < attempts - 1; i++)
  {
    // Attempt to drive backwards
    printf("i - %i", i);
    chassis.set_drive_pid(-12, 127);
    chassis.wait_drive();

    // If failsafed...
    if (chassis.interfered)
    {
      chassis.reset_drive_sensor();
      chassis.set_drive_pid(-2, 20);
      pros::delay(1000);
    }
    // If robot successfully drove back, return
    else
    {
      return;
    }
  }
}

// If there is no interference, robot will drive forward and turn 90 degrees.
// If interfered, robot will drive forward and then attempt to drive backwards.
void interfered_example()
{
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  if (chassis.interfered)
  {
    tug(3);
    return;
  }

  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
}

// . . .
// Make your own autonomous functions here!
// . . .
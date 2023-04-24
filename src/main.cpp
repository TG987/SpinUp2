#include "main.h"
pros::ADIDigitalOut expansion('A');
pros::ADIDigitalOut angleChanger('B');
pros::ADIDigitalOut intakeLift('C');
pros::Motor intake_indexer_mtr(10, MOTOR_GEARSET_6, false, MOTOR_ENCODER_DEGREES);
pros::Motor flywheel(1, MOTOR_GEARSET_6,false, MOTOR_ENCODER_DEGREES);
pros::Imu imu_sensor(6);
pros::Optical optical_sensorRight(3);
pros::Optical optical_sensorLeft(5);
pros::Distance distance_sensor(16);
pros::ADIAnalogIn line_tracker('D');
pros::Controller master(CONTROLLER_MASTER);

Drive chassis(
    // Left Chassis Ports (negative port will reverse it!)
    {-11, -12, -13}

    // Right Chassis Ports (negative port will reverse it!)
    //   the first port is the sensored port (when trackers are not used!)
    ,

    {18, 19, 20}

    // IMU Port
    ,
    9

    // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
    //    (or tracking wheel diameter)
    ,
    3.25

    // Cartridge RPM
    //   (or tick per rotation if using tracking wheels)
    ,
    600

    // External Gear Ratio (MUST BE DECIMAL)
    //    (or gear ratio of tracking wheel)
    // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
    // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
    ,
    1.25

    // Uncomment if using tracking wheels
    /*
    // Left Tracking Wheel Ports (negative port will reverse it!)
    // ,{1, 2} // 3 wire encoder
    // ,8 // Rotation sensor

    // Right Tracking Wheel Ports (negative port will reverse it!)
    // ,{-3, -4} // 3 wire encoder
    // ,-9 // Rotation sensor
    */

    // Uncomment if tracking wheels are plugged into a 3 wire expander
    // 3 Wire Port Expander Smart Port
    // ,1
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
  // Print our branding over your terminal :D
  ez::print_ez_template();
  flywheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake_indexer_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  pros::delay(500); // Stop the user from doing anything while legacy ports configure.
  pros::lcd::initialize();
  pros::lcd::print(3, "Distance: %d\n", distance_sensor.get());
  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(false); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0);                     // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(0, 0);                   // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)
  default_constants();                               // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults();                         // Set the exit conditions to your own constants from autons.cpp!

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
      Auton("Skills BONKASSS", Skills),
      Auton("AWP\n\n8 discs & 2 Rollers", AWP),
      Auton("Far Side\n\n9 Discs & Roller", Far_Side),
      Auton("Near Side\n\n9 Discs & Roller ", Near_Side),
      Auton("Swing Example\n\nSwing, drive, swing.", swing_example),
      Auton("Combine all 3 movements", combining_movements),
      Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", Far_Side),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()
{
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous()
{
  chassis.reset_pid_targets();               // Resets PID targets to 0
  chassis.reset_gyro();                      // Reset gyro position to 0
  chassis.reset_drive_sensor();              // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

bool intakeDir = true;
bool intakeToggle = true;
int flySpeed = 0;
int intakeSpeed = -600;
void IntakeCtrl()
{
  if(master.get_digital(DIGITAL_L2) && master.get_digital(DIGITAL_R1)){
    intakeLift.set_value(true);
         intakeDir = false;
 intake_indexer_mtr.move_velocity(600);
  }
    else if(master.get_digital(DIGITAL_L1) && master.get_digital(DIGITAL_R1)){
    intakeLift.set_value(false);
         intakeDir = false;
 intake_indexer_mtr.move_velocity(600);
  } else if(master.get_digital(DIGITAL_R2) && master.get_digital(DIGITAL_R1)){
     flySpeed = 530;
    angleChanger.set_value(false);
     intakeSpeed = -200;
       
  }
  else if (master.get_digital(DIGITAL_L2))
  {
    if (intakeToggle){
          intakeDir = false;
    intakeToggle = false;
 intake_indexer_mtr.move_velocity(600);

    }
    else{
          intakeToggle = true;
            intakeDir = true;
    }
     pros::delay(200);
  }
  else if (master.get_digital(DIGITAL_L1))
  {
    intake_indexer_mtr.move_velocity(intakeSpeed);
    intakeDir = true;
  }
    else if (master.get_digital(DIGITAL_R2))
  {
    intakeSpeed = -600;
      flySpeed = 480;
    angleChanger.set_value(true);
  }
  
  else if (master.get_digital(DIGITAL_B))
  {
    flySpeed = 0;
  }
    else if (master.get_digital(DIGITAL_A))
  {
    flySpeed = 470;
    angleChanger.set_value(true);
  }
    else if (master.get_digital(DIGITAL_Y))
  {
    flySpeed = 460;
    angleChanger.set_value(true);
  }
   else if (master.get_digital(DIGITAL_A))
  {
    flySpeed = 470;
    angleChanger.set_value(true);
  }
   else if (master.get_digital(DIGITAL_LEFT))
  {
    flySpeed -= 5;
  
  }
   else if (master.get_digital(DIGITAL_RIGHT))
  {
    flySpeed += 5;

  }
  else {
    if (intakeDir)
    {
      intake_indexer_mtr.move(0);
    }
  }
    flywheel.move_velocity(flySpeed);
}
void opcontrol()
{
  // This is preference to what you like to drive on.
  chassis.set_drive_brake(MOTOR_BRAKE_COAST);
  bool expand = true;
  while (true)
  {
    pros::lcd::print(3, "Distance: %d\n", distance_sensor.get());
    //FlyPID();
    chassis.tank(); // Tank control
    IntakeCtrl();
     if (master.get_digital(DIGITAL_R2) && master.get_digital(DIGITAL_L1) && master.get_digital(DIGITAL_L2) && master.get_digital(DIGITAL_R1)) {
      if (expand == true){
   expansion.set_value(true);
    expand = false;
      }
  else{
expansion.set_value(false);
    expand = true;
  }
  pros::delay(500);
    }

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}

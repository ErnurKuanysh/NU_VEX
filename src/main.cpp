#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "math.h"
#include "pros/misc.h"
using namespace pros;

pros::MotorGroup left_mg({-9, -20});
pros::MotorGroup right_mg({1, 11});
pros::Motor door(19);

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::ADIDigitalIn Button(8);

lemlib::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor group
                              6.417, // 10 inch track width
                              lemlib::Omniwheel::NEW_2, // using new 4" omnis
                              300, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

pros::Imu imu(4);

pros::Rotation horizontal_encoder(3);
pros::Rotation vertical_encoderl(-6);
pros::Rotation vertical_encoderr(-5);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -0.41);
lemlib::TrackingWheel vertical_tracking_wheell(&vertical_encoderl, lemlib::Omniwheel::NEW_2, -3.43);
lemlib::TrackingWheel vertical_tracking_wheelr(&vertical_encoderr, lemlib::Omniwheel::NEW_2, 3.43);

// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheell, // vertical tracking wheel 1, set to null
                            vertical_tracking_wheelr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr // inertial sensor
);

lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              0.4, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              1.2, // large error range, in inches
                                              300, // large error range timeout, in milliseconds
                                              50 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              0.5, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              1.5, // large error range, in degrees
                                              300, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);


bool DoorBool = false;

void doorloop(){
    door.set_brake_mode(pros::MotorBrake::brake);
    door.move_velocity(-70);
    delay(150);
    while(true){
        if(abs(door.get_actual_velocity()) < 1){
            break;
        }
    }
	door.tare_position();
	while(true){
		if (DoorBool) {
			door.move_absolute(100, 100);
		} else {
			door.move_absolute(6, 100);
		}
		pros::delay(20);
	}
}

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3,DoorBool == false? "Door Open":"Door Closed");
            pros::lcd::print(4,Button.get_value()?"pressed":"notpresed");
            
            // delay to save resources
            pros::delay(50);
        }
    });
    pros::delay(300);
	chassis.setPose(0,0,0);
	pros::Task DoorLoop(doorloop);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	// chassis.turnToHeading(90, 10000);
}

void Default(){
    chassis.moveToPoint(0, 20, 1200,{});
    chassis.waitUntil(18);
    DoorBool = true;
    chassis.turnToHeading(-90, 900);
    DoorBool = false;
    chassis.moveToPoint(-11, 20, 800,{.maxSpeed = 60});
    chassis.waitUntil(10);
    DoorBool = true;
    chassis.turnToHeading(90, 1400);
    chassis.moveToPoint(15, 20, 1000,{.maxSpeed = 80,.minSpeed = 60});
    chassis.waitUntil(5);
    DoorBool = false;
    chassis.waitUntil(21);
    DoorBool = true;
    chassis.turnToHeading(0,600);
}

int isRight = 1;
void DiogonalMid(){
    chassis.moveToPoint(isRight*0, 10, 500,{.minSpeed = 50});
    chassis.swingToPoint(isRight*-12, 20, isRight == 1 ? lemlib::DriveSide::LEFT : lemlib::DriveSide::RIGHT, 500,{.minSpeed = 50});
    chassis.moveToPoint(isRight*-11, 19, 700);
    chassis.waitUntilDone();
    DoorBool = true;
    chassis.turnToHeading(isRight*90, 600);
    chassis.waitUntilDone();
    DoorBool = false;
    chassis.moveToPoint(isRight*10.5, 19, 1000,{.maxSpeed = 95,.minSpeed = 90});
    chassis.waitUntilDone();
    DoorBool = true;
    chassis.turnToPoint(isRight*-10, 52, 1000);
    chassis.moveToPoint(isRight*0, 52, 1400,{.minSpeed = 90});
    // chassis.moveToPose(0, 52, 0, 1400);
    chassis.waitUntilDone();
    DoorBool = false;
    pros::delay(500);
    // chassis.moveToPoint(0, 40, 400,{.forwards = false});
    // chassis.moveToPoint(0, 60, 300);

    // DoorBool= true;
}

void SideObhod(){
    chassis.moveToPoint(isRight*0, 10, 500,{.minSpeed = 50});
    chassis.swingToPoint(isRight*-12, 20, isRight == 1 ? lemlib::DriveSide::LEFT : lemlib::DriveSide::RIGHT, 500,{.minSpeed = 50});
    chassis.moveToPoint(isRight*-11, 19, 700);
    chassis.waitUntilDone();
    DoorBool = true;
    chassis.turnToHeading(isRight*90, 600);
    chassis.waitUntilDone();
    DoorBool = false;
    chassis.moveToPoint(isRight*10.5, 19, 1000,{.maxSpeed = 95,.minSpeed = 90});
    chassis.waitUntilDone();
    DoorBool = true;
    chassis.turnToHeading(0, 700);
    chassis.waitUntilDone();
    // pros::delay(9000);
    chassis.moveToPoint(isRight*10.5, 38.5, 2000,{.minSpeed = 90});
    chassis.swingToPoint(0, 46, isRight == 1 ? lemlib::DriveSide::LEFT : lemlib::DriveSide::RIGHT, 700,{.minSpeed = 90});
    chassis.moveToPoint(0, 46, 1000,{.minSpeed = 90});
    chassis.turnToHeading(0, 600);
    chassis.waitUntilDone();
    DoorBool = false;
    chassis.moveToPoint(0, 65, 190,{.minSpeed = 120});
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.moveToPoint(0, 40, 400,{.forwards = false});
}

void Slow(){
    chassis.moveToPoint(isRight*0, 10, 500,{.minSpeed = 50});
    chassis.swingToPoint(isRight*-12, 20, isRight == 1 ? lemlib::DriveSide::LEFT : lemlib::DriveSide::RIGHT, 500,{.minSpeed = 50});
    chassis.moveToPoint(isRight*-11, 19, 700);
    chassis.waitUntilDone();
    DoorBool = true;
    chassis.turnToHeading(isRight*90, 600);
    chassis.waitUntilDone();
    DoorBool = false;
    chassis.moveToPoint(isRight*10.5, 19, 1000,{.maxSpeed = 95,.minSpeed = 90});
    chassis.waitUntilDone();
    DoorBool = true;
    chassis.turnToHeading(0, 700);
    chassis.waitUntilDone();
    // pros::delay(9000);
    chassis.moveToPoint(isRight*10.5, 38.5, 5000,{.maxSpeed = 40});
    chassis.swingToPoint(0, 46, isRight == 1 ? lemlib::DriveSide::LEFT : lemlib::DriveSide::RIGHT, 2000,{.maxSpeed = 40});
    chassis.moveToPoint(0, 46, 3000,{.maxSpeed = 40});
    chassis.turnToHeading(0, 600);
    chassis.waitUntilDone();
    DoorBool = false;
    chassis.moveToPoint(0, 65, 190,{.minSpeed = 120});
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.moveToPoint(0, 40, 400,{.forwards = false});
}

void opcontrol() {
    // while(true){
    //     if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
    //         isRight = 1;
    //         break;
    //     }
    //     if(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
    //         isRight = -1;
    //         break;
    //     }
    //     pros::delay(10);
    // }
    // while(true){
    //     if(Button.get_value() == 1){
    //         break;
    //     }
    // }
    // isRight = -1;

    // Slow();
    // SideObhod();
    // DiogonalMid();
    
	bool pressed = false;
	while (true) {
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            if(!pressed){
                pressed = true;
                DoorBool = !DoorBool;
            }
        }else{
            pressed = false;
		}
        // DoorBool = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
	
		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int strafe = master.get_analog(ANALOG_LEFT_X);  // Gets the turn left/right from right joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick

        left_mg.move(dir + turn);                      // Sets left motor voltage
		right_mg.move(dir - turn);                     // Sets right motor voltage
		pros::delay(50);                               // Run for 20 ms then update
	}
}

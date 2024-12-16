#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor rightFront = motor(PORT3, ratio18_1, false);
motor rightBack = motor(PORT20, ratio18_1, false);
motor rightMid = motor(PORT2, ratio18_1, true);
motor leftFront = motor(PORT21, ratio18_1, true);
motor leftBack = motor(PORT19, ratio18_1, true);
motor leftMid = motor(PORT4, ratio18_1, false);
controller Controller1 = controller(primary);
motor Intake = motor(PORT1, ratio36_1, true); 
inertial Inertial = inertial(PORT5);
digital_out MOGO = digital_out(Brain.ThreeWirePort.A);
distance detected = distance(PORT10);
motor Upper = motor(PORT20, ratio18_1, false);
digital_out mogo = digital_out(Brain.ThreeWirePort.B);
optical opticalsensor = optical(PORT12);
distance aligner = distance(PORT13);
motor WallStakes = motor(PORT7, ratio18_1, false); 
rotation rotationSensor = rotation(PORT13,false);
digital_out Doinker = digital_out(Brain.ThreeWirePort.E);
digital_out Puller = digital_out(Brain.ThreeWirePort.H);
digital_out sorter = digital_out(Brain.ThreeWirePort.C);


// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}
using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor rightFront;
extern motor rightBack;
extern motor rightMid;
extern motor leftFront;
extern motor leftBack;
extern motor leftMid;
extern controller Controller1;
extern motor Intake;
extern inertial Inertial;
extern digital_out MOGO;
extern distance detected;
extern motor Upper;
extern digital_out mogo;
extern optical opticalsensor;
extern distance aligner;
extern motor WallStakes;
extern rotation rotationSensor;
extern digital_out Doinker;
extern digital_out sorter;
extern digital_out Puller;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
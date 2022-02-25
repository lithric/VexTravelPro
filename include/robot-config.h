using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
extern controller Controller1;
extern smartdrive Drivetrain;
extern motor_group Arm;
extern signature Eyeball__SIG_1;
extern signature Eyeball__SIG_2;
extern signature Eyeball__SIG_3;
extern signature Eyeball__SIG_4;
extern signature Eyeball__SIG_5;
extern signature Eyeball__SIG_6;
extern signature Eyeball__SIG_7;
extern vision Eyeball;
extern motor_group Fork;
extern digital_out StickyPiston;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
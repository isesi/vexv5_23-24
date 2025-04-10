#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LF = motor(PORT1, ratio6_1, false);
motor LM = motor(PORT2, ratio6_1, false);
motor LB = motor(PORT3, ratio6_1, false);
motor LR = motor(PORT4, ratio6_1, false);
motor RF = motor(PORT6, ratio6_1, false);
motor RM = motor(PORT7, ratio6_1, false);
motor RB = motor(PORT8, ratio6_1, false);
motor RR = motor(PORT9, ratio6_1, false);
controller Controller1 = controller(primary);
motor cata = motor(PORT20, ratio36_1, false);
motor cata2 = motor(PORT19, ratio36_1, false);
pneumatics lower = pneumatics(Brain.ThreeWirePort.A);
pneumatics upper = pneumatics(Brain.ThreeWirePort.B);
//digital_out UpperPn = digital_out(Brain.ThreeWirePort.B);
//digital_out lowerpn = digital_out(Brain.ThreeWirePort.A);

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
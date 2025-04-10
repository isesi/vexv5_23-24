using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor LF;
extern motor LM;
extern motor LB;
extern motor RF;
extern motor RM;
extern motor RB;
extern motor LR;
extern motor RR;
extern controller Controller1;
extern motor cata;
extern motor cata2;
extern pneumatics upper;
extern pneumatics lower;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
// Linked data structure
struct State
{
    void (*cmdPtr)(void); // outout func
    unsigned long delay;  // time to delay in ms
    unsigned long next[8];
}; // Next if 3-bit input is 0-7
typedef const struct State StateType;
typedef StateType *StatePtr;
#define F 0 // forward
#define L 1 // off to the Left
#define R 2 // off to the Right
#define B 3 // Backward
#define irLeft 2
#define irCenter 3
#define irRight 4
uint8_t s;
StateType fsm[4] = {
    {&forward, 10, {F, R, B, B, L, R, B, B}}, // Center
    {&left, 10, {F, R, B, B, F, L, B, B}},    // off to the Left
    {&right, 10, {F, F, B, B, L, R, B, B}},   // off to the Right state, we need to turn left
    {&backward, 10, {F, R, B, B, L, L, B, B}}};

void setup()
{
    Serial.begin(9600);
    pinMode(irLeft, INPUT);
    pinMode(irCenter, INPUT);
    pinMode(irRight, INPUT);
    s = F; // state initialization
}
uint8_t irSensorInput()
{
    return digitalRead(irLeft) << 2 | digitalRead(irCenter) << 1 | digitalRead(irRight);
}
forward()
{
}
backward()
{
    if ((rand() % (1 - 0 + 1) + 0)) {
        left();
    } else {
        right();
        
    }
}
left()
{
}
right()
{
}
loop()
{
    uint8_t input;
    (fsm[s].cmdPtr)();  // run command based on states
    delay(fsm[s].delay);     // wait
    input = irSensorInput(); // read 3 sensors
    s = fsm[s].next[input];  // get next state depends on sensor readings and current state
}
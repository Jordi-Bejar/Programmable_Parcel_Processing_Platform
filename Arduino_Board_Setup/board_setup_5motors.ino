// Motor class to store pin configuration and control motor movement
class Motor {
public:
    int pulPin, dirPin, enaPin, sleepPin, resetPin;
    int maxSpeed; // Maximum delay in microseconds (lower value = faster speed)
    float stepDegrees;

    // Constructor
    Motor(int pul, int dir, int ena, int sleep, int reset,  int maxSpd, float stepDeg) {
        pulPin = pul;
        dirPin = dir;
        enaPin = ena;
        sleepPin = sleep;
        resetPin = reset;
        maxSpeed = maxSpd;
        stepDegrees = stepDeg;
    }

    // Initialize motor pins
    void begin() {
        pinMode(pulPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        pinMode(enaPin, OUTPUT);
        pinMode(sleepPin, OUTPUT);
        pinMode(resetPin, OUTPUT);

        digitalWrite(enaPin, LOW);   // Enable driver
        digitalWrite(sleepPin, HIGH); // Wake up driver
        digitalWrite(resetPin, HIGH); // Normal operation
    }
};

// Define motors
// To change the speed of each motor, adjust the sixth parameter
// The original number in the sixth parameter should be the minimum, you can go below these
Motor motor1(9, 8, 7, 51, 53, 400, 0.09);
Motor motor2(50, 49, 52, 51, 53, 600, 0.0353);
Motor motor3(48, 47, 46, 45, 44, 600, 0.067);
Motor motor4(43, 42, 41, 40, 39, 2000, 1.8);
Motor motor5(38, 37, 36, 35, 34, 1000, 1.8);


// Add more motors as needed
#define NUM_MOTORS 5
Motor* motors[NUM_MOTORS] = { &motor1, &motor2 , &motor3, &motor4, &motor5};

long steps[NUM_MOTORS];  // Steps remaining for each motor
int stepDelays[NUM_MOTORS];  // Delay between HIGH and LOW transitions
bool dirStates[NUM_MOTORS];  // Direction states for each motor
unsigned long lastStepTime[NUM_MOTORS] = {0};  // Time tracking
bool pulseState[NUM_MOTORS] = {false};  // Tracks if pin is HIGH or LOW

void setup() {
    Serial.begin(9600);
    Serial.println("Multi-Motor Control Ready. Enter 'motor_number angle speed'");

    // Initialize all motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i]->begin();
    }
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n'); 
        input.trim(); // Remove leading/trailing spaces

        Serial.print("Received: ");
        Serial.println(input);

        // Extract values
        int firstSpace = input.indexOf(' ');
        int secondSpace = input.indexOf(' ', firstSpace + 1);

        if (firstSpace == -1 || secondSpace == -1) {
            Serial.println("Invalid input format. Use: motor_number angle speed");
            return;
        }

        // Extract values as substrings and convert to numbers
        int motorNum = input.substring(0, firstSpace).toInt();
        int degrees = input.substring(firstSpace + 1, secondSpace).toInt();
        float speedScale = input.substring(secondSpace + 1).toFloat();  // Now should work correctly!
        // Validate motor number
        if (motorNum >= 1 && motorNum <= NUM_MOTORS) {
            // Ensure speed scale is within range
            if (speedScale < 0) speedScale = 0;
            if (speedScale > 1) speedScale = 1;

            // Compute movement parameters
            float stepDeg = motors[motorNum - 1]->stepDegrees;
            steps[motorNum - 1] = abs(degrees) / stepDeg; // Convert degrees to steps
            stepDelays[motorNum - 1] = motors[motorNum - 1]->maxSpeed/speedScale;
            dirStates[motorNum - 1] = (degrees > 0) ? LOW : HIGH;

            Serial.print("Motor ");
            Serial.print(motorNum);
            Serial.print(" moving: ");
            Serial.print(degrees);
            Serial.print(" degrees at speed scale ");
            Serial.println(speedScale);
            Serial.print(" delay");
            Serial.print(stepDelays[motorNum - 1]);

            // Set direction
            digitalWrite(motors[motorNum - 1]->dirPin, dirStates[motorNum - 1]);
        } else {
            Serial.println("Invalid motor number. Please enter 1 or 2.");
        }
    }
    // Move motors simultaneously
   unsigned long currentTime = micros();
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (steps[i] > 0 && (currentTime - lastStepTime[i] >= stepDelays[i])) {
            lastStepTime[i] = currentTime; // Update last step time

            if (pulseState[i] == false) {
                digitalWrite(motors[i]->pulPin, HIGH);
            } else {
                digitalWrite(motors[i]->pulPin, LOW);
                steps[i]--; // Only decrement steps when the full pulse cycle completes
            }

            pulseState[i] = !pulseState[i]; // Toggle pulse state
        }
    }
}

// Motor driver pins
const int RPWM = 9;  // Right PWM pin
const int LPWM = 10; // Left PWM pin
const int encoderPin = 2; // Encoder signal connected to Pin 2
volatile int pulseCount = 0;
unsigned long lastPulseTime = 0;

// Motor control variables
double setpoint = 100.0;  // Target speed, will change based on A0
double actualSpeed = 0.0;  // Measured speed (from encoder)

// Moving average variables for smoother control
const int NUM_READINGS = 20;  // Number of readings to average
int readings[NUM_READINGS];    // Array to store readings
int readIndex = 0;             // Index of the current reading
int total = 0;                 // Running total of the readings
int average = 0;               // The moving average

// Rate limiting variables
double lastOutput = 0;        // Previous motor output
double maxOutputChange = 10;  // Maximum change in output per loop (for smoothing)

// Function to smooth the setpoint
double smoothSetpoint(double newSetpoint) {
  static double smoothedSetpoint = newSetpoint;
  smoothedSetpoint += (newSetpoint - smoothedSetpoint) * 0.1;  // Apply smoothing factor (10% smoothing)
  return smoothedSetpoint;
}

void setup() {
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);

  // Initialize serial communication at 9600 bits per second:
  pinMode(encoderPin, INPUT_PULLUP); // Enable internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, RISING); // Count rising edges
  Serial.begin(9600);
  
  // Initialize the readings array
  for (int i = 0; i < NUM_READINGS; i++) {
    readings[i] = 0; // Set initial readings to 0
  }
}

void loop() {
  // Read analog values from pin A0 for setpoint
  int valA0 = analogRead(A0);

  // Subtract the last reading and subtract the new reading
  total -= readings[readIndex];
  readings[readIndex] = valA0;
  total += readings[readIndex];

  // Advance to the next index in the array
  readIndex = (readIndex + 1) % NUM_READINGS;

  // Calculate the average
  average = total / NUM_READINGS;

  // Map the average value (0-1023) to the setpoint range (adjust as needed)
  setpoint = map(average, 150, 165, 0, 255);  // Example: Maps to PWM range (0-255)

  // Apply smoothing to the setpoint to reduce jitter
  setpoint = smoothSetpoint(setpoint);
  


  // Measure actual motor speed
  actualSpeed = getMotorSpeed();  // Read speed from encoder

  // Calculate the error
  double error = setpoint - actualSpeed;

  // Apply rate limiting to the output (don't change too rapidly)
  double output = lastOutput + (error * 0.5);  // Smooth the output change by scaling the error

  // Limit the rate of change in the output
  output = constrain(output, lastOutput - maxOutputChange, lastOutput + maxOutputChange);
  
  // Apply constraints to the output to prevent exceeding PWM limits
  output = constrain(output, 0, 255);

  // Set motor speed based on the calculated output
  motorControl(output);

  // Update last output for next loop iteration
  lastOutput = output;

  // Debugging output
  Serial.print("Setpoint: ");
  Serial.print(setpoint);
  Serial.print(" Actual Speed: ");
  Serial.print(actualSpeed);
  Serial.print(" Output: ");
  Serial.println(output);

  delay(10);  // Short delay to allow continuous loop
}

// Interrupt function to count pulses
void countPulse() {
  pulseCount++;  // Increment pulse count on each interrupt (each pulse from encoder)
}

// Function to control motor speed
void motorControl(double output) {
  if (output > 0) {
    analogWrite(RPWM, output); // Forward direction
    analogWrite(LPWM, 0);      // Stop reverse direction
  } else {
    analogWrite(RPWM, 0);      // Stop forward direction
    analogWrite(LPWM, -output); // Reverse direction
  }
}

// Function to get motor speed (pulses per second)
double getMotorSpeed() {
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();
  
  // Continuously calculate speed, but update every 100ms for smooth control
  if (currentTime - lastTime >= 100) { // Update every 100ms
    double motorSpeed = (pulseCount * 10.0); // Pulses in 100ms (multiply by 10 to get per second)
    pulseCount = 0;  // Reset pulse count for the next interval
    lastTime = currentTime;  // Update the last time checked
    return motorSpeed;
  }
  return actualSpeed; // Return previous speed if it's not time to update yet
}

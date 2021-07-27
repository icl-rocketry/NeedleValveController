
const byte interruptPinA = 2; //Define two interrupt pins for the quadrature encoder
const byte interruptPinB = 3;
const byte analogPin = 5; //Define a pin for the input potentiometer

//Define pins for the L298N motor driver
const byte PWMPin = 5; 
const byte DirPin1 = 9;
const byte DirPin2 = 11;

const int RevolutionCounts = 5820; //Define number of encoder counts per revolution of motor output
const float NumRevolutions = 6; //Define number of revolutions of travel of the needle valve

volatile long EncoderCount = 0; //Initialise encoder count
long EncoderCount_prev = 0;
unsigned long t;
unsigned long t_prev = 0;

//Define position, demand position and time increment between control cycles
float Theta, Theta_d, dt;

volatile unsigned long count = 0;
unsigned long count_prev = 0;

//Initialise position error and integral error variables
float e, e_prev = 0, inte, inte_prev = 0;

const float Vmax = 18; //Define max and min voltage the motor can run at
const float Vmin = 13;
float V = 0; //Initialise the motor voltage

//Define the PID constants
float kp = 0.8;
float ki = 0;
float kd = 0.5;

//Timer Interrupt Service Routine for a change in Encoder A
void ISR_EncoderA(){
  bool PinA = digitalRead(interruptPinA);
  bool PinB = digitalRead(interruptPinB);

  if (PinB == LOW){
    if (PinA == HIGH){
      EncoderCount++;
    }
    else{
      EncoderCount--;
    }
  }
  else{
    if (PinA == HIGH){
      EncoderCount--;
    }
    else{
      EncoderCount++;
    }
  }
}

//Timer Interrupt Service Routine for a change in Encoder B
void ISR_EncoderB(){
  bool PinA = digitalRead(interruptPinA);
  bool PinB = digitalRead(interruptPinB);

  if (PinB == LOW){
    if (PinA == HIGH){
      EncoderCount--;
    }
    else{
      EncoderCount++;
    }
  }
  else{
    if (PinA == HIGH){
      EncoderCount++;
    }
    else{
      EncoderCount--;
    }
  }
}

//Function that turns the motor in the correct direction and speed for a given input voltage
void WriteMotorVoltage(float V, float Vmax){
  int PWMVal = int(255*abs(V)/Vmax); //Convert voltage to PWM value
  if (PWMVal > 255){ //Check PWM value does not exceed 255
    PWMVal = 255; 
  }

  // If the voltage is positive, drive the motor clockwise
  if (V>0){
    digitalWrite(DirPin1, HIGH);
    digitalWrite(DirPin2, LOW);
  }
  // If the voltage is negative, drive the motor anticlockwise
  else if (V<0){
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, HIGH);
  }
  // If the voltage is zero, turn the motor off
  else{
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, LOW);
  }
  analogWrite(PWMPin, PWMVal);  //Write the PWM value to the enable pin of the motor driver
}

// Use the current valve position and demanded valve position to get a voltage output using PID control
float getVoltage(float Theta, float Theta_d) {
   
   t = millis();
   dt = (t - t_prev); //Work out the time step since the last PID calculation

   e = Theta_d - Theta; //Work out the error between the actual and demanded valve position
   inte = inte_prev + (dt*(e+e_prev)/2); //Add to the integral of the error using the trapezium method

   V = kp*e + ki*inte + (kd*(e-e_prev)/dt); //Sum the P, I and D terms to get the voltage output

   // Check that the voltage is within the suitable range - correct it if not
   if (V > Vmax) {
     V = Vmax;
     inte = inte_prev; //Prevent windup of the integral term by limiting its max value
   }
   if (V < Vmin && V > -Vmin) {
     V = 0;
   }
   if (V < -Vmax) {
     V = -Vmax;
     inte = inte_prev; //Prevent windup of the integral term by limiting its max value
   }

   // Make the previous terms equal to the current terms
   inte_prev = inte;
   e_prev = e;
   t_prev = t;

   return V; //Return the voltage output for the motor
}

// Timer interrupt that is triggered every 50ms
ISR(TIMER1_COMPA_vect){
  count++; //Increment the variable count
}


void setup() {
  Serial.begin(9600); //Begin serial communication for debugging

  //Set the quadrature encoder pins to inputs
  pinMode(interruptPinA, INPUT);
  pinMode(interruptPinB, INPUT);

  //attach the interrupts to the encoder pins
  attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR_EncoderB, CHANGE);

  //Set the direction pins on the motor driver to outputs
  pinMode(DirPin1, OUTPUT);
  pinMode(DirPin2, OUTPUT);

  //Setup the timer interrupt to trigger every 50ms
  cli(); //Stop all other interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 12499; //Prescalar = 64
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS11 | 1 << CS10);
    TIMSK1 |= (1 << OCIE1A);
  sei(); //Restart other interrupts again

  //Calibrate valve zero position
  WriteMotorVoltage(-Vmin, Vmax); //Start running the motor at minimum speed to close the valve
  delay(500); //Wait to ensure the motor has started moving
  // Check every 50ms that the motor is still moving at a reasonable speed - when it stops, detect that it has reached the hard endstop
  while ((EncoderCount_prev - EncoderCount) > 20) {
    EncoderCount_prev = EncoderCount;
    delay(50);
  }

  EncoderCount = 0; //After reaching the hard endstop, zero the encoder count
  Theta_d = 40; //Set a demand position for the valve soft limit - this offset ensures the motor never runs into the hard endstop during operation

  //Using PID control, drive the motor to the soft limit point - loop every 10ms
  for (int i = 0; i <= 100; i++) {
     Theta = (1023*EncoderCount/(RevolutionCounts*NumRevolutions));
     WriteMotorVoltage(getVoltage(Theta, Theta_d),Vmax);
     delay(10);
  }  
  EncoderCount = 0; //After reaching the soft limit, set the new encoder zero position

}

void loop() {
  if (count > count_prev){ // Check if the count has incremented - the count increments every 50ms due to the timer interrupt

    Theta_d = analogRead(analogPin); //Read the demanded valve position from the potentiometer (0 - 1023)
    Theta = (1023*EncoderCount/(RevolutionCounts*NumRevolutions)); //Scale the encoder count to give the actual valve position between 0 and 1023
    
    V = getVoltage(Theta, Theta_d); //Call the getVoltage function

    //Output serial data for debugging
    Serial.print(Theta_d);
    Serial.print(" ");
    Serial.print(Theta);
    Serial.print(" ");
    Serial.print(V);
    Serial.println(" ");

    WriteMotorVoltage(V, Vmax); //Drive the motor with calculated voltage

    count_prev = count; //Make the previous count equal to the current count
  }

}

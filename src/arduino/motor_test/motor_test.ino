// Declare motor pins.

const int right_pwm_pin = 5;
const int right_dir_pin = A0;
const int left_pwm_pin = 6;
const int left_dir_pin = A1;
const bool left_fwd = true;
const bool right_fwd = false;

// variables for serial communication
int state = 0;
String serial_command = "";

void MoveLeft(const size_t speed){
    digitalWrite(right_dir_pin, right_fwd);
    digitalWrite(left_dir_pin, !left_fwd);
    analogWrite(right_pwm_pin, speed);
    analogWrite(left_pwm_pin, speed);
}

void MoveRight(const size_t speed){
    digitalWrite(right_dir_pin, !right_fwd);
    digitalWrite(left_dir_pin, left_fwd);
    analogWrite(right_pwm_pin, speed);
    analogWrite(left_pwm_pin, speed);
}

void MoveFwd(const size_t speed){
    digitalWrite(right_dir_pin, right_fwd);
    digitalWrite(left_dir_pin, left_fwd);
    analogWrite(right_pwm_pin, speed);
    analogWrite(left_pwm_pin, speed);
}

void MoveBwd(const size_t speed){
    digitalWrite(right_dir_pin, !right_fwd);
    digitalWrite(left_dir_pin, !left_fwd);
    analogWrite(right_pwm_pin, speed);
    analogWrite(left_pwm_pin, speed);
}

void MoveStop(){
    digitalWrite(right_dir_pin, right_fwd);
    digitalWrite(left_dir_pin, left_fwd);
    analogWrite(right_pwm_pin, 0);
    analogWrite(left_pwm_pin, 0);
}

void setup() {
  pinMode(right_pwm_pin, OUTPUT);    // sets the digital pin 13 as output
  pinMode(right_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);

  Serial.begin(9600);


  // Set initial values for directions. Set both to forward.
  digitalWrite(right_dir_pin, right_fwd);
  digitalWrite(left_dir_pin, left_fwd);


  // Send forward command.
  MoveFwd(200);

  delay(1000);

    MoveRight(180);
  delay(100);

  MoveStop();


}



void loop()
{
    // model as state machine


    switch (state)
    {

    // state 0, wait for serial information
    case 0:
        // if we have something to read, read it until getting endline
        if (Serial.available() != 0)
        {
            char in = Serial.read();
            // if the char we got is a new line, then continue to the next state and work with the command
            if (in == '\n')
            {
                Serial.println("serial command= " + serial_command);

                state = 1;
            }
            else
            {
                // otherwise, put what we got in the string and continue getting chars
                serial_command += in;
            }
        }
        break;

    // do stuff with the command
    case 1:
              Serial.print("got from serial=");
              Serial.println(serial_command);

        //if the command is 'a' turn left
        if (serial_command == "a")
        {
            MoveLeft(200);
        }
        else if (serial_command == "d")
        {
            MoveRight(200);
        }
        else if (serial_command == "w")
        {
            MoveFwd(255);
        }

        else if (serial_command == "s")
        {
            MoveBwd(255);
        }
        else
        {
            MoveStop();
            serial_command = "";
        }

        // Did something with the user input. Go on to state 2 to wait for a new input.

        state = 2;

        break;
    
    case 2:
        if (Serial.available() > 0)
        {
            serial_command = "";
            state = 0;
        }
        break;
    }

}

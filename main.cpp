#include <DabbleESP32.h>
#include <Servo.h>

//Servo Connections
Servo base;
int basepin = 13;
int basepos = 90;

Servo arm1;
int arm1pin = 12;
int arm1pos = 180;

Servo arm2;
int arm2pin = 11;
int arm2pos = 180;

Servo arm3;
int arm3pin = 8;
int arm3pos = 90;

Servo arm4;
int arm4pin = 9;
int arm4pos = 0;

Servo arm5;
int arm5pin = 10;
int arm5pos = 0;

// Motor A connections
int enA = 2;
int in1 = 3;
int in2 = 4;
// Motor B connections
int enB = 7;
int in3 = 5;
int in4 = 6;

// Variables for controlling the servos
int activecontrol = 1;
int servoDelay = 10; // Adjust this as needed

void stopMotors() {
    // Dừng tất cả các động cơ bằng cách tắt các chân điều khiển
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}

void setup() {
    Serial.begin(115200);  // Baudrate cho Serial Monitor.
    Dabble.begin(9600);    // Đảm bảo tốc độ baudrate cho HM-10 Bluetooth.

    // Setup Servo Pins
    base.attach(basepin);
    arm1.attach(arm1pin);
    arm2.attach(arm2pin);
    arm3.attach(arm3pin);
    arm4.attach(arm4pin);
    arm5.attach(arm5pin);

    // Set all the motor control pins to outputs
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    // Turn off motors - Begin state
    stopMotors();

    // Set all servos to default position
    base.write(90);
    arm1.write(180);
    arm2.write(180);
    arm3.write(90);
    arm4.write(0);
    arm5.write(0);
}

void controlMotorWithJoystick() {
    float x = GamePad.getXaxisData(); // Joystick X-axis value (-7 to 7)
    float y = GamePad.getYaxisData(); // Joystick Y-axis value (-7 to 7)

    // Calculate motor speeds
    int leftSpeed = constrain((y - x) * 36, -255, 255);  // Swapped: subtract x for left motor
    int rightSpeed = constrain((y + x) * 36, -255, 255); // Swapped: add x for right motor

    // Set motor A (left side)
    if (leftSpeed > 0) {
        analogWrite(enA, leftSpeed);
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else if (leftSpeed < 0) {
        analogWrite(enA, abs(leftSpeed));
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else {
        stopMotors();
    }

    // Set motor B (right side)
    if (rightSpeed > 0) {
        analogWrite(enB, rightSpeed);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
    } else if (rightSpeed < 0) {
        analogWrite(enB, abs(rightSpeed));
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
    } else {
        stopMotors();
    }
}

// Servo movement functions
void moveServo(Servo &servo, int &pos, int direction) {
    if (direction == -1 && pos > 0) {
        pos -= 1;
    } else if (direction == 1 && pos < 180) {
        pos += 1;
    }
    servo.write(pos);
    delay(servoDelay); // short delay to allow servo to reach the position
}

void loop() {
    Dabble.processInput();  // This function refreshes data obtained from the smartphone

    // Button press actions
    if (GamePad.isUpPressed()) {
        Serial.println("UP");
        // Add your forward motion code here
    }

    if (GamePad.isDownPressed()) {
        Serial.println("DOWN");
        // Add your backward motion code here
    }

    if (GamePad.isLeftPressed()) {
        Serial.println("LEFT");
        // Add your left motion code here
    }

    if (GamePad.isRightPressed()) {
        Serial.println("RIGHT");
        // Add your right motion code here
    }

    if (GamePad.isSquarePressed()) {
        Serial.println("Square");
        // Control servo based on active control
        if (activecontrol == 1) {
            moveServo(base, basepos, -1);
        } else if (activecontrol == 2) {
            moveServo(arm3, arm3pos, 1);
        } else if (activecontrol == 3) {
            moveServo(arm5, arm5pos, -1);
        }
    }

    if (GamePad.isCirclePressed()) {
        Serial.println("Circle");
        if (activecontrol == 1) {
            moveServo(base, basepos, 1);
        } else if (activecontrol == 2) {
            moveServo(arm3, arm3pos, -1);
        } else if (activecontrol == 3) {
            moveServo(arm5, arm5pos, 1);
        }
    }

    if (GamePad.isCrossPressed()) {
        Serial.println("Cross");
        if (activecontrol == 1) {
            moveServo(arm1, arm1pos, -1);
        } else if (activecontrol == 2) {
            moveServo(arm2, arm2pos, -1);
        } else if (activecontrol == 3) {
            moveServo(arm4, arm4pos, -1);
        }
    }

    if (GamePad.isTrianglePressed()) {
        Serial.println("Triangle");
        if (activecontrol == 1) {
            moveServo(arm1, arm1pos, 1);
        } else if (activecontrol == 2) {
            moveServo(arm2, arm2pos, 1);
        } else if (activecontrol == 3) {
            moveServo(arm4, arm4pos, 1);
        }
    }

    if (GamePad.isStartPressed()) {
        activecontrol = (activecontrol == 3) ? 1 : 3;
        delay(100);
    }

    if (GamePad.isSelectPressed()) {
        activecontrol = (activecontrol == 2) ? 1 : 2;
        delay(100);
    }

    controlMotorWithJoystick(); // Call to handle motor control via joystick
}

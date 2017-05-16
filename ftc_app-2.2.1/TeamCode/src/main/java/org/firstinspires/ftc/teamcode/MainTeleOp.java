/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * CONTROL SCHEME
 *
 * Gamepad 1:
 * Motor one and motor two - left joystick
 * Motor three and motor four - right joystick
 * Color sensor enable - X
 */

// It's a teleop. It ops the tele. Simple.
// This is the teleOp for the new and improved robot!!!
@TeleOp(name = "TheTeleOp", group = "Iterative Opmode")
public class MainTeleOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    // These are constants, once you set them you cannot change them
    // Nice for settings things like motor power
    // This way you can use names instead of "magic numbers"
    static final double MOTOR_FULL_POWER=1.0;
    static final double MOTOR_LESS_POWER=0.7;
    static final double MOTOR_HALF_POWER=0.5;
    static final double MOTOR_LEAST_POWER=0.3;
    static final double MOTOR_POWER_OFF=0.0;

    // Defining your motors - DcMotor is a class provided by the FTC SDK (software dev kit)
    DcMotor motorOne;
    DcMotor motorTwo;
    DcMotor motorThree;
    DcMotor motorFour;
    DcMotor liftMotor;
    Servo ballStopper;
    DcMotor ballShooter;


    // Similarly, if you wanted to define a servo, you would put:
    // Servo servoName;
    // The servoName can be anything you want, it's a variable


    /*
     * This runs when the driver station "init" button is pressed
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        /*
         * Use hardwareMap.dcMotor.get() to map the variable initialized above to the motor.
         * The argument in quotes is the name of the motor. You set this in the robot profile
         * on the robot controller phone.
         */
        motorOne = hardwareMap.dcMotor.get("motor one"); // MAP ALL THE HARDWARE
        motorTwo = hardwareMap.dcMotor.get("motor two"); // HARDWARE ALL THE MAP
        motorThree = hardwareMap.dcMotor.get("motor three");
        motorFour = hardwareMap.dcMotor.get("motor four");
        liftMotor = hardwareMap.dcMotor.get("lift motor");
        ballStopper = hardwareMap.servo.get("ball stopper");
        ballShooter = hardwareMap.dcMotor.get("ball shooter");


        // You have to reverse one motor, otherwise a power value of 1.0 would make the motors run
        // in different directions. This just makes it more convenient, so you don't have to use 1.0
        // for one motor and -1.0 for the other motor.
        motorOne.setDirection(DcMotorSimple.Direction.REVERSE);
        motorThree.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    /*
     * This runs after the "init" button is pressed but before the "play" button is pressed
     */
    @Override
    public void init_loop() {
    }

    //Hello

    /*
     * This runs when the "play" button is pressed on the driver station
     */
    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    // This runs after the "play" button is pressed and before the "stop" button is pressed
    public void loop() {
        // If you put the cursor on a comment that says "region" and press command-minus, you can collapse the code

        //region Telemetry

        // Telemetry == stuff that shows up on the driver station phone screen
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Wheels", "1: " + motorOne.getPower() + " 2: " + motorTwo.getPower() + " 3: " + motorThree.getPower() + "4: " + motorFour.getPower());

        //endregion

        //region Wheels


        // There will be 4 motors in the next design
        // This should now work to have motor one and motor two to work on the left joystick and motor three and four to work on the right joystick


        // This will let the robot turn left and right
        if(gamepad1.left_bumper){
            motorOne.setPower(-MOTOR_HALF_POWER);
            motorTwo.setPower(MOTOR_HALF_POWER);
            motorThree.setPower(-MOTOR_HALF_POWER);
            motorFour.setPower(MOTOR_HALF_POWER);
        }
        else if(gamepad1.right_bumper) {
            motorOne.setPower(MOTOR_HALF_POWER);
            motorTwo.setPower(-MOTOR_HALF_POWER);
            motorThree.setPower(MOTOR_HALF_POWER);
            motorFour.setPower(-MOTOR_HALF_POWER);
        }
        else{
            motorOne.setPower(joystickToMotorValue(gamepad1.left_stick_y)); // This sets power
            motorTwo.setPower(joystickToMotorValue(gamepad1.left_stick_y));

            // This will set motor three and motor four to guide the robot to the left and right
            motorThree.setPower(joystickToMotorValue(gamepad1.left_stick_x)); // This sets power. again. but the other motor.
            motorFour.setPower(joystickToMotorValue(gamepad1.left_stick_x));  // Move the joystick left and right
        }


        //endregion

        //region center motors - this is currently all commented (not in use)

        //if(gamepad1.a){
            //centerMotorOne.setPower(MOTOR_FULL_POWER);
            //centerMotorTwo.setPower(MOTOR_FULL_POWER);
        //}
        //else {
            //centerMotorOne.setPower(MOTOR_POWER_OFF);
            //centerMotorTwo.setPower(MOTOR_POWER_OFF);
        //}

        //endregion -  -

        //region lifting motors

        //remember this motor will spin two sets of lifters

        double liftPower;
        if (gamepad1.left_trigger>0.0 && gamepad1.right_trigger==0.0) {
            liftPower=gamepad1.left_trigger;
        }
        else if (gamepad1.right_trigger>0.0 && gamepad1.left_trigger==0.0) {
            liftPower=-gamepad1.right_trigger;
        }
        else {
            liftPower=MOTOR_POWER_OFF;
        }
        liftMotor.setPower(liftPower);



        //endregion

        //region Ball stopping servo
        if(gamepad1.a){
            ballStopper.setPosition(0.5);
        }
        else {
            ballStopper.setPosition(1.0);
        }



        //endregion

        //region Ball shooter

        if(gamepad1.b){
            ballShooter.setPower(MOTOR_LESS_POWER);
        }
        else {
            ballShooter.setPower(MOTOR_POWER_OFF);
        }



        //endregion

        //Gotta write up that code for the shooting mechanism
    }

    /*
     * This runs once after the stop button is pressed on the driver station
     */
    @Override
    public void stop() {
    }

    // This scales the input so that we maintain precision control
    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

    // This clips the values from the joystick so that they can be used for the motor.
    double joystickToMotorValue(double joystickValue) {
        joystickValue = Range.clip(joystickValue, (float) -1.0, (float) 1.0);
        double scaled = scaleInput(joystickValue); // This scales input for some reason
        return scaled;
    }

}

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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@Autonomous(name = "Beacon Press B", group = "Autonomous Blue")
public class AutoBlueBeacon extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor;
    DcMotor rightMotor;
    ColorSensor colorSensor;
    DcMotor highMotor;
    static final double DRIVE_POWER = 1.0;
    static final double DRIVE_LESS_POWER = 0.3;
    final double MOVE_TIME = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Place next to ramp");
        telemetry.update();

        leftMotor = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor");
        highMotor = hardwareMap.dcMotor.get("highMotor");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        colorSensor.enableLed(false);

        telemetry.addData("LMotor:", leftMotor.getConnectionInfo(), leftMotor.getDirection());
        telemetry.addData("RMotor:", rightMotor.getConnectionInfo(), rightMotor.getDirection());
        telemetry.addData("HMotor:", highMotor.getConnectionInfo(), rightMotor.getDirection());
        telemetry.addData("Color:", colorSensor.getConnectionInfo());
        telemetry.update();

        waitForStart();

        telemetry.clearAll();
        telemetry.addLine("Color Sensor");
        telemetry.addData("Red:", colorSensor.red());
        telemetry.addData("Green:", colorSensor.green());
        telemetry.addData("Blue:", colorSensor.blue());
        telemetry.update();

        // Run the robot
        // action(DRIVE_POWER, time sec)
        driveF(DRIVE_POWER, 1.65);
        turnRight(DRIVE_POWER, 1.75);
        //driveF(DRIVE_POWER,1.35);
        turnRightArc(2.35); // todo: this time value may not be correct
        turnRight(DRIVE_POWER, 1.65);
        driveF(DRIVE_LESS_POWER, 0.7);

        //region Claim beacon
        if (colorSensor.blue() > colorSensor.red()) {
            highMotor.setPower(1.0);
            while (runtime.seconds() < MOVE_TIME && opModeIsActive()) ;
            highMotor.setPower(0);
        } else {
            double tracker = System.currentTimeMillis() + 550;
            while (System.currentTimeMillis() < tracker && opModeIsActive()) {
                leftMotor.setPower(1.0);
                rightMotor.setPower(1.0);
            }
            stopDriving();
            highMotor.setPower(1.0);
            while (runtime.seconds() < MOVE_TIME && opModeIsActive()) ;
            highMotor.setPower(0);
        }
        //endregion

        turnRight(DRIVE_POWER,0.9);
        driveF(DRIVE_POWER,1.5);
        turnLeft(DRIVE_POWER,1.2);
        driveF(DRIVE_POWER,12);
    }

        /*
        To get the color from color sensor, you can assign a variable:
        float[] perceivedColor = getColorRGB();

        Then you would update it, maybe using a while loop, like this:
        perceivedColor = getColorRGB();

        You can also use it with condition checking:
        if (getColorRGB()[0]>200) { do something }
         */

    public void driveF(double power, double time) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        runtime.reset();
        while (runtime.seconds() < time && opModeIsActive()) ;
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        waitSec(0.3);
    }

    public void driveR(double power, double time) {
        leftMotor.setPower(-power);
        rightMotor.setPower(-power);
        runtime.reset();
        while (runtime.seconds() < time && opModeIsActive()) ;
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        waitSec(0.3);
    }

    public void turnLeft(double power, double time) {
        rightMotor.setPower(power);
        leftMotor.setPower(-power);
        runtime.reset();
        while (runtime.seconds() < time && opModeIsActive()) ;
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        waitSec(0.3);

    }

    public void turnRight(double power, double time) {
        rightMotor.setPower(-power);
        leftMotor.setPower(power);
        runtime.reset();
        while (runtime.seconds() < time && opModeIsActive()) ;
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        waitSec(0.3);
    }

    public void turnLeftArc(double time) {
        rightMotor.setPower(1.0);
        leftMotor.setPower(0.1);
        waitSec(time);
        stopDriving();
    }

    public void turnRightArc(double time) {
        leftMotor.setPower(1.0);
        rightMotor.setPower(0.1);
        waitSec(time);
        stopDriving();
    }

    public void stopDriving() {
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }

    public void extendArm(double power, double time) {
        highMotor.setPower(1.0);
    }

    public void shortenArm(double power, double time) {
        highMotor.setPower(-1.0);
    }

    public void waitSec(double length) {
        runtime.reset();
        while (runtime.seconds() < length && opModeIsActive()) ;
    }
}

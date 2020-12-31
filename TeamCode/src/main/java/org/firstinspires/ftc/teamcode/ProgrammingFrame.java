/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.ar.pl.SystemTools;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



public class ProgrammingFrame
{

    // define motors
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;
    public DcMotor intake = null;
    public DcMotor shooting = null;
    public DcMotor lifting = null;
    public Servo gripperServo = null;
    public Servo ringPusher = null;
    public Servo storageServo = null;

    public NormalizedColorSensor colorSensor1;
    public NormalizedColorSensor colorSensor2;
    public NormalizedColorSensor bottomRing;
    public NormalizedColorSensor topRing;
    public OpMode systemTools;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, OpMode systemToolsIn) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        systemTools = systemToolsIn;


        // Define and Initialize Motors
        frontLeftMotor  = hwMap.get(DcMotor.class, "front_left_drive");
        frontRightMotor = hwMap.get(DcMotor.class, "front_right_drive");
        backLeftMotor = hwMap.get(DcMotor.class, "back_left_drive");
        backRightMotor = hwMap.get(DcMotor.class, "back_right_drive");
        intake = hwMap.get(DcMotor.class, "intake");
        shooting = hwMap.get(DcMotor.class, "shooting");
        lifting = hwMap.get(DcMotor.class, "lifting");


        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifting.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // define and initialize servo
        gripperServo = hwMap.get(Servo.class, "gripper");
        ringPusher = hwMap.get(Servo.class, "push_arm");
        storageServo = hwMap.get(Servo.class, "storage_servo");
        // set motor directions
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // set to brake when power is 0
        //backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // Set all motors to zero power
        stopAllMotors();
//        frontLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        backLeftMotor.setPower(0);
//        backRightMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        startDriveEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        colorSensor1 = hwMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor2 = hwMap.get(NormalizedColorSensor.class, "sensor_color2");
        bottomRing = hwMap.get(NormalizedColorSensor.class, "sensor_color3");
        topRing = hwMap.get(NormalizedColorSensor.class, "sensor_color4");

        ringPusher.setPosition(0);
        storageServo.setPosition(1);
    }

    // go distance function
    public void GoDistanceCM(int centimeters, double power, LinearOpMode linearOpMode){
        // holds the conversion factor for ticks to centimeters
        final double conversion_factor = 27.82;

        // sets the power negative if the distance is negative
        if (centimeters < 0 && power > 0) {
            power = power * -1;
        }

        // calculates the target amount of motor ticks
        int TICKS = (int) Math.abs(Math.round(centimeters * conversion_factor));

        // Debug: Send telemetry message with calculated ticks;
        systemTools.telemetry.addData("Calculated Counts =", TICKS);
     //   systemTools.telemetry.update();

       // Thread.sleep(2000); // debugging: allow time to view telemetry

        // Send telemetry message to signify robot waiting;
        systemTools.telemetry.addLine();
        systemTools.telemetry.addData("Status", "Resetting Encoders");
     //   systemTools.telemetry.update();

        resetDriveEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        systemTools.telemetry.addLine();
        systemTools.telemetry.addData("Initial pos.", "Starting at %7d :%7d :%7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
      //  systemTools.telemetry.update();
      //  wait(2000); // debugging: allow time to view telemetry
        /*
        // sets the target position for each of the motor encoders
        int FLtarget = frontLeftMotor.getCurrentPosition() + TICKS;
        int FRtarget = frontRightMotor.getCurrentPosition() + TICKS;
        int BLtarget = backLeftMotor.getCurrentPosition() + TICKS;
        int BRtarget = backRightMotor.getCurrentPosition() + TICKS;
        M.M. 12/24- Redundant */

        startDriveEncoders();

        // reset the timeout time and start motion.
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
        backLeftMotor.setPower(power);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (linearOpMode.opModeIsActive() &&
                (Math.abs(frontLeftMotor.getCurrentPosition()) < TICKS && Math.abs(frontRightMotor.getCurrentPosition()) < TICKS && Math.abs(backLeftMotor.getCurrentPosition()) < TICKS && Math.abs(backRightMotor.getCurrentPosition()) < TICKS)) {
        }

        stopDriveMotors();

//        frontLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        backRightMotor.setPower(0);
//        backLeftMotor.setPower(0);

 // fem 12-24  debug       startDriveEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        systemTools.telemetry.addLine();
        systemTools.telemetry.addData("Final", "Starting at %7d :%7d :%7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        systemTools.telemetry.update();

        /*
        systemTools.telemetry.addData("Path", "Complete");
        systemTools.telemetry.addData("counts", TICKS);
        systemTools.telemetry.update();

         */
    }

    // function for rotating the robot
    public void RotateDEG(int degrees, double power, LinearOpMode linearOpMode) {

        // conversion for degrees to ticks
        final double conversion_factor = 27.82;

        // if degrees are negative, set the power negative
        if (degrees < 0 && power > 0) {
            power = power * -1;
        }

        int TICKS = (int) Math.abs(Math.round(degrees * conversion_factor));
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        systemTools.telemetry.addData("Status", "Resetting Encoders");
        systemTools.telemetry.update();

        resetDriveEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        systemTools.telemetry.update();

        // set target position for all the motor encoders
        int FLtarget = frontLeftMotor.getCurrentPosition() + TICKS;
        int FRtarget = frontRightMotor.getCurrentPosition() - TICKS;
        int BLtarget = backLeftMotor.getCurrentPosition() + TICKS;
        int BRtarget = backRightMotor.getCurrentPosition() - TICKS;

        startDriveEncoders();

        // reset the timeout time and start motion.
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
        backRightMotor.setPower(-power);
        backLeftMotor.setPower(power);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (linearOpMode.opModeIsActive() &&
                (Math.abs(frontLeftMotor.getCurrentPosition()) < TICKS && Math.abs(frontRightMotor.getCurrentPosition()) < TICKS && Math.abs(backLeftMotor.getCurrentPosition()) < TICKS && Math.abs(backRightMotor.getCurrentPosition()) < TICKS)) {
        }

        stopDriveMotors();
//        frontLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        backRightMotor.setPower(0);
//        backLeftMotor.setPower(0);

        startDriveEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        systemTools.telemetry.addData("Path", "Complete");
        systemTools.telemetry.addData("counts", TICKS);
        systemTools.telemetry.update();
    }

    // robot strafing function
    public void StrafeCM(int centimeters, double power, LinearOpMode linearOpMode){

        // conversion factor between ticks and centimeters
        final double conversion_factor = 27.55;  //Corrected from 27.82 to 27.55 12-26-20

        // if the distance is negative, set power negative
        if (centimeters < 0 && power > 0) {
            power = power * -1;
        }

        int TICKS = (int) Math.abs(Math.round(centimeters * conversion_factor));

        // Send telemetry message to signify robot waiting;
        systemTools.telemetry.addData("Status", "Resetting Encoders");
        systemTools.telemetry.update();

        resetDriveEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        systemTools.telemetry.update();

        // set target position for motor encoders
        int FLtarget = frontLeftMotor.getCurrentPosition() - TICKS;
        int FRtarget = frontRightMotor.getCurrentPosition() + TICKS;
        int BLtarget = backLeftMotor.getCurrentPosition() + TICKS;
        int BRtarget = backRightMotor.getCurrentPosition() - TICKS;

        startDriveEncoders();

        // after resetting encoders, apply power to the motors
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
        backLeftMotor.setPower(-power);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (linearOpMode.opModeIsActive() &&
                (Math.abs(frontLeftMotor.getCurrentPosition()) < TICKS && Math.abs(frontRightMotor.getCurrentPosition()) < TICKS && Math.abs(backLeftMotor.getCurrentPosition()) < TICKS && Math.abs(backRightMotor.getCurrentPosition()) < TICKS)) {
        }

        stopDriveMotors();
//        frontLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        backRightMotor.setPower(0);
//        backLeftMotor.setPower(0);

        startDriveEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        systemTools.telemetry.addData("Path", "Complete");
        systemTools.telemetry.addData("counts", TICKS);
        systemTools.telemetry.update();
    }

    // find line function
    public void findLine(double power, LinearOpMode linearOpMode) {
        // Send telemetry message to signify robot waiting;
       systemTools.telemetry.addData("Status", "Resetting Encoders");
       systemTools.telemetry.update();

        resetDriveEncoders();


        // Send telemetry message to indicate successful Encoder reset
        systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        systemTools.telemetry.update();

        NormalizedRGBA colors1 = colorSensor1.getNormalizedColors();
        NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();

        // reset the timeout time and start motion.
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
        backLeftMotor.setPower(power);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // When the color sensors detect the line, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (linearOpMode.opModeIsActive() &&
                //        (runtime.seconds() < 30) &&
                ((colors1.red != 0 && colors1.green != 0 && colors1.blue != 0) || (colors2.red != 0 && colors2.green != 0 && colors2.blue != 0))) {
            colors1 = colorSensor1.getNormalizedColors();
            colors2 = colorSensor2.getNormalizedColors();
        }

        stopDriveMotors();

        startDriveEncoders();

        systemTools.telemetry.addData("Path", "Complete");
        systemTools.telemetry.update();
    }

    public char ringFinder() {
        char path;
        boolean sensor1Detected;
        boolean sensor2Detected;
        float gain = 2;
        int hueTarget = 30;
        final float[] hsvValues = new float[3];
        final float[] hsvValues2 = new float[3];

        systemTools.telemetry.addData("Gain", gain);

        // set gain on color sensors
        topRing.setGain(gain);
        bottomRing.setGain(gain);

        // get color sensors
        NormalizedRGBA colors2 = topRing.getNormalizedColors();
        NormalizedRGBA colors1 = bottomRing.getNormalizedColors();

        Color.colorToHSV(colors1.toColor(), hsvValues);
        Color.colorToHSV(colors2.toColor(), hsvValues2);

        // checks if values are within the bounds
        sensor1Detected = hsvValues[0] > hueTarget;
        sensor2Detected = hsvValues2[0] > hueTarget;

        // return a character determined by the color sensor output
        if (sensor1Detected && sensor2Detected) {
            path = 'C';
        } else if (sensor1Detected && !sensor2Detected) {
            path = 'B';
        } else if (!sensor1Detected && !sensor2Detected) {
            path = 'A';
        } else { // Means there was an error
            path = 'E';
        }

        systemTools.telemetry.addData("Path letter (E is Error): ", path);
        systemTools.telemetry.update();
        //while (linearOpMode.opModeIsActive()) {}  //  Empty while loop - program waits until user terminates op-mode
        return path;
    }


    public void stopDriveMotors() {
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);

    }

    public void stopAllMotors() {
        stopDriveMotors();
        intake.setPower(0);
        shooting.setPower(0);
        lifting.setPower(0);
    }

    public void resetDriveEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void startDriveEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveGripper(boolean close) {
        gripperServo.scaleRange(0, 1.0);
        if (close) {
            gripperServo.setDirection(Servo.Direction.FORWARD);
            gripperServo.setPosition(0.25);
        }
        else {
            gripperServo.setDirection(Servo.Direction.REVERSE);
            gripperServo.setPosition(0);
        }
    }

    public void raiseGripper(LinearOpMode linearOpMode) {
        lifting.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifting.setTargetPosition(lifting.getCurrentPosition() + 320);
        lifting.setPower(1);
        while (linearOpMode.opModeIsActive() && lifting.isBusy()) {}
        lifting.setPower(0);
    }

    public void lowerGripper(LinearOpMode linearOpMode) {
        lifting.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifting.setTargetPosition(lifting.getCurrentPosition() - 320);
        lifting.setPower(-1);
        while (linearOpMode.opModeIsActive() && lifting.isBusy()) {}
        lifting.setPower(0);
    }

    public void GoDistanceTICKS2(int ticks, double power, LinearOpMode linearOpMode) {


        // Send telemetry message to signify robot waiting;
        systemTools.telemetry.addData("Status", "Resetting Encoders");
       // systemTools.telemetry.update();

        resetDriveEncoders();

        startDriveEncoders();
        // Send telemetry message to indicate successful Encoder reset
        systemTools.telemetry.addLine();
        systemTools.telemetry.addData("Initial pos.", "Starting at %7d :%7d :%7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());

               // Wait for the game to start (driver presses PLAY)


        // start motion.
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
        backLeftMotor.setPower(power);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (linearOpMode.opModeIsActive() &&
                (Math.abs(frontLeftMotor.getCurrentPosition()) < ticks && Math.abs(frontRightMotor.getCurrentPosition()) < ticks && Math.abs(backLeftMotor.getCurrentPosition()) < ticks && Math.abs(backRightMotor.getCurrentPosition()) < ticks)) {
        }

        stopDriveMotors();

        startDriveEncoders();
        systemTools.telemetry.addLine();
        systemTools.telemetry.addData("Final pos.", "Starting at %7d :%7d :%7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());

        systemTools.telemetry.addLine().addData("Path", "Complete");
        systemTools.telemetry.update();
    }


    public void GoDistanceTICKS(int ticks, double power, LinearOpMode linearOpMode) {


        // Send telemetry message to signify robot waiting;
        systemTools.telemetry.addData("Status", "Resetting Encoders");
        systemTools.telemetry.update();

        resetDriveEncoders();

        // Send telemetry message to indicate successful Encoder reset
        systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        systemTools.telemetry.update();

        // Wait for the game to start (driver presses PLAY)


        // reset the timeout time and start motion.
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
        backLeftMotor.setPower(power);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (linearOpMode.opModeIsActive() &&
                (Math.abs(frontLeftMotor.getCurrentPosition()) < ticks && Math.abs(frontRightMotor.getCurrentPosition()) < ticks && Math.abs(backLeftMotor.getCurrentPosition()) < ticks && Math.abs(backRightMotor.getCurrentPosition()) < ticks)) {
        }

        stopDriveMotors();

        startDriveEncoders();

        systemTools.telemetry.addData("Path", "Complete");
        systemTools.telemetry.update();
    }

    //*********************************************************************************************
    // go distance function
    public void GoDistanceCM2(int centimeters, double power, LinearOpMode linearOpMode){
        // holds the conversion factor for TICKS to centimeters
        final double conversion_factor = 27.55;

        // sets the power negative if the distance is negative
        if (centimeters < 0 && power > 0) {
            power = power * -1;
        }

        // calculates the target amount of motor TICKS
        int TICKS = (int) Math.abs(Math.round(centimeters * conversion_factor));

        // Debug: Send telemetry message with calculated TICKS;
        systemTools.telemetry.addData("Calculated Counts =", TICKS);
        //   systemTools.telemetry.update();


        // Send telemetry message to signify robot waiting;
        systemTools.telemetry.addLine();
        systemTools.telemetry.addData("Status", "Resetting Encoders");
        //   systemTools.telemetry.update();

        resetDriveEncoders();

//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        systemTools.telemetry.addLine();
        systemTools.telemetry.addData("Initial pos.", "Starting at %7d :%7d :%7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        //  systemTools.telemetry.update();


        // sets the target position for each of the motor encoders
        int FLtarget = frontLeftMotor.getCurrentPosition() + TICKS;
        int FRtarget = frontRightMotor.getCurrentPosition() + TICKS;
        int BLtarget = backLeftMotor.getCurrentPosition() + TICKS;
        int BRtarget = backRightMotor.getCurrentPosition() + TICKS;

        frontLeftMotor.setTargetPosition(FLtarget);
        frontRightMotor.setTargetPosition(FRtarget);
        backLeftMotor.setTargetPosition(BLtarget);
        backRightMotor.setTargetPosition(BRtarget);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //startDriveEncoders();  // disabled 12-26-20 - This is not needed when driving by RUN_TO_POSITION.  Enabling this causes measurement errors

        // reset the timeout time and start motion.
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
        backLeftMotor.setPower(power);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.

        /*while (linearOpMode.opModeIsActive() &&
                (Math.abs(frontLeftMotor.getCurrentPosition()) < TICKS && Math.abs(frontRightMotor.getCurrentPosition()) < TICKS && Math.abs(backLeftMotor.getCurrentPosition()) < TICKS && Math.abs(backRightMotor.getCurrentPosition()) < TICKS)) {
        }
        */

        while (linearOpMode.opModeIsActive() &&
                (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())){
        }
        stopDriveMotors();

//        frontLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        backRightMotor.setPower(0);
//        backLeftMotor.setPower(0);

        // fem 12-24  debug       startDriveEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        systemTools.telemetry.addLine();
        systemTools.telemetry.addData("Final", "Starting at %7d :%7d :%7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        systemTools.telemetry.update();

        //while (linearOpMode.opModeIsActive()) {}  //  Empty while loop - program waits until user terminates op-mode

        /*
        systemTools.telemetry.addData("Path", "Complete");
        systemTools.telemetry.addData("counts", TICKS);
        systemTools.telemetry.update();

         */
    }

    public void StrafeCM2(int centimeters, double power, LinearOpMode linearOpMode){
        // holds the conversion factor for TICKS to centimeters
        final double conversion_factor = 27.55;

        // sets the power negative if the distance is negative
        if (centimeters < 0 && power > 0) {
            power = power * -1;
        }

        // calculates the target amount of motor TICKS
        int TICKS = (int) Math.abs(Math.round(centimeters * conversion_factor));

        // Debug: Send telemetry message with calculated TICKS;
        systemTools.telemetry.addData("Calculated Counts =", TICKS);
        //   systemTools.telemetry.update();


        // Send telemetry message to signify robot waiting;
        systemTools.telemetry.addLine();
        systemTools.telemetry.addData("Status", "Resetting Encoders");
        //   systemTools.telemetry.update();

        resetDriveEncoders();

//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        systemTools.telemetry.addLine();
        systemTools.telemetry.addData("Initial pos.", "Starting at %7d :%7d :%7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        //  systemTools.telemetry.update();


        // sets the target position for each of the motor encoders
        int FLtarget = frontLeftMotor.getCurrentPosition() - TICKS;   // was positive.  Front Left and Back Right Need negative to strafe right 12-26-20
        int FRtarget = frontRightMotor.getCurrentPosition() + TICKS;    // was negative.  Front right and Back left Need negative to strafe right 12-26-20
        int BLtarget = backLeftMotor.getCurrentPosition() + TICKS;
        int BRtarget = backRightMotor.getCurrentPosition() - TICKS;

        frontLeftMotor.setTargetPosition(FLtarget);
        frontRightMotor.setTargetPosition(FRtarget);
        backLeftMotor.setTargetPosition(BLtarget);
        backRightMotor.setTargetPosition(BRtarget);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //startDriveEncoders();  // disabled 12-26-20 - This is not needed when driving by RUN_TO_POSITION.  Enabling this causes measurement errors

        // reset the timeout time and start motion.
        // Should these be all positive with the targets? Or should they still be negative?
        frontLeftMotor.setPower(power);     // was positive.  Front Left and Back Right Need negative to strafe right 12-26-20
        frontRightMotor.setPower(-power); // was negative.  Front right and Back left Need negative to strafe right 12-26-20
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(power);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (linearOpMode.opModeIsActive() &&
                (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {
        }

        stopDriveMotors();

//        frontLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        backRightMotor.setPower(0);
//        backLeftMotor.setPower(0);

        // fem 12-24  debug       startDriveEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        systemTools.telemetry.addLine();
        systemTools.telemetry.addData("Final", "Starting at %7d :%7d :%7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        systemTools.telemetry.update();

        //while (linearOpMode.opModeIsActive()) {}  //  Empty while loop - program waits until user terminates op-mode

        /*
        systemTools.telemetry.addData("Path", "Complete");
        systemTools.telemetry.addData("counts", TICKS);
        systemTools.telemetry.update();

         */
    }

    public void StrafeDistanceCM(int centimeters, double power, LinearOpMode linearOpMode){

        final double conversion_factor = 27.82;
        boolean left = centimeters > 0;
        int TICKS = (int) Math.round(centimeters * conversion_factor);
        int FLtarget = 0;
        int FRtarget = 0;
        int BLtarget = 0;
        int BRtarget = 0;

        power = Math.abs(power);


        // Send telemetry message to signify robot waiting;
        systemTools.telemetry.addData("Status", "Resetting Encoders");
        systemTools.telemetry.update();

        resetDriveEncoders();

        // Send telemetry message to indicate successful Encoder reset
        systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        systemTools.telemetry.update();

        // Wait for the game to start (driver presses PLAY)

        if (left) {
            FLtarget = frontLeftMotor.getCurrentPosition() - TICKS;
            FRtarget = frontRightMotor.getCurrentPosition() + TICKS;
            BLtarget = backLeftMotor.getCurrentPosition() + TICKS;
            BRtarget = backRightMotor.getCurrentPosition() - TICKS;
        } else {
            FLtarget = frontLeftMotor.getCurrentPosition() + TICKS;
            FRtarget = frontRightMotor.getCurrentPosition() - TICKS;
            BLtarget = backLeftMotor.getCurrentPosition() - TICKS;
            BRtarget = backRightMotor.getCurrentPosition() + TICKS;
        }
        frontLeftMotor.setTargetPosition(FLtarget);
        frontRightMotor.setTargetPosition(FRtarget);
        backLeftMotor.setTargetPosition(BLtarget);
        backRightMotor.setTargetPosition(BRtarget);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if (left) {
            frontLeftMotor.setPower(-power);
            frontRightMotor.setPower(power);
            backRightMotor.setPower(-power);
            backLeftMotor.setPower(power);
        } else {
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(-power);
            backRightMotor.setPower(power);
            backLeftMotor.setPower(-power);
        }
        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (linearOpMode.opModeIsActive() &&
                (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {
        }

        stopDriveMotors();

        startDriveEncoders();

        systemTools.telemetry.addData("Path", "Complete");
        systemTools.telemetry.addData("counts", TICKS);
        systemTools.telemetry.update();
    }
}
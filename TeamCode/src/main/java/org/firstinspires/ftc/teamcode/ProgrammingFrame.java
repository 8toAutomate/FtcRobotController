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
import android.telephony.mbms.MbmsErrors;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.ar.pl.SystemTools;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.BlueAutoSecondTourney;
import org.firstinspires.ftc.teamcode.TeleOp.WobbleControl;

import static com.qualcomm.robotcore.hardware.configuration.ConfigurationType.DeviceFlavor.I2C;
import com.qualcomm.hardware.rev.RevColorSensorV3;

public class ProgrammingFrame {

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
    public NormalizedColorSensor bottomRingColor;
    public NormalizedColorSensor topRingColor;

    public TouchSensor lowSwitch1;
    public TouchSensor highSwitch1;
    public TouchSensor lowSwitch2;
    public TouchSensor highSwitch2;
    public OpMode systemTools;

    public DistanceSensor bottomRing;
    public DistanceSensor topRing;
    public DistanceSensor wobbleSensor;

    public RevBlinkinLedDriver lights;

    enum States {
        On, Off, Backwards, Forwards
    }

    States armState = States.Off;

    enum LightsStates {
        Off, SixtySecs, FiftySecs, FourtySecs, FlashFreeze, LowBattery, GrabberLimit, Normal, Custom
    }

    public LightsStates lightsState = LightsStates.Off;

    public boolean wobbleSuccess;

    public int degreesRotated;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
//*************************************************************************************************
//*******************************  Initialization *************************************************

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, OpMode systemToolsIn) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        systemTools = systemToolsIn;


        // Define and Initialize Motors
        frontLeftMotor = hwMap.get(DcMotor.class, "front_left_drive");
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
        lights = hwMap.get(RevBlinkinLedDriver.class, "ledController");
        // set motor directions
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // set to brake when power is 0
        //backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lifting.setDirection(DcMotor.Direction.FORWARD);

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
        colorSensor1 = hwMap.get(NormalizedColorSensor.class, "leftLine");
        colorSensor2 = hwMap.get(NormalizedColorSensor.class, "rightLine");
        // bottomRing = hwMap.get(RevColorSensorV3.class, "bottomRing");
        //topRing = hwMap.get(RevColorSensorV3.class, "topRing");
        bottomRing = hwMap.get(DistanceSensor.class, "bottomRing");
        topRing = hwMap.get(DistanceSensor.class, "topRing");
        wobbleSensor = hwMap.get(DistanceSensor.class, "wobbleSensor");

        lowSwitch1 = hwMap.get(TouchSensor.class, "limit_low1");
        highSwitch1 = hwMap.get(TouchSensor.class, "limit_hi1");
        lowSwitch2 = hwMap.get(TouchSensor.class, "limit_low2");
        highSwitch2 = hwMap.get(TouchSensor.class, "limit_hi2");

        ringPusher.setPosition(0);      // reset ring pussher arm
        storageServo.setPosition(1.0);  // lower storage
        gripperServo.setPosition(0);
        /*
        lifting.setTargetPosition(lifting.getCurrentPosition() + 1000);
        lifting.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifting.setPower(.6);
        while (lifting.isBusy() && lowSwitch1.isPressed() == false && lowSwitch2.isPressed() == false) {}
        lifting.setPower(0);
        lifting.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        */

    }

    //****************************************************************************************************
//****************************************************************************************************
    // go distance function - obsolete,use GoDistanceCM2 for normal driving or goDistanceacceleration for acceleration
    public void GoDistanceCM(int centimeters, double power, LinearOpMode linearOpMode) {
        // holds the conversion factor for ticks to centimeters
        // 27.55 for 3 3:1 Cartridges
        final double conversion_factor = 21.4;

        // sets the power negative if the distance is negative
        if (centimeters < 0 && power > 0) {
            power = power * -1;
        }

        // calculates the target amount of motor ticks
        int TICKS = (int) Math.abs(Math.round(centimeters * conversion_factor));

        // Debug: Send telemetry message with calculated ticks;
        //systemTools.telemetry.addData("Calculated Counts =", TICKS);
        //   systemTools.telemetry.update();

        // Thread.sleep(2000); // debugging: allow time to view telemetry

        // Send telemetry message to signify robot waiting;
        //systemTools.telemetry.addLine();
        //systemTools.telemetry.addData("Status", "Resetting Encoders");
        //   systemTools.telemetry.update();

        resetDriveEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        // systemTools.telemetry.addLine();
        // systemTools.telemetry.addData("Initial pos.", "Starting at %7d :%7d :%7d :%7d",
        //         frontLeftMotor.getCurrentPosition(),
        //         frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
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
        // systemTools.telemetry.addLine();
        // systemTools.telemetry.addData("Final", "Starting at %7d :%7d :%7d :%7d",
        //         frontLeftMotor.getCurrentPosition(),
        //         frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        // systemTools.telemetry.update();

        /*
        systemTools.telemetry.addData("Path", "Complete");
        systemTools.telemetry.addData("counts", TICKS);
        systemTools.telemetry.update();

         */
    }

    // function for rotating the robot
    public void RotateDEG(int degrees, double power, LinearOpMode linearOpMode) {

        // conversion for degrees to ticks
        final double conversion_factor = 12.73;

        // if degrees are negative, set the power negative
        if (degrees < 0 && power > 0) {
            power = power * -1;
        }

        int TICKS = (int) Math.round(degrees * conversion_factor);
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        // systemTools.telemetry.addData("Status", "Resetting Encoders");
        // systemTools.telemetry.update();

        resetDriveEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        // systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
        //         frontLeftMotor.getCurrentPosition(),
        //         frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        // systemTools.telemetry.update();

        // set target position for all the motor encoders
        int FLtarget = frontLeftMotor.getCurrentPosition() + TICKS;
        int FRtarget = frontRightMotor.getCurrentPosition() - TICKS;
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
                (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {
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

        // systemTools.telemetry.addData("Path", "Complete");
        // systemTools.telemetry.addData("counts", TICKS);
        // systemTools.telemetry.update();
    }

    // robot strafing function
    public void StrafeCM(int centimeters, double power, LinearOpMode linearOpMode) {

        // conversion factor between ticks and centimeters
        final double conversion_factor = 27.55;  //Corrected from 27.82 to 27.55 12-26-20

        // if the distance is negative, set power negative
        if (centimeters < 0 && power > 0) {
            power = power * -1;
        }

        int TICKS = (int) Math.abs(Math.round(centimeters * conversion_factor));

        // Send telemetry message to signify robot waiting;
        // systemTools.telemetry.addData("Status", "Resetting Encoders");
        // systemTools.telemetry.update();

        resetDriveEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        // systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
        //         frontLeftMotor.getCurrentPosition(),
        //         frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        // systemTools.telemetry.update();

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

        // systemTools.telemetry.addData("Path", "Complete");
        // systemTools.telemetry.addData("counts", TICKS);
        // systemTools.telemetry.update();
    }

    // find line function
    public void findLine(double power, LinearOpMode linearOpMode) {
        // Send telemetry message to signify robot waiting;
        // systemTools.telemetry.addData("Status", "Resetting Encoders");
        // systemTools.telemetry.update();

        resetDriveEncoders();


        // Send telemetry message to indicate successful Encoder reset
        // systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
        //         frontLeftMotor.getCurrentPosition(),
        //         frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        // systemTools.telemetry.update();

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

        // systemTools.telemetry.addData("Path", "Complete");
        // systemTools.telemetry.update();
    }

    /*  public char ringFinder() {
          char path;
          boolean sensor1Detected;
          boolean sensor2Detected;
          float gain = 2;
          int hueTarget = 30;
          final float[] hsvValues = new float[3];
          final float[] hsvValues2 = new float[3];

          systemTools.telemetry.addData("Gain", gain);

          // set gain on color sensors
          topRingColor.setGain(gain);
          bottomRingColor.setGain(gain);

          // get color sensors
          NormalizedRGBA colors2 = topRingColor.getNormalizedColors();
          NormalizedRGBA colors1 = bottomRingColor.getNormalizedColors();

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

     */
//**********************************************************************************************************************
    public char ringFinderDistance(LinearOpMode linearOpMode) {
        char path;

        // final float[] rgbValues = new float[3];

        //double maxTopRingDistCM = 2.9;// updated from 2.9 to 6 when changing to 2m Distance sensor 1-27-2021
        double maxTopRingDistCM = 11;
        double maxBotRingDistCM = 11;  // updated from 4 to 6 when changing to 2m Distance sensor 1-27-2021

        double bottomRingValueCM;
        double topRingValueCM;

        boolean bottomRingDetected;
        boolean topRingDetected;
        // float gain = 2;

        //  systemTools.telemetry.addData("Gain", gain);

        // bottomRing.setGain(gain);
        // topRing.setGain(gain);

        bottomRingValueCM = bottomRing.getDistance(DistanceUnit.CM);
        topRingValueCM = topRing.getDistance(DistanceUnit.CM);

        bottomRingDetected = bottomRingValueCM < maxBotRingDistCM;
        topRingDetected = topRingValueCM < maxTopRingDistCM;

        if (bottomRingDetected && topRingDetected) {
            path = 'C';
        } else if (bottomRingDetected && !topRingDetected) {
            path = 'B';
        } else if (!bottomRingDetected && !topRingDetected) {
            path = 'A';
        } else { // Means there was an error
            path = 'E';
        }

        // systemTools.telemetry.addData("Sensor 1 Distance (CM): ", bottomRingValueCM);
        // systemTools.telemetry.addData("Sensor 2 Distance (CM): ", topRingValueCM);
        // systemTools.telemetry.addData("Maximum Top Ring Distance (CM): ", maxTopRingDistCM + "Maximum Bottom Ring Distance (CM): ", maxBotRingDistCM);
        // systemTools.telemetry.addData("Path: ", path);
        // systemTools.telemetry.update();
        //   while (linearOpMode.opModeIsActive()) { }
        return path;
    }

    //*******************************************************************************************************************************************************
//************************************************** motor functions *********************************************************************************
    public void stopDriveMotors() {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);


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

    public LightsStates updateLightsState(boolean lowSpeedActivated, OpMode opMode) {
        LightsStates prevLightsState = lightsState;
        if (lowSpeedActivated) {
            lightsState = LightsStates.FlashFreeze;
        } else if (highSwitch1.isPressed() || highSwitch2.isPressed() || lowSwitch1.isPressed() || lowSwitch2.isPressed()) {
            lightsState = LightsStates.GrabberLimit;
        }
        else if (opMode.getRuntime() >= 80) {
            lightsState = LightsStates.FourtySecs;
        } else if (opMode.getRuntime() >= 70) {
            lightsState = LightsStates.FiftySecs;
        } else if (opMode.getRuntime() >= 60) {
            lightsState = LightsStates.SixtySecs;
        } else {
            lightsState = LightsStates.Normal;
        }
        if (lightsState != prevLightsState) {
            updateLightsToState();
        }
        return lightsState;
    }

    public void updateLightsToState() {
        switch (lightsState) {
            case Off:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                break;
            case FourtySecs:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);
                break;
            case FiftySecs:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                break;
            case SixtySecs:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
                break;
            case LowBattery:
                // Don't quite know how do this yet as it wasn't in last years code
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                break;
            case FlashFreeze:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);
                break;
            case GrabberLimit:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                break;
            case Normal:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                break;
            case Custom:
                break;
        }
    }

    public void setLightsPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        lights.setPattern(pattern);
        lightsState = LightsStates.Custom;
    }

    public void moveGripper(boolean close) {
        gripperServo.setDirection(Servo.Direction.REVERSE);
        gripperServo.scaleRange(0, 1.0);
        if (close) {
            gripperServo.setPosition(1);
        } else {
            gripperServo.setPosition(0);
        }
    }
//***************************************************************************************************
    //**************   GRIPPER FUNCTIONS  *************************************************************

    public void gripperClose() {
        gripperServo.setDirection(Servo.Direction.REVERSE);
        gripperServo.scaleRange(0, 1.0);
        gripperServo.setPosition(1);
    }

    public void gripperOpen() {
        gripperServo.setDirection(Servo.Direction.REVERSE);
        gripperServo.scaleRange(0, 1.0);
        gripperServo.setPosition(0);
    }

    public void raiseGripper(int maxTicks) {
        lifting.setTargetPosition(lifting.getCurrentPosition() - maxTicks); // maxTicks was 850
        lifting.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifting.setPower(-1);
        while (lifting.isBusy() && highSwitch1.isPressed() == false && highSwitch2.isPressed() == false) {
        }
        lifting.setPower(0);
    }

    public void lowerGripper(int maxTicks) {
        lifting.setTargetPosition(lifting.getCurrentPosition() + maxTicks); // maxTicks was 2000
        lifting.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifting.setPower(1);
        while (lifting.isBusy() && lowSwitch1.isPressed() == false && lowSwitch2.isPressed() == false) {
        }
        lifting.setPower(0);
    }

    public void moveArm(States autoLifterState) {
        if (!lowSwitch1.isPressed() && !lowSwitch2.isPressed() && !highSwitch1.isPressed() && !highSwitch2.isPressed()) {
            if (autoLifterState == States.Backwards) {
                lifting.setTargetPosition(lifting.getCurrentPosition() - 850);
                lifting.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lifting.setPower(-0.8);
            } else if (autoLifterState == States.Forwards) {
                lifting.setTargetPosition(lifting.getCurrentPosition() + 850);
                lifting.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lifting.setPower(.5);
            }

        }
    }

    public void GoDistanceTICKS2(int ticks, double power, LinearOpMode linearOpMode) {


        // Send telemetry message to signify robot waiting;
        // systemTools.telemetry.addData("Status", "Resetting Encoders");
        // systemTools.telemetry.update();

        resetDriveEncoders();

        startDriveEncoders();
        // Send telemetry message to indicate successful Encoder reset
        // systemTools.telemetry.addLine();
        // systemTools.telemetry.addData("Initial pos.", "Starting at %7d :%7d :%7d :%7d",
        //         frontLeftMotor.getCurrentPosition(),
        //         frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());

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
        // systemTools.telemetry.addLine();
        // systemTools.telemetry.addData("Final pos.", "Starting at %7d :%7d :%7d :%7d",
        //         frontLeftMotor.getCurrentPosition(),
        //         frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());

        // systemTools.telemetry.addLine().addData("Path", "Complete");
        // systemTools.telemetry.update();
    }


    public void GoDistanceTICKS(int ticks, double power, LinearOpMode linearOpMode) {


        // Send telemetry message to signify robot waiting;
        // systemTools.telemetry.addData("Status", "Resetting Encoders");
        // systemTools.telemetry.update();

        resetDriveEncoders();

        // Send telemetry message to indicate successful Encoder reset
        // systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
        //         frontLeftMotor.getCurrentPosition(),
        //         frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        // systemTools.telemetry.update();

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

        //systemTools.telemetry.addData("Path", "Complete");
        //systemTools.telemetry.update();
    }

    //*********************************************************************************************
    // go distance function

    //*************************************************************************************************
    //************************* Driving & Strafing funtions ******************************************************
    //******************** go distance function  *************************************************
    //  This the latest used in autonomous along with GoDistance Acceleration

    public void GoDistanceCM2(int centimeters, double power, boolean handoff, LinearOpMode linearOpMode) {

        // holds the conversion factor for TICKS to centimeters
        // 27.55 for 3 3:1 cartridges
        final double conversion_factor = 21.4;

        // sets the power negative if the distance is negative
        if (centimeters < 0 && power > 0) {
            power = power * -1;
        }

        // calculates the target amount of motor TICKS
        int TICKS = (int) Math.round(centimeters * conversion_factor);

        // Debug: Send telemetry message with calculated TICKS;
        // systemTools.telemetry.addData("Calculated Counts =", TICKS);
        //   systemTools.telemetry.update();


        // Send telemetry message to signify robot waiting;
        // systemTools.telemetry.addLine();
        // systemTools.telemetry.addData("Status", "Resetting Encoders");
        //   systemTools.telemetry.update();

        resetDriveEncoders();

//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        // systemTools.telemetry.addLine();
        // systemTools.telemetry.addData("Initial pos.", "Starting at %7d :%7d :%7d :%7d",
        //         frontLeftMotor.getCurrentPosition(),
        //         frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
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
                (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {
        }

        if (!handoff) stopDriveMotors();

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
        // systemTools.telemetry.addLine();
        // systemTools.telemetry.addData("Final", "Starting at %7d :%7d :%7d :%7d",
        //         frontLeftMotor.getCurrentPosition(),
        //         frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        // systemTools.telemetry.update();

        //while (linearOpMode.opModeIsActive()) {}  //  Empty while loop - program waits until user terminates op-mode

        /*
        systemTools.telemetry.addData("Path", "Complete");
        systemTools.telemetry.addData("counts", TICKS);
        systemTools.telemetry.update();

         */
    }

    public void StrafeCM2(int centimeters, double power, LinearOpMode linearOpMode) {
        // holds the conversion factor for TICKS to centimeters
        final double conversion_factor = 27.55;

        // sets the power negative if the distance is negative
        if (centimeters < 0 && power > 0) {
            power = power * -1;
        }

        // calculates the target amount of motor TICKS
        int TICKS = (int) Math.abs(Math.round(centimeters * conversion_factor));

        // Debug: Send telemetry message with calculated TICKS;
        // systemTools.telemetry.addData("Calculated Counts =", TICKS);
        //   systemTools.telemetry.update();


        // Send telemetry message to signify robot waiting;
        // systemTools.telemetry.addLine();
        // systemTools.telemetry.addData("Status", "Resetting Encoders");
        //   systemTools.telemetry.update();

        resetDriveEncoders();

//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        // systemTools.telemetry.addLine();
        // systemTools.telemetry.addData("Initial pos.", "Starting at %7d :%7d :%7d :%7d",
        //         frontLeftMotor.getCurrentPosition(),
        //         frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
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
        // systemTools.telemetry.addLine();
        // systemTools.telemetry.addData("Final", "Starting at %7d :%7d :%7d :%7d",
        //         frontLeftMotor.getCurrentPosition(),
        //         frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        // systemTools.telemetry.update();

        //while (linearOpMode.opModeIsActive()) {}  //  Empty while loop - program waits until user terminates op-mode

        /*
        systemTools.telemetry.addData("Path", "Complete");
        systemTools.telemetry.addData("counts", TICKS);
        systemTools.telemetry.update();

         */
    }

    public void StrafeDistanceCM(int centimeters, double power, LinearOpMode linearOpMode) {

        final double conversion_factor = 27.82;
        boolean left = centimeters > 0;
        int TICKS = (int) Math.round(centimeters * conversion_factor);
        int FLtarget = 0;
        int FRtarget = 0;
        int BLtarget = 0;
        int BRtarget = 0;

        power = Math.abs(power);


        // Send telemetry message to signify robot waiting;
        // systemTools.telemetry.addData("Status", "Resetting Encoders");
        // systemTools.telemetry.update();

        resetDriveEncoders();

        // Send telemetry message to indicate successful Encoder reset
       // systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
       //         frontLeftMotor.getCurrentPosition(),
       //         frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
       // systemTools.telemetry.update();

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

        // systemTools.telemetry.addData("Path", "Complete");
        // systemTools.telemetry.addData("counts", TICKS);
        // systemTools.telemetry.update();
    }

    public void flywheel(boolean on, double onPower) {
        if (on) {
            shooting.setPower(onPower);
        } else {
            shooting.setPower(0);
        }
    }

    public void intake(States setting) {
        if (setting == States.Off) {
            intake.setPower(0);
        } else if (setting == States.Forwards) {
            intake.setPower(1);
        } else if (setting == States.Backwards) {
            intake.setPower(-1);
        }
    }

    public void storage(boolean up, LinearOpMode linearOpMode) {
        if (up) {
            storageServo.setPosition(0);
        } else {
            storageServo.setPosition(1);
        }
    }

    public void pushRing(double timeout, LinearOpMode linearOpMode) {
        //double initial = linearOpMode.getRuntime();
        ringPusher.setPosition(1);

        /* the following 2-lines of code do not execute properly. as soon as the time is checked and it is false, the function exits
        since it is not called recursively, the push arm is never reset.  Use a while loop here
        if (linearOpMode.getRuntime() - initial > timeout) {
            ringPusher.setPosition(0);
         */
        double initial = linearOpMode.getRuntime();
        while (linearOpMode.getRuntime() - initial < 0.35) {
        }
        ringPusher.setPosition(0);
    }

    //*********************************************************************************************************************************
    public void strafeDistanceCM2(int centimeters, double power, boolean handoff, LinearOpMode linearOpMode) {
        //   this method is used for strafing a controlled distance in autonomous
        //
        //double conversion_factor = 31.3;  old conversion factor using 3x3x3 cartridges on the drive motor
        double conversion_factor = 24.0;  // new conversion factor using 4x5 gear cartridges

        boolean left = centimeters < 0;
        int TICKS = (int) Math.abs(Math.round(centimeters * conversion_factor));
        int FLtarget = 0;
        int FRtarget = 0;
        int BLtarget = 0;
        int BRtarget = 0;

        power = Math.abs(power);


        // Send telemetry message to signify robot waiting;
        // systemTools.telemetry.addData("Status", "Resetting Encoders");
        // systemTools.telemetry.update();

        resetDriveEncoders();

        // Send telemetry message to indicate successful Encoder reset
        // systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
        //         frontLeftMotor.getCurrentPosition(),
        //         frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        // systemTools.telemetry.update();

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

        // start motion

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

        if (!handoff) stopDriveMotors();

        startDriveEncoders();

        // systemTools.telemetry.addData("Path", "Complete");
        // systemTools.telemetry.addData("counts", TICKS);
        // systemTools.telemetry.update();
    }

    public void strafeDistanceCM3(int centimeters, double power, boolean handoff) {
        //****************************************************************************
        //   This method is used for strafing a controlled distance in Teleop mode
        //
        //double conversion_factor = 31.3;  old conversion factor using 3x3x3 cartridges on the drive motor
        double conversion_factor = 24.48;  // new conversion factor using 4x5 gear cartridges
        //This method is used for TeleOp
        //was 31.3 for 3 3:1 cartridges

        boolean left = centimeters < 0;
        int TICKS = (int) Math.abs(Math.round(centimeters * conversion_factor));
        int FLtarget = 0;
        int FRtarget = 0;
        int BLtarget = 0;
        int BRtarget = 0;

        power = Math.abs(power);

        resetDriveEncoders();

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


        // start motion

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
        while (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy()) {
        }

        if (!handoff) stopDriveMotors();

        startDriveEncoders();

    }

    //************************************************************************************************
    //************************************************************************************************
    public void goDistanceAcceleration(int centimeters, double power, boolean handoff, double frontRamp, double backRamp, LinearOpMode linearOpMode) {

        // IMPORTANT: for backramp, subtract the percent from 100. For example, if you want the robot to ramp down for the last 30.0 percent, set it to 70.0

        // holds the conversion factor for TICKS to centimeters
        // holds the conversion factor for TICKS to centimeters

        //final double conversion_factor = 27.55; // old conversion factor using 3x3x3 cartridges on the drive motor
        final double conversion_factor = 22;  // new conversion factor using 4x5 gear cartridges
        double setPower = 0.0;
        double percent;
        double percent2;
        boolean backwards;


        // sets the power negative if the distance is negative
        if (centimeters < 0 && power > 0) {
            power = power * -1;
        }
        backwards = power < 0;
        power = Math.abs(power);

        // calculates the target amount of motor TICKS
        int TICKS = (int) Math.round(centimeters * conversion_factor);


        // Debug: Send telemetry message with calculated TICKS;
        // systemTools.telemetry.addData("Calculated Counts =", TICKS);
        //   systemTools.telemetry.update();


        // Send telemetry message to signify robot waiting;
        // systemTools.telemetry.addLine();
        // systemTools.telemetry.addData("Status", "Resetting Encoders");
        //   systemTools.telemetry.update();

        resetDriveEncoders();

//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        // systemTools.telemetry.addLine();
        // systemTools.telemetry.addData("Initial pos.", "Starting at %7d :%7d :%7d :%7d",
        //         frontLeftMotor.getCurrentPosition(),
        //         frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
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

        // Sets the motors to start running to the position they have been given
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


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
                (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {

            // Finds out how far into the motion we are (0-100)
            double fLpercent = (double) (frontLeftMotor.getCurrentPosition()) / frontLeftMotor.getTargetPosition() * 100;


            if (fLpercent <= frontRamp) { // front ramp was 30.0
                percent = fLpercent / frontRamp; // Finds out how far into the front ramp (0-1)
                setPower = percent * power; // accelerates from 0-max power

                // Set minimum power to .1 to get the robot started moving
                if (setPower < 0.1) {
                    setPower = 0.1;
                }
            }
            if (fLpercent > frontRamp && fLpercent < backRamp) {
                setPower = power; // power stays at max in the middle of the course
            }
            if (fLpercent >= backRamp) { // back ramp was 70.0
                // Finds out how far into the back ramp(0-backRamp)
                percent2 = fLpercent - backRamp;
                // converts that to a percent on the backramp left (0-1)
                percent = percent2 / (100.0 - backRamp);
                setPower = (1 - percent) * power; // power decreases to zero at the end
            }


            // set the power the motors need to be going at
            if (!backwards) {
                frontLeftMotor.setPower(setPower);
                frontRightMotor.setPower(setPower);
                backRightMotor.setPower(setPower);
                backLeftMotor.setPower(setPower);
            } else {
                frontLeftMotor.setPower(-setPower);
                frontRightMotor.setPower(-setPower);
                backRightMotor.setPower(-setPower);
                backLeftMotor.setPower(-setPower);
            }
        }

        if (!handoff) stopDriveMotors();

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
        // systemTools.telemetry.addLine();
        // systemTools.telemetry.addData("Final", "Starting at %7d :%7d :%7d :%7d",
        //         frontLeftMotor.getCurrentPosition(),
        //         frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        // systemTools.telemetry.update();

        //while (linearOpMode.opModeIsActive()) {}  //  Empty while loop - program waits until user terminates op-mode

        /*
        systemTools.telemetry.addData("Path", "Complete");
        systemTools.telemetry.addData("counts", TICKS);
        systemTools.telemetry.update();

         */
    }


    public void strafeAcceleration(int centimeters, double power, boolean handoff, double frontRamp, double backRamp, LinearOpMode linearOpMode) {

        // IMPORTANT: for backramp, subtract the percent from 100. For example, if you want the robot to ramp down for the last 30.0 percent, set it to 70.0

        // holds the conversion factor for TICKS to centimeters
        final double conversion_factor = 24;
        double setPower = 0.0;
        double percent;
        double percent2;
        int FLtarget = 0;
        int FRtarget = 0;
        int BLtarget = 0;
        int BRtarget = 0;
        boolean left = centimeters < 0;

        centimeters = Math.abs(centimeters);
        power = Math.abs(power);

        // calculates the target amount of motor TICKS
        int TICKS = (int) Math.round(centimeters * conversion_factor);


        // Debug: Send telemetry message with calculated TICKS;
        // systemTools.telemetry.addData("Calculated Counts =", TICKS);
        //   systemTools.telemetry.update();


        // Send telemetry message to signify robot waiting;
        // systemTools.telemetry.addLine();
        // systemTools.telemetry.addData("Status", "Resetting Encoders");
        //   systemTools.telemetry.update();

        resetDriveEncoders();

//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        // systemTools.telemetry.addLine();
        // systemTools.telemetry.addData("Initial pos.", "Starting at %7d :%7d :%7d :%7d",
        //         frontLeftMotor.getCurrentPosition(),
        //         frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        //  systemTools.telemetry.update();


        // sets the target position for each of the motor encoders
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

        // Sets the motors to start running to the position they have been given
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


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
                (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {

            // Finds out how far into the motion we are (0-100)
            double fLpercent = (double) (frontLeftMotor.getCurrentPosition()) / frontLeftMotor.getTargetPosition() * 100;


            if (fLpercent <= frontRamp) { // front ramp was 30.0
                percent = fLpercent / frontRamp; // Finds out how far into the front ramp (0-1)
                setPower = percent * power; // accelerates from 0-max power

                // Set minimum power to .1 to get the robot started moving
                if (setPower < 0.1) {
                    setPower = 0.1;
                }
            }
            if (fLpercent > frontRamp && fLpercent < backRamp) {
                setPower = power; // power stays at max in the middle of the course
            }
            if (fLpercent >= backRamp) { // back ramp was 70.0
                // Finds out how far into the back ramp(0-backRamp)
                percent2 = fLpercent - backRamp;
                // converts that to a percent on the backramp left (0-1)
                percent = percent2 / (100.0 - backRamp);
                setPower = (1 - percent) * power; // power decreases to zero at the end
                //set minimum power to 0.05 to allow the robot to actually hit the target
                if (setPower < 0.05) {
                    setPower = 0.05;
                }
            }


            // set the power the motors need to be going at
            if (left) {
                frontLeftMotor.setPower(-setPower);
                frontRightMotor.setPower(setPower);
                backRightMotor.setPower(-setPower);
                backLeftMotor.setPower(setPower);
            } else {
                frontLeftMotor.setPower(setPower);
                frontRightMotor.setPower(-setPower);
                backRightMotor.setPower(setPower);
                backLeftMotor.setPower(-setPower);
            }
        }

        if (!handoff) stopDriveMotors();

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
        // systemTools.telemetry.addLine();
        // systemTools.telemetry.addData("Final", "Starting at %7d :%7d :%7d :%7d",
        //         frontLeftMotor.getCurrentPosition(),
        //         frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        // systemTools.telemetry.update();

        //while (linearOpMode.opModeIsActive()) {}  //  Empty while loop - program waits until user terminates op-mode

        /*
        systemTools.telemetry.addData("Path", "Complete");
        systemTools.telemetry.addData("counts", TICKS);
        systemTools.telemetry.update();

         */
    }


    public void wait(long timeout, LinearOpMode linearOpMode) {
        linearOpMode.sleep(timeout);
    }


        public void wobble(double liftingPower) {
            if (armState == States.Off) { // don't do manual movements if moving automatically


//           if (!robot.lowSwitch1.isPressed() && !robot.lowSwitch2.isPressed() && !robot.highSwitch1.isPressed() && !robot.highSwitch2.isPressed()) {
//               liftingPower = y2/2;
//           }
                if (lowSwitch1.isPressed() || lowSwitch2.isPressed()) {
                    lifting.setPower(Range.clip(liftingPower, -.5, 0));
                    //robot.lifting.setPower(Range.clip(liftingPower, 0,0.5));
                } else if (highSwitch1.isPressed() || highSwitch2.isPressed()) {
                    lifting.setPower(Range.clip(liftingPower, 0, 0.3));
                } else {
                    lifting.setPower(liftingPower);
                }

            } else {
                if (armState == States.Forwards) {
                    // Don't break the robot check
                    if (highSwitch1.isPressed() || highSwitch2.isPressed()) {
                        lifting.setPower(0);
                        lifting.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        armState = States.Off;
                    }
                } else { // lifter is going backwards, aka down
                    // Don't break the robot check
                    if (lowSwitch1.isPressed() || lowSwitch2.isPressed()) {
                        lifting.setPower(0);
                        lifting.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        armState = States.Off;
                    }
                }
                if (!lifting.isBusy()) {
                    lifting.setPower(0);
                    lifting.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    armState = States.Off;
                }
            }
        }
    public static class wobble extends ProgrammingFrame {

        public static boolean success = false;
        public static int rotateBack;
    }
//***************************************************************************************************

    public void wobbleFind(int degrees, double power, double difference, LinearOpMode linearOpMode) {
        double distance = 100;
        // conversion for degrees to ticks
        final double conversion_factor = 12.73;

        // if degrees are negative, set the power negative
        if (degrees < 0 && power > 0) {
            power = power * -1;
        }

        int TICKS = (int) Math.round(degrees * conversion_factor);
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        // systemTools.telemetry.addData("Status", "Resetting Encoders");
        // systemTools.telemetry.update();

        resetDriveEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        // systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
        //         frontLeftMotor.getCurrentPosition(),
        //         frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        // systemTools.telemetry.update();

        int FLstart = frontLeftMotor.getCurrentPosition();

        // set target position for all the motor encoders
        int FLtarget = frontLeftMotor.getCurrentPosition() + TICKS;
        int FRtarget = frontRightMotor.getCurrentPosition() - TICKS;
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


        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (linearOpMode.opModeIsActive() &&
                (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {

            distance = wobbleSensor.getDistance(DistanceUnit.CM);
            // systemTools.telemetry.addData("Distance= ", "%.3f%n", distance);
            // systemTools.telemetry.update();
            // reset the timeout time and start motion.
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(-power);
            backRightMotor.setPower(-power);
            backLeftMotor.setPower(power);


            if (distance < difference) {
                wobble.success = true;
                for (int i = 1; i <= 20000; ++i) {}        // waste some time to allow robot to turn more and align gripper with wobble.
                                                            // this saves over 0.5 seconds over adding another 3 degree turn

               // frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + 12);
               // frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() - 12);
               // backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + 12);
               // backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() - 12);

                break;
            }
        }
        stopDriveMotors();

        // calculate change after entire drive
        int FLdelta2 = frontLeftMotor.getCurrentPosition() - FLstart;
        int rotateBackDeg2 = (int) (FLdelta2 / conversion_factor);


//        frontLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        backRightMotor.setPower(0);
//        backLeftMotor.setPower(0);

        startDriveEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // systemTools.telemetry.addData("Path", "Complete");
        // systemTools.telemetry.addData("counts", TICKS);
        // systemTools.telemetry.update();
//

        wobble.rotateBack = rotateBackDeg2;

    }  // end wobble find
}   //end programming frame






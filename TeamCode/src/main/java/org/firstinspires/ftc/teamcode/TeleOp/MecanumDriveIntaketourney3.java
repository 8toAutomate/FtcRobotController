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

package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@TeleOp(name="MecanumDriveIntakeTourney3", group="Iterative Opmode")
//@Disabled
public class MecanumDriveIntaketourney3 extends OpMode
{

    ProgrammingFrame robot   = new ProgrammingFrame();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    double initialSH;  //initial time for shotting button timer
    double initialST;  //initial time for storage button timer
    double initialFL;  //initial time for flywheel button timer
    double initialSR;  //initial time for right strafe 20
    //double initialIN;  //initial time for intake button timer
    // Setup a variable for each drive wheel to save power level for telemetry
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;
    // Setup a variable for strafing constant
    double strafingConstant = 1.5;
    // Setup boolean variables and states
    boolean isIntakeOn = false;
    boolean isAPressed = false;
    enum States {
        Forwards, Backwards, Off, On
    }
    States autoLifterState = States.Off;
    States ringPusher = States.Backwards;
    States flywheel = States.Off;
    States intakeState = States.Off;
    States intakeButtonState = States.Off;
    boolean gripperClosed = true;
    boolean gripperMoving = false;
    boolean gripperPressed = false;
    boolean gripperRaised = false;
    boolean lowSpeedActivated = false;
    boolean driverAButtonDown = false;
    boolean xClick, yClick = false;
    boolean shooting = false;   // Flag  is true when shooting process is in progress
    boolean shootButton = false;   // shoot button status flag -  true  = button was pressed
    boolean shootingReset = true;  // shooting arm return flag - false = shooting arm reset in process
    boolean storageUp = false;
    boolean movingStorage = false;
    boolean storagePressed = false;
    boolean flyWheel, flyMotor, flyWheel2 = false;
    boolean strafe20,rtClick = false;
    boolean shootingReverse = false;
    boolean whackerServoRunning = false;
    boolean leftBumperDown = false;
    double liftingPower;
    int travelDist;
    double wStartTime;
    boolean wobbleFound = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void moveGripper(boolean close) {
        robot.gripperServo.setDirection(Servo.Direction.REVERSE);
        robot.gripperServo.scaleRange(0, 0.93);
        if (close) {
            robot.gripperServo.setPosition(1);
        }
        else {
            robot.gripperServo.setPosition(0);
        }
    }

    // exponential function for joystick inputs
    public double exponential(double value, int constant) {
        double cubed =  value*value*value;
        return cubed * constant;
    }

    public void raiseGripper() {
        autoLifterState = States.Forwards;
        robot.lifting.setTargetPosition(robot.lifting.getCurrentPosition() - 800);
        robot.lifting.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lifting.setPower(-0.8);
        // while (robot.lifting.isBusy() && robot.highSwitch1.isPressed() == false && robot.highSwitch2.isPressed() == false) {}
        // robot.lifting.setPower(0);
        // robot.lifting.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void lowerGripper() {
        autoLifterState = States.Backwards;
        robot.lifting.setTargetPosition(robot.lifting.getCurrentPosition() + 800);
        robot.lifting.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lifting.setPower(.5);
        // while (robot.lifting.isBusy() && robot.lowSwitch1.isPressed() == false && robot.lowSwitch2.isPressed() == false) {}
        // robot.lifting.setPower(0);
        // robot.lifting.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void wobbleFind(int degrees, double power, double difference) {
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

        robot.resetDriveEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        // systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
        //         frontLeftMotor.getCurrentPosition(),
        //         frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        // systemTools.telemetry.update();

        int FLstart = robot.frontLeftMotor.getCurrentPosition();

        // set target position for all the motor encoders
        int FLtarget = robot.frontLeftMotor.getCurrentPosition() + TICKS;
        int FRtarget = robot.frontRightMotor.getCurrentPosition() - TICKS;
        int BLtarget = robot.backLeftMotor.getCurrentPosition() + TICKS;
        int BRtarget = robot.backRightMotor.getCurrentPosition() - TICKS;

        robot.frontLeftMotor.setTargetPosition(FLtarget);
        robot.frontRightMotor.setTargetPosition(FRtarget);
        robot.backLeftMotor.setTargetPosition(BLtarget);
        robot.backRightMotor.setTargetPosition(BRtarget);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy())) {

            distance = robot.wobbleSensor.getDistance(DistanceUnit.CM);
            // systemTools.telemetry.addData("Distance= ", "%.3f%n", distance);
            // systemTools.telemetry.update();
            // reset the timeout time and start motion.
            robot.frontLeftMotor.setPower(power);
            robot.frontRightMotor.setPower(-power);
            robot.backRightMotor.setPower(-power);
            robot.backLeftMotor.setPower(power);


            if (distance < difference) {
                ProgrammingFrame.wobble.success = true;
                for (int i = 1; i <= 20000; ++i) {}        // waste some time to allow robot to turn more and align gripper with wobble.
                // this saves over 0.5 seconds over adding another 3 degree turn

                // frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + 12);
                // frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() - 12);
                // backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + 12);
                // backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() - 12);

                break;
            }
        }
        robot.stopDriveMotors();

        // calculate change after entire drive
        int FLdelta2 = robot.frontLeftMotor.getCurrentPosition() - FLstart;
        int rotateBackDeg2 = (int) (FLdelta2 / conversion_factor);


//        frontLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        backRightMotor.setPower(0);
//        backLeftMotor.setPower(0);

        robot.startDriveEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // systemTools.telemetry.addData("Path", "Complete");
        // systemTools.telemetry.addData("counts", TICKS);
        // systemTools.telemetry.update();
//

        ProgrammingFrame.wobble.rotateBack = rotateBackDeg2;

    }  // end wobble find

    //*********************************************************************************************
    //wobbleFindTele (based on Wobblefind2 in prgramming frame used enhanced search for wobble and adjusts travel distance to grip wobble
    // best to place the wobble about 10-15 Cm in front of the robot about 1/3 of the way from the left.
    // call with 40-45 deg and 0.2 power and distance of 40 or 35.

    public void wobbleFindTele(int degrees, double power, double difference) {
        double distance = 100;
        // conversion for degrees to ticks
        final double conversion_factor = 12.73;
        boolean frontEdgeFound = false;  // lower flag for detecting  front edge.
        // if degrees are negative, set the power negative
        if (degrees < 0 && power > 0) {
            power = power * -1;
        }
        int wStart=0, wEnd=0;
        int TICKS = (int) Math.round(degrees * conversion_factor);
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        robot.resetDriveEncoders();

        int FLstart = robot.frontLeftMotor.getCurrentPosition();

        // set target position for all the motor encoders
        int FLtarget = robot.frontLeftMotor.getCurrentPosition() + TICKS;
        int FRtarget = robot.frontRightMotor.getCurrentPosition() - TICKS;
        int BLtarget = robot.backLeftMotor.getCurrentPosition() + TICKS;
        int BRtarget = robot.backRightMotor.getCurrentPosition() - TICKS;

        robot.frontLeftMotor.setTargetPosition(FLtarget);
        robot.frontRightMotor.setTargetPosition(FRtarget);
        robot.backLeftMotor.setTargetPosition(BLtarget);
        robot.backRightMotor.setTargetPosition(BRtarget);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        robot.frontLeftMotor.setPower(power);
        robot.frontRightMotor.setPower(-power);
        robot.backRightMotor.setPower(-power);
        robot.backLeftMotor.setPower(power);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.

        while (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy()) {
            distance = robot.wobbleSensor.getDistance(DistanceUnit.CM);
            frontEdgeFound = false;
            // reset the timeout time and start motion.

            if (distance < difference) {//  wobble found
                ProgrammingFrame.wobble.success = true;
                wStart = robot.frontLeftMotor.getCurrentPosition() ;
                frontEdgeFound = true;
                // for ( int i = 1; i < 10000; ++i) {sum += i;}   // sum = sum + i.  waste some time to allow robot to turn more and align gripper with wobble.
                //this saves over 0.5 seconds over adding another 3 degree turn
            }  // end if

            if (frontEdgeFound && distance > difference) {
                robot.stopDriveMotors();
                //   break;
            } // end if

        } // end while

        //stopDriveMotors();
        wEnd = robot.frontLeftMotor.getCurrentPosition();
        int FLdelta = Math.abs(wEnd - wStart);
        //telemetry.addData("wStart: ", wStart + "  wEnd: "+ wEnd + "  FLdelta2: " + FLdelta2);
        //int alignWobbleDeg = (int)((-FLdelta / conversion_factor)-8);
        int alignWobbleDeg = (int)(-FLdelta -7.5*conversion_factor);      // FLdelta is already in ticks so no need to change twice

        // startDriveEncoders();
        //telemetry.addData("rotateBackDeg2: ", rotateBackDeg2);
        //RotateDEG(-alignWobbleDeg,0.5);  // rotate robot to align gripper with wobble
        //************************************************
        //rotate robot back
        power = 0.4;
        if (alignWobbleDeg< 0 && power > 0) {
            power = power * -1;
        }// end if

        //TICKS = (int) Math.round(alignWobbleDeg * conversion_factor);
        TICKS = alignWobbleDeg;
        // resetDriveEncoders();

        // set target position for all the motor encoders
        FLtarget = robot.frontLeftMotor.getCurrentPosition() + TICKS;
        FRtarget = robot.frontRightMotor.getCurrentPosition() - TICKS;
        BLtarget = robot.backLeftMotor.getCurrentPosition() + TICKS;
        BRtarget = robot.backRightMotor.getCurrentPosition() - TICKS;

        robot.frontLeftMotor.setTargetPosition(FLtarget);
        robot.frontRightMotor.setTargetPosition(FRtarget);
        robot.backLeftMotor.setTargetPosition(BLtarget);
        robot.backRightMotor.setTargetPosition(BRtarget);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        robot.frontLeftMotor.setPower(power);
        robot.frontRightMotor.setPower(-power);
        robot.backRightMotor.setPower(-power);
        robot.backLeftMotor.setPower(power);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.

        while (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy()) {}
        robot.stopDriveMotors();

        //************************************************

        // startDriveEncoders();
        double startWobbleDist=robot.wobbleSensor.getDistance(DistanceUnit.CM);
         travelDist=6;
        if (startWobbleDist > 22.5 ||startWobbleDist < 21.5 ){
            travelDist = (int)(6.5 + (startWobbleDist-22.5));
        }
        if (travelDist >20d
        ) {
            //telemetry.addData("Error, travel distance exceeded. Travel distance:   ", ProgrammingFrame.wobble.travelDist);
           // telemetry.update();
            travelDist=3;
            ProgrammingFrame.wobble.success = false;
        }
        robot.startDriveEncoders();
            /*double startTime = getRuntime();
            if (getRuntime()-startTime>=0.3) {
                moveGripper(false);
                raiseGripper();
                GoDistanceCM(travelDist, .2);
            }
            */


    }  // end wobbleFindTele
//******************** go distance function  *************************************************
    //  This the latest used in autonomous along with GoDistance Acceleration

    public void GoDistanceCM(int centimeters, double power) {

        // holds the conversion factor for TICKS to centimeters
        // 27.55 for 3 3:1 cartridges
        final double conversion_factor = 21.4;

        // sets the power negative if the distance is negative
        if (centimeters < 0 && power > 0) {
            power = power * -1;
        }

        // calculates the target amount of motor TICKS
        int TICKS = (int) Math.round(centimeters * conversion_factor);

        robot.resetDriveEncoders();

        // sets the target position for each of the motor encoders
        int FLtarget = robot.frontLeftMotor.getCurrentPosition() + TICKS;
        int FRtarget = robot.frontRightMotor.getCurrentPosition() + TICKS;
        int BLtarget = robot.backLeftMotor.getCurrentPosition() + TICKS;
        int BRtarget = robot.backRightMotor.getCurrentPosition() + TICKS;

        robot.frontLeftMotor.setTargetPosition(FLtarget);
        robot.frontRightMotor.setTargetPosition(FRtarget);
        robot.backLeftMotor.setTargetPosition(BLtarget);
        robot.backRightMotor.setTargetPosition(BRtarget);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //startDriveEncoders();  // disabled 12-26-20 - This is not needed when driving by RUN_TO_POSITION.  Enabling this causes measurement errors

        // reset the timeout time and start motion.
        robot.frontLeftMotor.setPower(power);
        robot.frontRightMotor.setPower(power);
        robot.backRightMotor.setPower(power);
        robot.backLeftMotor.setPower(power);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.

        while (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy()) {
        }

        robot.stopDriveMotors();
        robot.startDriveEncoders();
    }

    public void square(int centimeters, double power, boolean handoff) {

        // holds the conversion factor for TICKS to centimeters
        // 27.55 for 3 3:1 cartridges
        final double conversion_factor = 21.4;
        boolean sensor1Detected;
        boolean sensor2Detected;
        float gain = 2;
        int hueTarget = 30;
        final float[] hsvValues = new float[3];
        final float[] hsvValues2 = new float[3];

        // sets the power negative if the distance is negative
        if (centimeters < 0 && power > 0) {
            power = power * -1;
        }

        // calculates the target amount of motor TICKS
        int TICKS = (int) Math.round(centimeters * conversion_factor);

        robot.resetDriveEncoders();

        // set gain on color sensors
        robot.topRingColor.setGain(gain);
        robot.bottomRingColor.setGain(gain);


        // sets the target position for each of the motor encoders
        int FLtarget = robot.frontLeftMotor.getCurrentPosition() + TICKS;
        int FRtarget = robot.frontRightMotor.getCurrentPosition() + TICKS;
        int BLtarget = robot.backLeftMotor.getCurrentPosition() + TICKS;
        int BRtarget = robot.backRightMotor.getCurrentPosition() + TICKS;

        robot.frontLeftMotor.setTargetPosition(FLtarget);
        robot.frontRightMotor.setTargetPosition(FRtarget);
        robot.backLeftMotor.setTargetPosition(BLtarget);
        robot.backRightMotor.setTargetPosition(BRtarget);

        // Testing this new function call instead of using RUN_TO_POSITION everywhere
        robot.startDriveEncodersTarget();

        // reset the timeout time and start motion.
        robot.frontLeftMotor.setPower(power);
        robot.frontRightMotor.setPower(power);
        robot.backRightMotor.setPower(power);
        robot.backLeftMotor.setPower(power);

        // keep looping while we are still active, and there is time left, and all motors are running.
        while (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy()) {

            // get color sensors
            NormalizedRGBA colors2 = robot.topRingColor.getNormalizedColors();
            NormalizedRGBA colors1 = robot.bottomRingColor.getNormalizedColors();
            Color.colorToHSV(colors1.toColor(), hsvValues);
            Color.colorToHSV(colors2.toColor(), hsvValues2);

            if (hsvValues[0] > hueTarget) {
                sensor1Detected = true;
                break;
            }

            if (hsvValues2[0] > hueTarget) {
                sensor2Detected = true;
                break;
            }

        }

        robot.stopDriveMotors();

        // sets the target position for each of the motor encoders
        FLtarget = robot.frontLeftMotor.getCurrentPosition() + TICKS;
        FRtarget = robot.frontRightMotor.getCurrentPosition() + TICKS;
        BLtarget = robot.backLeftMotor.getCurrentPosition() + TICKS;
        BRtarget = robot.backRightMotor.getCurrentPosition() + TICKS;

        robot.frontLeftMotor.setTargetPosition(FLtarget);
        robot.frontRightMotor.setTargetPosition(FRtarget);
        robot.backLeftMotor.setTargetPosition(BLtarget);
        robot.backRightMotor.setTargetPosition(BRtarget);

        // Testing this new function call instead of using RUN_TO_POSITION everywhere
        robot.startDriveEncodersTarget();

        // reset the timeout time and start motion.
        robot.frontLeftMotor.setPower(power);
        robot.frontRightMotor.setPower(power);
        robot.backRightMotor.setPower(power);
        robot.backLeftMotor.setPower(power);




        if (!handoff) robot.stopDriveMotors();

        // fem 12-24  debug       startDriveEncoders();

        // Send telemetry message to indicate successful Encoder reset
        // systemTools.telemetry.addLine();
        // systemTools.telemetry.addData("Final", "Starting at %7d :%7d :%7d :%7d",
        //         frontLeftMotor.getCurrentPosition(),
        //         frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        // systemTools.telemetry.update();
    }



//**************************************************************************************************


    @Override
    public void init() {
        robot.init(hardwareMap, this);
        robot.shooting.setPower(0);
        gamepad1.setJoystickDeadzone(.1f);
        gamepad2.setJoystickDeadzone(.1f);
        /*
        robot.gripperServo.setPosition(0);
        robot.lifting.setTargetPosition(robot.lifting.getCurrentPosition() + 1000);
        robot.lifting.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lifting.setPower(.8);
        while (robot.lifting.isBusy() && robot.lowSwitch1.isPressed() == false && robot.lowSwitch2.isPressed() == false) {}
        robot.lifting.setPower(0);
        robot.lifting.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
        runtime.reset();
        robot.lifting.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        initialSH = 0.6;
    }

    @Override
    public void loop() {
//********************************* Robot movement ********************************************************

        // controller variables
        // y: inverse of left stick's y value
        double y = exponential(-gamepad1.left_stick_y, 1);
        // x underscored: left stick's x value multiplied by the strafing coefficient in order to counteract imperfect strafing
        double x = exponential(gamepad1.left_stick_x,1);
        // rx: right stick's x value
        double rx = exponential(gamepad1.right_stick_x,1);

        double y2 = gamepad2.right_stick_y;

        liftingPower = y2/2;


        if (autoLifterState == States.Off) { // don't do manual movements if moving automatically


//           if (!robot.lowSwitch1.isPressed() && !robot.lowSwitch2.isPressed() && !robot.highSwitch1.isPressed() && !robot.highSwitch2.isPressed()) {
//               liftingPower = y2/2;
//           }
            if (robot.lowSwitch1.isPressed() || robot.lowSwitch2.isPressed()) {
                robot.lifting.setPower(Range.clip(liftingPower, -.7,0));
                //robot.lifting.setPower(Range.clip(liftingPower, 0,0.5));
            }
            else if (robot.highSwitch1.isPressed() || robot.highSwitch2.isPressed()) {
                robot.lifting.setPower(Range.clip(liftingPower,0, 1));
            }
            else {robot.lifting.setPower(liftingPower);}

        } else {
            if (autoLifterState == States.Forwards) {
                // Don't break the robot check
                if (robot.highSwitch1.isPressed() || robot.highSwitch2.isPressed()) {
                    robot.lifting.setPower(0);
                    robot.lifting.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    autoLifterState = States.Off;
                }
            } else {
                // lifter is going backwards, aka down
                // Don't break the robot check
                if (robot.lowSwitch1.isPressed() || robot.lowSwitch2.isPressed()) {
                    robot.lifting.setPower(0);
                    robot.lifting.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    autoLifterState = States.Off;
                }
            }
            if (!robot.lifting.isBusy()) {
                robot.lifting.setPower(0);
                robot.lifting.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                autoLifterState = States.Off;
            }
        }

        if (gamepad2.dpad_up && !robot.highSwitch1.isPressed() && !robot.highSwitch2.isPressed() && autoLifterState == States.Off) {
            raiseGripper();
        }

        if (gamepad2.dpad_down && !robot.lowSwitch1.isPressed() && !robot.lowSwitch2.isPressed() && autoLifterState == States.Off) {
            lowerGripper();
        }

        if (gamepad1.right_bumper) {
            if (!driverAButtonDown) {
                driverAButtonDown = true;
                lowSpeedActivated = !lowSpeedActivated;
            }
        } else {
            driverAButtonDown = false;
        }

        // for the programming frame
        // frontLeftPower = y + x + rx;
        // frontRightPower = y - x - rx;
        // backLeftPower = -y - x + rx;
        // backRightPower = -y + x - rx;

        // for actual robot
        frontLeftPower = y + x + rx;
        frontRightPower = y - x - rx;
        backLeftPower = y - x + rx;
        backRightPower = y + x - rx;

   //     liftingPower = y2/2;

        frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
        frontRightPower   = Range.clip(frontRightPower, -1.0, 1.0);
        backLeftPower = Range.clip(backLeftPower, -1.0, 1.0);
        backRightPower   = Range.clip(backRightPower, -1.0, 1.0);
        liftingPower = Range.clip(liftingPower, -1,1);

        if (lowSpeedActivated) {
            frontLeftPower /= 3;
            frontRightPower /= 3;
            backLeftPower /= 3;
            backRightPower /= 3;
        }

        robot.frontLeftMotor.setPower(frontLeftPower);
        robot.frontRightMotor.setPower(frontRightPower);
        robot.backLeftMotor.setPower(backLeftPower);
        robot.backRightMotor.setPower(backRightPower);

        robot.updateLightsState(lowSpeedActivated, this);

    /*    robot.lifting.setPower(liftingPower);

        if (!robot.lowSwitch1.isPressed() && !robot.lowSwitch2.isPressed()) {
            robot.lifting.setPower(y2/2);
        }
        if (robot.lowSwitch1.isPressed() || robot.lowSwitch2.isPressed()) {
            robot.lifting.setPower(Range.clip(liftingPower, -.5,0));
            //robot.lifting.setPower(Range.clip(liftingPower, 0,0.5));
        }
        if (robot.highSwitch1.isPressed() || robot.lowSwitch2.isPressed()) {
            robot.lifting.setPower(Range.clip(liftingPower,0, 0.3));
        }


     */
        //*******************Flywheel motor (shooting) *************************************************
      /*
        // shooting motors turn on by pressing the X key
         Original code for the flywheel.  It worked sometimes and most others it did not.

        //if (gamepad1.right_bumper) {
         //   robot.shooting.setPower(1);
         //   motorOff = false;
        //}

        if (gamepad1.x) {
            if (XClick == false) {
                XClick = true;
                if (flywheel == States.On) {
                    flywheel = States.Off;
                }
                else {
                    flywheel = States.On;
                }
            }
            else{
                XClick = false;
            }
        }

        if (flywheel == States.On) {
            robot.shooting.setPower(0.7);
        }else{
            robot.shooting.setPower(0);
            flywheel = States.Off;
        }
*/
        //Update 1-1-2021
        // This code turns the flywheel motor on or off.
        // This is similar to the storage box program except adapted for motor instead of servo

        // FLAGS:
        // flyWheel - holds the state of the Flywheel operation
        // flyMotor- holds if the flywheel motor is running or off
        // xClick - checks if flywheel button is pressed

        if (!flyWheel) { // checks if the flywheel is not already moving
            if (gamepad2.x) { // checks if the bumper is pressed
                xClick = true; // set X-button flag - X-button was pressed
                flyWheel = true; // Flywheel process has started
                //flyMotor = true;
                initialFL = getRuntime(); // gets current time
               // telemetry.addData("Status", "xClick " + gamepad1.x);
            }
        }

        if (xClick && flyWheel) { // checks if the flywheel is moving and if the storage pressed flag is raised
            if (!flyMotor) {robot.shooting.setPower(0.8);} // if the flywheel is off , turn it on
            else if (flyMotor) {robot.shooting.setPower(0);} // if the flywheel is on , turn it off
        }
        if (flyWheel) {
            if (getRuntime() - initialFL > .3) {
                flyWheel = !flyWheel; // updates state
                flyMotor = !flyMotor;
                xClick = false; // Flywheel button flag is lowered
            }
        }
        //*******************Flywheel motor (Power shot shooting) *************************************************
      /*

*/
        //Update 1-3-2021
        // This code turns the flywheel motor on or off.
        // This is similar to the Flywheel program but runs motor at lower power for power-shot

        // FLAGS:
        // flyWheel2 - holds the state of the Flywheel operation
        // flyMotor- holds if the flywheel motor is running or off
        // xClick - checks if flywheel button is pressed

        if (!flyWheel2) { // checks if the flywheel is not already moving
            if (gamepad2.y) { // checks if the bumper is pressed
                yClick = true; // set X-button flag - X-button was pressed
                flyWheel2 = true; // Flywheel process has started
                //flyMotor = true;
                initialFL = getRuntime(); // gets current time
             //   telemetry.addData("Status", "yClick " + gamepad1.y);
            }
        }

        if (yClick && flyWheel2) { // checks if the storage is moving and if the storage pressed flag is raised
            if (!flyMotor) {robot.shooting.setPower(0.71);} // if the flywheel is off , turn it on  was 0.7; is 0.73 max 2-24-21
            else if (flyMotor) {robot.shooting.setPower(0);} // // if the flywheel is on , turn it off
        }
        if (flyWheel2) {
            if (getRuntime() - initialFL > .3) {
                flyWheel2 = !flyWheel2; // updates state
                flyMotor = !flyMotor;
                yClick = false; // Flywheel button is lowered
            }
        }

        //********************Shooting Servo************************************************************
  /*      if (!shooting) {
            if (gamepad1.left_bumper) {
                shooting = true;
                initial = getRuntime();
                robot.ringPusher.setPosition(0.25);
            } else if (getRuntime() - initial > .5) {
                robot.ringPusher.setPosition(0);
                initial2 = getRuntime();
            } else if (getRuntime() - initial2 > .5) {
                shooting = false;
            }
        }

*/
        /*   The shooting arm servo advances to max position when the LB button is pressed.  Game timer is checked and
        after 0.5 sec the servo returns to original position getting ready for next shot.  There is an additional time delay of 0.5 seconds
        while servo is resetting back to original position.   Check for button press is disabled once the process is started
        to prevent retriggering mid cycle.   Using a button press status flag in the code is a better way than checking the button itself
        if the code is executed over several cycles to prevent unintentional retriggering the process

        Flags used:
        boolean shooting = false;   // Flag  is true when shooting process is in progress
        boolean shoot_button = false;   // shoot button status flag -  true  = button was pressed
        boolean shooting_reset = true;  // shooting arm return flag - false = shooting arm reset in process
        */

        if (!shooting) {                    // if shooting process has not started then check button for press
            if (!shootButton) {            // if button has been pressed then check current time and set
                if (gamepad2.right_bumper) { // shooting button and shooting process status flags to true.
                    initialSH = getRuntime();
                    shooting = true;
                    shootButton = true;
                } // end if gamepad.left_bumper
            } // end if !shoot_button
        } // end if !shooting

        if (shootButton){                              // check button status flag. If true
            if (shooting) {                             //  then check if shooting process is in progress
                robot.ringPusher.setPosition(0.7);        // run shooting servo to max position.
                    if (getRuntime() - initialSH > .4) {  // check if enough time has passed.  Time was 0.5 at first tourney
                        robot.ringPusher.setPosition(0); // return servo to starting position.
                        shooting = false;               //  Shooting is done.
                        shootingReset = false;         //  servo reset is not complete yet so set status flag false.
                        initialSH = getRuntime();        // check current time
                    }
            }
        }

        if(!shootingReset) {                   // check if shooting reset has completed.  If not
            if (getRuntime() - initialSH > .5) { // see if enough time has passed.  If true
                shootingReset = true;          // shooting reset process is complete
                shootButton = false;           // reset shoot button flag so it can be read on the next cycle
            }
        }

        //********************************* Intake motor ********************************************************

        /* Old toggle design
        if (gamepad1.right_trigger > .8) {
            if (intakeButtonState == States.Off) {
                if (intakeState == States.Forwards) {
                    intakeState = States.Off;
                    intakeButtonState = States.Forwards;
                }
            } if (intakeButtonState != States.Forwards) {
                intakeState = States.Forwards;
            }
            intakeButtonState = States.Forwards;
        } else if (gamepad1.left_trigger > .8) {
            if (intakeButtonState == States.Off) {
                if (intakeState == States.Backwards) {
                    intakeState = States.Off;
                    intakeButtonState = States.Backwards;
                }
            } if (intakeButtonState != States.Backwards) {
                intakeState = States.Backwards;
            }
            intakeButtonState = States.Backwards;
        } else {
            intakeButtonState = States.Off;
        } */

        if (gamepad1.right_trigger > .8) {
            intakeState = States.Forwards;
        } else if (gamepad1.left_trigger > .8) {
            intakeState = States.Backwards;
        } else {
            intakeState = States.Off;
        }

        if(intakeState == States.Forwards) {
            robot.intake.setPower(1);
        } else if (intakeState == States.Backwards) {
            robot.intake.setPower(-1);
        } else {
            robot.intake.setPower(0);
        }
        //********************************* Gripper and Gripper arm ************************************
        // logic version
        /*
        if (gamepad2.dpad_up && !gripperRaised) {
            raiseGripper();
            gripperRaised = true;
        }

        if (gamepad2.dpad_down && gripperRaised) {
            lowerGripper();
            gripperRaised = false;
        }

        if (gamepad2.dpad_left && !gripperClosed) {
            moveGripper(true);
            gripperClosed = true;
        }
        if (gamepad2.dpad_right && gripperClosed) {
            moveGripper(false);
            gripperClosed = false;
        }
        */

        // no logic version
        if (gamepad2.dpad_up && robot.highSwitch1.isPressed() == false && robot.highSwitch2.isPressed() == false) {
            raiseGripper();
        }

        if (gamepad2.dpad_down && robot.lowSwitch1.isPressed() == false && robot.lowSwitch2.isPressed() == false) {
            lowerGripper();
        }
/*

        if (gamepad2.dpad_right) {
            moveGripper(true);
        }
        if (gamepad2.dpad_left) {
            moveGripper(false);
        }
*/
        if (!gripperMoving) { // checks if the storage is not already moving
            if (gamepad2.b) { // checks if the b button is pressed
                gripperPressed = true; // raises storage pressed flag
                gripperMoving = true; // raises the moving storage flag
                initialST = getRuntime(); // gets current time
            }
        }
        if (gripperPressed && gripperMoving) { // checks if the storage is moving and if the storage pressed flag is raised
            if (!gripperClosed) { moveGripper(true); } // if the storage is not up it moves it up
            else if (gripperClosed) { moveGripper(false); } // if the storage is up it moves it down
        }
        if (gripperMoving) {
            if (getRuntime() - initialST > .3) {
                gripperPressed = false; // storage pressed flag is lowered
                gripperClosed = !gripperClosed; // updates state
                gripperMoving = false;
            }
        }

        if (gamepad1.left_bumper) {
            if (!leftBumperDown) {
                whackerServoRunning = !whackerServoRunning;
                leftBumperDown = true;
            }
        } else {
            leftBumperDown = false;
        }

        if (whackerServoRunning) {
            robot.whackerServo.setPower(1);
        } else {
            robot.whackerServo.setPower(0);
        }
        //********************************* Storage Servo **********************************************

        // this code raises and lowers the storage servo using the left bumper button.
        // This is similar to the ring pusher program, except it is implemented for toggle functionality.

        // FLAGS:
        // storageUp - holds the state of the storage
        // movingStorage - holds if the storage servo is in motion
        // storagePressed - checks if storage button is pressed

        if (!movingStorage) { // checks if the storage is not already moving
            if (gamepad2.left_bumper) { // checks if the b button is pressed
                storagePressed = true; // raises storage pressed flag
                movingStorage = true; // raises the moving storage flag
                initialST = getRuntime(); // gets current time
            }
        }
  /*      if (storagePressed && movingStorage) { // checks if the storage is moving and if the storage pressed flag is raised
            if (!storageUp) { robot.storageServo.setPosition(0); } // if the storage is not up it moves it up
            else if (storageUp) { robot.storageServo.setPosition(1); } // if the storage is up it moves it down
        }
        if (getRuntime() - initialST > .3) { // if half a second has passed since the storage has started moving (determined by when the flag is raised)
            movingStorage = false; // moving storage flag is lowered
        }
        if (getRuntime() - initialST > .5) { // .2 seconds after movingStorage = false;
            storagePressed = false; // storage pressed flag is lowered
            storageUp = !storageUp; // updates state
        }

*/
        if (storagePressed && movingStorage) { // checks if the storage is moving and if the storage pressed flag is raised
            if (!storageUp) { robot.storageServo.setPosition(0); } // if the storage is not up it moves it up
            else if (storageUp) { robot.storageServo.setPosition(1); } // if the storage is up it moves it down
        }
        if (movingStorage) {
            if (getRuntime() - initialST > .3) {
                storagePressed = false; // storage pressed flag is lowered
                storageUp = !storageUp; // updates state
                movingStorage = false;
            }
        }

        if (gamepad2.right_trigger > 0.5) {
            robot.strafeDistanceCM3(20,0.2,false);
        }
        else if (gamepad2.left_trigger > 0.5) {
            robot.strafeDistanceCM3(-20,0.2,false);
        }

//*******************************Reverse shooting while LT is held, and turn off when released*********************************************

        while (gamepad1.x) {
            robot.shooting.setPower(-1);
            if (shootingReverse = false); {
                robot.shooting.setPower(0);
            }
        }
        // manages state
        if (gamepad1.x) {
            shootingReverse = true;
        } else {
            shootingReverse = false;
        }

        //************************************************************************************************************

        if (gamepad1.y) {
            wobbleFindTele(40, 0.2, 40);
            wobbleFound = ProgrammingFrame.wobble.success;
            if (wobbleFound) {
                wStartTime = getRuntime();
                moveGripper(false);

        }
        //if (wobbleFound){

        //    moveGripper(false);
        //    gripperMoving=true;
        }
        if (wobbleFound) {
        //moveGripper(false);
            if (getRuntime() - wStartTime > 0.3) {
               raiseGripper();
                GoDistanceCM(travelDist, .2);
                wobbleFound=false;
                gripperClosed = !gripperClosed; // updates state
                //gripperMoving=false;
            }
        }
       // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Strafing constant", "Strafing Constant = " + strafingConstant);
        telemetry.addData("Motors", "front_left (%.2f), front_right (%.2f), back_left (%.2f), back_right (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.addData("Switch 1 Status", robot.lowSwitch1.isPressed());
        telemetry.addData("Switch 2 Status", robot.lowSwitch2.isPressed());
    }

    public void stop() {
        robot.stopAllMotors();
    }

}

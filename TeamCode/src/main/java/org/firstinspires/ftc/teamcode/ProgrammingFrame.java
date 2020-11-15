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

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;


public class ProgrammingFrame
{

    /* Public OpMode members. */
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;

    public NormalizedColorSensor colorSensor1;
    public NormalizedColorSensor colorSensor2;

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

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        // Set all motors to zero power
        stopDriveMotors();
//        frontLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        backLeftMotor.setPower(0);
//        backRightMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        startEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        colorSensor1 = hwMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor2 = hwMap.get(NormalizedColorSensor.class, "sensor_color");


    }

    public void GoDistanceCM(int centimeters, double power, LinearOpMode linearOpMode){

        final double conversion_factor = 8.46;
        if (centimeters < 0 && power > 0) {
            power = power * -1;
        }
        else if (centimeters > 0 && power < 0) {
            centimeters = centimeters * -1;
        }
        int TICKS = (int) Math.round(centimeters * conversion_factor);

        // Send telemetry message to signify robot waiting;
        systemTools.telemetry.addData("Status", "Resetting Encoders");
        systemTools.telemetry.update();

        resetEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        systemTools.telemetry.update();

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

        startEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        systemTools.telemetry.addData("Path", "Complete");
        systemTools.telemetry.addData("counts", TICKS);
        systemTools.telemetry.update();
    }

    public void RotateDEG(int degrees, double power, LinearOpMode linearOpMode) {

        final double conversion_factor = 8.46;
        if (degrees < 0 && power > 0) {
            power = power * -1;
        }
        else if (degrees > 0 && power < 0) {
            degrees = degrees * -1;
        }
        int TICKS = (int) Math.round(degrees * conversion_factor);
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        systemTools.telemetry.addData("Status", "Resetting Encoders");
        systemTools.telemetry.update();

        resetEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        systemTools.telemetry.update();

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
                (Math.abs(frontLeftMotor.getCurrentPosition()) < TICKS && Math.abs(frontRightMotor.getCurrentPosition()) < TICKS && Math.abs(backLeftMotor.getCurrentPosition()) < TICKS && Math.abs(backRightMotor.getCurrentPosition()) < TICKS)) {
        }

        stopDriveMotors();
//        frontLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        backRightMotor.setPower(0);
//        backLeftMotor.setPower(0);

        startEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        systemTools.telemetry.addData("Path", "Complete");
        systemTools.telemetry.addData("counts", TICKS);
        systemTools.telemetry.update();
    }

    public void StrafeCM(int centimeters, double power){

        final double conversion_factor = 8.46;
        if (centimeters < 0 && power > 0) {
            power = power * -1;
        }
        else if (centimeters > 0 && power < 0) {
            centimeters = centimeters * -1;
        }

        int TICKS = (int) Math.round(centimeters * conversion_factor);

        // Send telemetry message to signify robot waiting;
        systemTools.telemetry.addData("Status", "Resetting Encoders");
        systemTools.telemetry.update();

        resetEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        systemTools.telemetry.update();

        int FLtarget = frontLeftMotor.getCurrentPosition() - TICKS;
        int FRtarget = frontRightMotor.getCurrentPosition() + TICKS;
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

        startEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        systemTools.telemetry.addData("Path", "Complete");
        systemTools.telemetry.addData("counts", TICKS);
        systemTools.telemetry.update();
    }

    public void findLine(double power) {
        // Send telemetry message to signify robot waiting;
       systemTools.telemetry.addData("Status", "Resetting Encoders");
        systemTools.telemetry.update();

        resetEncoders();
//        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (
                //        (runtime.seconds() < 30) &&
                ((colors1.red != 0 && colors1.green != 0 && colors1.blue != 0) || (colors2.red != 0 && colors2.green != 0 && colors2.blue != 0))) {
            colors1 = colorSensor1.getNormalizedColors();
            colors2 = colorSensor2.getNormalizedColors();
        }

        stopDriveMotors();
//        robot.frontLeftMotor.setPower(0);
//        robot.frontRightMotor.setPower(0);
//        robot.backRightMotor.setPower(0);
//        robot.backLeftMotor.setPower(0);

        startEncoders();
//        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        systemTools.telemetry.addData("Path", "Complete");
        systemTools.telemetry.update();
    }

    public char ringFinder() {
        char path;
        boolean sensor1Detected;
        boolean sensor2Detected;
        float gain = 2;
        float redLowerVal = 1;
        float redUpperVal = 1;
        float greenLowerVal = 1;
        float greenUpperVal = 1;
        float blueLowerVal = 1;
        float blueUpperVal = 1;

        systemTools.telemetry.addData("Gain", gain);

        colorSensor1.setGain(gain);
        colorSensor2.setGain(gain);

        NormalizedRGBA colors1 = colorSensor1.getNormalizedColors();
        NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();

        sensor1Detected = colors1.red >= redLowerVal && colors1.red <= redUpperVal && colors1.green >= greenLowerVal && colors1.green <= greenUpperVal && colors1.blue >= blueLowerVal && colors1.blue <= blueUpperVal;

        sensor2Detected = colors2.red == 0 && colors2.green == 0 && colors2.blue == 0;

        if (sensor1Detected && sensor2Detected) {
            path = 'C';
        } else if (sensor1Detected && !sensor2Detected) {
            path = 'B';
        } else if (!sensor1Detected && !sensor2Detected) {
            path = 'A';
        } else {
            path = 'E';
        }

        systemTools.telemetry.addData("Path letter (E is Error): ", path);
        systemTools.telemetry.update();
        return path;
    }

    public void stopDriveMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void startEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
 }


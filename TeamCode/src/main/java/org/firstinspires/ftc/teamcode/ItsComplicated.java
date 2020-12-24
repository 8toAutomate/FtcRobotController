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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;


public class ItsComplicated
{

    // declare motors
    public DcMotorImplEx frontLeftMotor = null;
    public DcMotorImplEx frontRightMotor = null;
    public DcMotorImplEx backLeftMotor = null;
    public DcMotorImplEx backRightMotor = null;
    public DcMotorImplEx intake = null;



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
        frontLeftMotor  = hwMap.get(DcMotorImplEx.class, "front_left_drive");
        frontRightMotor = hwMap.get(DcMotorImplEx.class, "front_right_drive");
        backLeftMotor = hwMap.get(DcMotorImplEx.class, "back_left_drive");
        backRightMotor = hwMap.get(DcMotorImplEx.class, "back_right_drive");
        intake = hwMap.get(DcMotorImplEx.class, "intake");

        frontLeftMotor.setDirection(DcMotorImplEx.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorImplEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorImplEx.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorImplEx.Direction.REVERSE);
        // Set all motors to zero power
        stopAllMotors();

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        startDriveEncoders();

        // Define and initialize ALL installed servos.


    }

    public void GoDistanceCM(int centimeters, double power, LinearOpMode linearOpMode){

        final double conversion_factor = 8.46;
        int TICKS = (int) Math.round(centimeters * conversion_factor);

        // Send telemetry message to signify robot waiting;
        systemTools.telemetry.addData("Status", "Resetting Encoders");
        systemTools.telemetry.update();

        resetDriveEncoders();
//        frontLeftMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);


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

        frontLeftMotor.setMode(DcMotorImplEx.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotorImplEx.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotorImplEx.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotorImplEx.RunMode.RUN_TO_POSITION);

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

        startDriveEncoders();
//        frontLeftMotor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
//        frontRightMotor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
//        backLeftMotor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
//        backRightMotor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);

        systemTools.telemetry.addData("Path", "Complete");
        systemTools.telemetry.addData("counts", TICKS);
        systemTools.telemetry.update();
    }

    public void Rotate(int degrees, double power, LinearOpMode linearOpMode) {

        final double conversion_factor = 8.46;
        int TICKS = (int) Math.round(degrees * conversion_factor);

        // Send telemetry message to signify robot waiting;
        systemTools.telemetry.addData("Status", "Resetting Encoders");
        systemTools.telemetry.update();

        resetDriveEncoders();
//        frontLeftMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);


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

        frontLeftMotor.setMode(DcMotorImplEx.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotorImplEx.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotorImplEx.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotorImplEx.RunMode.RUN_TO_POSITION);

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
//        frontLeftMotor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
//        frontRightMotor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
//        backLeftMotor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
//        backRightMotor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);

        systemTools.telemetry.addData("Path", "Complete");
        systemTools.telemetry.addData("counts", TICKS);
        systemTools.telemetry.update();
    }

    public void Strafe(int centimeters, double power){

        final double conversion_factor = 8.46;
        int TICKS = (int) Math.round(centimeters * conversion_factor);

        // Send telemetry message to signify robot waiting;
        systemTools.telemetry.addData("Status", "Resetting Encoders");
        systemTools.telemetry.update();

        resetDriveEncoders();
//        frontLeftMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);


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

        frontLeftMotor.setMode(DcMotorImplEx.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotorImplEx.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotorImplEx.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotorImplEx.RunMode.RUN_TO_POSITION);

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
//        frontLeftMotor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
//        frontRightMotor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
//        backLeftMotor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
//        backRightMotor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);

        systemTools.telemetry.addData("Path", "Complete");
        systemTools.telemetry.addData("counts", TICKS);
        systemTools.telemetry.update();
    }

    public void stopDriveMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public void stopAllMotors() {
        stopDriveMotors();
        // add additional motors here, with a 0 power
        intake.setPower(0);
    }

    public void resetDriveEncoders() {
        frontLeftMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void startDriveEncoders() {
        frontLeftMotor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
    }
 }


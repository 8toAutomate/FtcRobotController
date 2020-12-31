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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ItsComplicated;
import org.firstinspires.ftc.teamcode.ProgrammingFrame;


@TeleOp(name="MecanumDriveIntake", group="Iterative Opmode")
//@Disabled
public class MecanumDriveIntake extends OpMode
{

    ProgrammingFrame robot   = new ProgrammingFrame();



    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    double initial;
    // Setup a variable for each drive wheel to save power level for telemetry
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;
    // Setup a variable for strafing constant
    double strafingConstant = 1.5;
    // Setup boolean variables
    boolean isIntakeOn = false;
    boolean isAPressed = false;
    enum States {
        Forwards, Backwards, Off, On
    }
    States ringPusher = States.Backwards;
    States flywheel = States.Off;
    States intakeState = States.Off;
    States intakeButtonState = States.Off;
    boolean gripperClosed = false;
    boolean gripperRaised = false;
    boolean XClick = false;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void moveGripper(boolean close) {
        robot.gripperServo.scaleRange(0, 1.0);
        if (close) {
            robot.gripperServo.setDirection(Servo.Direction.FORWARD);
            robot.gripperServo.setPosition(0.25);
        }
        else {
            robot.gripperServo.setDirection(Servo.Direction.REVERSE);
            robot.gripperServo.setPosition(0);
        }
    }

    public void moveRingPusher(States state) {
        robot.ringPusher.scaleRange(0, 1.0);
        if (state == States.Backwards) {
            robot.ringPusher.setDirection(Servo.Direction.FORWARD);
            robot.ringPusher.setPosition(0.25);
        }
        else {
            robot.ringPusher.setDirection(Servo.Direction.REVERSE);
            robot.ringPusher.setPosition(0);
        }
    }

    public void raiseGripper() {
        robot.lifting.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lifting.setTargetPosition(robot.lifting.getCurrentPosition() + 320);
        robot.lifting.setPower(1);
        while (robot.lifting.isBusy()) {}
        robot.lifting.setPower(0);
    }

    public void lowerGripper() {
        robot.lifting.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lifting.setTargetPosition(robot.lifting.getCurrentPosition() - 320);
        robot.lifting.setPower(-1);
        while (robot.lifting.isBusy()) {}
        robot.lifting.setPower(0);
    }
    @Override
    public void init() {
        robot.init(hardwareMap, this);
        robot.shooting.setPower(0);
        gamepad1.setJoystickDeadzone(.1f);
        gamepad2.setJoystickDeadzone(.1f);
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
        initial = 0.5;
    }

    @Override
    public void loop() {

        // controller variables
        // y: inverse of left stick's y value
        double y = -gamepad1.left_stick_y;
        // x underscored: left stick's x value multiplied by the strafing coefficient in order to counteract imperfect strafing
        double x = gamepad1.left_stick_x * strafingConstant;
        // rx: right stick's x value
        double rx = gamepad1.right_stick_x;

        if (Math.abs(x) <= .2) x = 0;

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

        frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
        frontRightPower   = Range.clip(frontRightPower, -1.0, 1.0);
        backLeftPower = Range.clip(backLeftPower, -1.0, 1.0);
        backRightPower   = Range.clip(backRightPower, -1.0, 1.0);

        robot.frontLeftMotor.setPower(frontLeftPower);
        robot.frontRightMotor.setPower(frontRightPower);
        robot.backLeftMotor.setPower(backLeftPower);
        robot.backRightMotor.setPower(backRightPower);

        // shooting motors turn on by pressing the right bumper
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
//*******************Flywheel**********************************************************************
        if (flywheel == States.On) {
            robot.shooting.setPower(1);
        }else{
            robot.shooting.setPower(0);
            flywheel = States.Off;
        }
//********************SHooting push Arm************************************************************
        if (gamepad1.left_bumper && ringPusher == States.Backwards && getRuntime() - initial > .5) {
            moveRingPusher(ringPusher);
            ringPusher = States.Forwards;
            double initial = getRuntime();
        }
        if (getRuntime() - initial > .5 && ringPusher == States.Forwards ) {
                moveRingPusher(ringPusher);
                ringPusher = States.Backwards;
                initial = getRuntime();
        }



        if (gamepad1.a) {
            if (intakeButtonState == States.Off && intakeState == States.Forwards) {
                intakeState = States.Off;
            } else {
                intakeState = States.Forwards;
            }
            intakeButtonState = States.Forwards;
        } else if (gamepad1.b) {
            if (intakeButtonState == States.Off && intakeState == States.Backwards) {
                intakeState = States.Off;
            } else {
                intakeState = States.Backwards;
            }
            intakeButtonState = States.Backwards;
        } else {
            intakeButtonState = States.Off;
        }

        if(intakeState == States.Forwards) {
            robot.intake.setPower(1);
        } else if (intakeState == States.Backwards) {
            robot.intake.setPower(-1);
        } else {
            robot.intake.setPower(0);
        }

        if (gamepad1.dpad_up && !gripperRaised) {
            lowerGripper();
            gripperRaised = true;
        }

        if (gamepad1.dpad_down && gripperRaised) {
            lowerGripper();
            gripperRaised = false;
        }

        if (gamepad1.dpad_left && !gripperClosed) {
            moveGripper(true);
            gripperClosed = true;
        }
        if (gamepad1.dpad_right && gripperClosed) {
            moveGripper(false);
            gripperClosed = false;
        }

       // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Strafing constant", "Strafing Constant = " + strafingConstant);
        telemetry.addData("Motors", "front_left (%.2f), front_right (%.2f), back_left (%.2f), back_right (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }


    public void stop() {
        robot.stopAllMotors();
    }

}

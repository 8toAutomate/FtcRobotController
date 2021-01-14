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

import org.firstinspires.ftc.teamcode.ProgrammingFrame;


@TeleOp(name="MecanumDriveIntake2", group="Iterative Opmode")
//@Disabled
public class MecanumDriveIntake2 extends OpMode
{

    ProgrammingFrame robot   = new ProgrammingFrame();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;
    double initialSH;  //initial time for shotting button timer
    double initialST;  //initial time for storage button timer
    double initialFL;  //initial time for flywheel button timer
    //double initialIN;  //initial time for intake button timer
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
    boolean xClick, yClick = false;
    boolean shooting = false;   // Flag  is true when shooting process is in progress
    boolean shootButton = false;   // shoot button status flag -  true  = button was pressed
    boolean shootingReset = true;  // shooting arm return flag - false = shooting arm reset in process
    boolean storageUp = false;
    boolean movingStorage = false;
    boolean storagePressed = false;
    boolean flyWheel, flyMotor, flyWheel2 = false;

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
/*
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
*/
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
    //*********************************************************************************************************************************
    public void strafeDistanceCM2(int centimeters, double power, boolean handoff){

        double conversion_factor = 31.3;

        boolean left = centimeters < 0;
        int TICKS = (int) Math.abs(Math.round(centimeters * conversion_factor));
        int FLtarget = 0;
        int FRtarget = 0;
        int BLtarget = 0;
        int BRtarget = 0;

        power = Math.abs(power);

        robot.resetDriveEncoders();

         if (left) {
            FLtarget = robot.frontLeftMotor.getCurrentPosition() - TICKS;
            FRtarget = robot.frontRightMotor.getCurrentPosition() + TICKS;
            BLtarget = robot.backLeftMotor.getCurrentPosition() + TICKS;
            BRtarget = robot.backRightMotor.getCurrentPosition() - TICKS;
        } else {
            FLtarget = robot.frontLeftMotor.getCurrentPosition() + TICKS;
            FRtarget = robot.frontRightMotor.getCurrentPosition() - TICKS;
            BLtarget = robot.backLeftMotor.getCurrentPosition() - TICKS;
            BRtarget = robot.backRightMotor.getCurrentPosition() + TICKS;
        }
        frontLeftMotor.setTargetPosition(FLtarget);
        frontRightMotor.setTargetPosition(FRtarget);
        backLeftMotor.setTargetPosition(BLtarget);
        backRightMotor.setTargetPosition(BRtarget);

        /*frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


         */

        // start motion

        if (left) {
            robot.frontLeftMotor.setPower(-power);
            robot.frontRightMotor.setPower(power);
            robot.backRightMotor.setPower(-power);
            robot.backLeftMotor.setPower(power);
        } else {
            robot.frontLeftMotor.setPower(power);
            robot.frontRightMotor.setPower(-power);
            robot.backRightMotor.setPower(power);
            robot.backLeftMotor.setPower(-power);
        }


        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while  (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy()) {
        }

        if (!handoff) robot.stopDriveMotors();

        robot.startDriveEncoders();

          }
    public void stopDriveMotors() {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
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
    //*****************************************************************************************
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
        initialSH = 0.6;
    }

    @Override
    public void loop() {
//********************************* Robot movement ********************************************************
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
            if (gamepad1.x) { // checks if the bumper is pressed
                xClick = true; // set X-button flag - X-button was pressed
                flyWheel = true; // Flywheel process has started
                //flyMotor = true;
                initialFL = getRuntime(); // gets current time
               // telemetry.addData("Status", "xClick " + gamepad1.x);
            }
        }

        if (xClick && flyWheel) { // checks if the sflywheel is moving and if the storage pressed flag is raised
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
        // This is similar to the FLywheel program but runs motor at lower power for power-shot

        // FLAGS:
        // flyWheel2 - holds the state of the Flywheel operation
        // flyMotor- holds if the flywheel motor is running or off
        // xClick - checks if flywheel button is pressed

        if (!flyWheel2) { // checks if the flywheel is not already moving
            if (gamepad1.y) { // checks if the bumper is pressed
                yClick = true; // set X-button flag - X-button was pressed
                flyWheel2 = true; // Flywheel process has started
                //flyMotor = true;
                initialFL = getRuntime(); // gets current time
             //   telemetry.addData("Status", "yClick " + gamepad1.y);
            }
        }

        if (yClick && flyWheel2) { // checks if the storage is moving and if the storage pressed flag is raised
            if (!flyMotor) {robot.shooting.setPower(0.70);} // if the flywheel is off , turn it on
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
                if (gamepad1.right_bumper) { // shooting button and shooting process status flags to true.
                    initialSH = getRuntime();
                    shooting = true;
                    shootButton = true;
                } // end if gamepad.left_bumper
            } // end if !shoot_button
        } // end if !shooting

        if (shootButton){                              // check button status flag. If true
            if (shooting) {                             //  then check if shooting process is in progress
                robot.ringPusher.setPosition(1);        // run shooting servo to max position.
                    if (getRuntime() - initialSH > .5) {  // check if enough time has passed.
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

        if (gamepad1.a) {
            if (intakeButtonState == States.Off) {
                if (intakeState == States.Forwards) {
                    intakeState = States.Off;
                    intakeButtonState = States.Forwards;
                }
            } if (intakeButtonState != States.Forwards) {
                intakeState = States.Forwards;
            }
            intakeButtonState = States.Forwards;
        } else if (gamepad1.b) {
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
        }

        if(intakeState == States.Forwards) {
            robot.intake.setPower(1);
        } else if (intakeState == States.Backwards) {
            robot.intake.setPower(-1);
        } else {
            robot.intake.setPower(0);
        }
        //********************************* Gripper and Gripper arm ************************************
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

        //********************************* Storage Servo **********************************************

        // this code raises and lowers the storage servo using the left bumper button.
        // This is similar to the ring pusher program, except it is implemented for toggle functionality.

        // FLAGS:
        // storageUp - holds the state of the storage
        // movingStorage - holds if the storage servo is in motion
        // storagePressed - checks if storage button is pressed

        if (!movingStorage) { // checks if the storage is not already moving
            if (gamepad1.left_bumper) { // checks if the bumper is pressed
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
//************************************************************************************************************
        if (gamepad1.right_trigger>0.9) {
            robot.strafeDistanceCM3(20,0.2,false);
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

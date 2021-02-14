package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Autonomous(name="AutonomousSecondTourney", group="Motion")
public class BlueAutoSecondTourney extends LinearOpMode {
    // This program starts on the left blue line, shoots at the high goal, drops off a wobble goal
    // in it's target, than drives to center shooting spot to park at the end.
    char ringAt;
    ProgrammingFrame robot   = new ProgrammingFrame();
    private ElapsedTime runtime = new ElapsedTime();
    ProgrammingFrame.wobble wobble = new ProgrammingFrame.wobble();


    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        waitForStart();
        // Have method(s) that shoot 3 rings here, likely in the high goal
        // Driving to the starter stack

     /*   robot.gripperOpen();
        robot.gripperOpen();
        robot.wait(1000L,this);
        robot.raiseGripper(800);

        robot.gripperClose();
        robot.wait(1000L,this);
        robot.raiseGripper(300);

        robot.GoDistanceCM2(70, .5, false,this);
        robot.wait(500,this);  // Wait for robot to stop before reading rings

            // Detect the rings here and return A, B, C, or E for Error
        ringAt = robot.ringFinderDistance(this);
        if (ringAt == 'E') {  // Top saw a ring but bottom didn't somehow, try one more time
            char tryAgain = robot.ringFinderDistance(this);  // If this fails it will take C path
            if (tryAgain == 'E') {
                ringAt = 'C';
            }
            else {
                ringAt = tryAgain;
            }
        }
       // ringAt = robot.ringFinderDistance();
        telemetry.addData("Target zone", ringAt);
        telemetry.update();

        // Gets us to the target zone
        robot.GoDistanceCM2(60, .5, false, this);

        robot.storage(true, this);
        robot.flywheel(true, 0.8);
        robot.wait(3000,this);

        for (int i = 0; i<4; i++) {                             // added extra shot in case last ring is stuck.
            robot.pushRing(0.5, this);
            robot.wait(1200, this);
        }
        robot.flywheel(false, 0.0);

        if (ringAt == 'A') {
            robot.GoDistanceCM2(26, .7, false, this);
            robot.strafeDistanceCM2(-37, .7,false, this);
            // drop wobble stick here
            robot.lowerGripper(250);
            robot.gripperOpen();
            robot.wait(1000L,this);
            robot.lowerGripper(900);
            robot.GoDistanceCM2(-20, -.7,false, this);
            robot.strafeDistanceCM2(55, .7,false, this);  // go sidways and forward  to launch line without
            robot.GoDistanceCM2(35, .7, false, this);   // disturbing wobble (strafe needed for Target zone A only)
        }
        else if (ringAt == 'B') {
            robot.GoDistanceCM2(95, .7, false,this);   // added extra 20 cm to get ring out of the way.  Wobble was landing on top of ring 1-27-2021
            robot.strafeDistanceCM2(35, .7,false, this);
            robot.GoDistanceCM2(-20, -.7, false,this);  //  back up and drop wobble away from ring
            // drop wobble goal here
            robot.lowerGripper(250);
            robot.gripperOpen();
            robot.wait(1000L,this);
            robot.lowerGripper(900);
            robot.GoDistanceCM2(-30, -.7, false,this);
        }
        else {   // ring at C
            robot.GoDistanceCM2(147, .7, false,this);
            robot.strafeDistanceCM2(-30, .7,false, this);
            // drop wobble goal here
            robot.lowerGripper(250);
            robot.gripperOpen();
            robot.wait(1000L,this);
            robot.lowerGripper(950);
            robot.GoDistanceCM2(-80, -.7, false,this);
        }
        robot.gripperClose();
        robot.storage(false, this);
        robot.wait(1500L,this);  //allow time for servo to finish closing grip before terminating
        // /*  Debug: comment out rest of method  MAx M. 12-24-2020
        // Add function that drops a wobble goal
        // Move to the launch line
      /*  if (ringAt == 'A') {
            robot.StrafeCM2(-22, .7, this);
            robot.GoDistanceCM2(27, .7, this);
        }
        else if (ringAt == 'B') {
            robot.GoDistanceCM2(-27, .7, this);
        }
        else {
            robot.GoDistanceCM2(-86, .7, this);
            robot.StrafeCM2(-59, .7, this);
        }


        // Comment out the below if we don't have time!!!
        // Use our sensor to make sure we are on the line
        // Backup to make sure we are behind the line
        //robot.GoDistanceCM(-15, .8, this);
        // Line up to the line
        //robot.findLine(.5);
        // Go forward a tiny bit that way we are more centered on the line
        //robot.GoDistanceCM(5, .8, this);
        //end of Debug: comment out rest of method  FEM 12-24-2020

       // telemetry.addLine();
        //telemetry.addData("Final", "Starting at %7d :%7d :%7d :%7d",
        //        robot.frontLeftMotor.getCurrentPosition(),
        //        robot.frontRightMotor.getCurrentPosition(), robot.backLeftMotor.getCurrentPosition(), robot.backRightMotor.getCurrentPosition());

       // ringAt = robot.ringFinderDistance();
        telemetry.addData("Target zone", ringAt);
        telemetry.update();
       // while (opModeIsActive()) {}  //  Empty while loop - program waits until user terminates op-mode
*/
      robot.RotateDEG(147, 0.5, this);
      robot.goDistanceAcceleration(105, 0.7, false, 20, 70, this);
      robot.wobbleFind(30, 0.2, 40, this);
      boolean success = ProgrammingFrame.wobble.success;
      int degrees = ProgrammingFrame.wobble.rotateBack;
        robot.gripperOpen();
      robot.RotateDEG(8, .2, this);
      // robot.wait(1000L,this);
      robot.raiseGripper(750);
      robot.GoDistanceCM2(5, .2, false, this);
      robot.gripperClose();
      robot.wait(1000, this);
      robot.gripperOpen();
      robot.wait(1000, this);
      robot.GoDistanceCM2(-20, 30, false, this);
      robot.gripperClose();
      robot.lowerGripper(800);

    }

}

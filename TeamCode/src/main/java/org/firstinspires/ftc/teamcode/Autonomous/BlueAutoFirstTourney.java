package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Autonomous(name="AutonomousFirstTourney", group="Motion")
public class BlueAutoFirstTourney extends LinearOpMode {
    // This program starts on the left blue line, shoots at the high goal, drops off a wobble goal
    // in it's target, than drives to center shooting spot to park at the end.
    char ringAt;
    ProgrammingFrame robot   = new ProgrammingFrame();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        waitForStart();
        // Have method(s) that shoot 3 rings here, likely in the high goal
        // Driving to the starter stack

        robot.gripperOpen();
        robot.wait(1000L,this);
        robot.raiseGripper(850);

        robot.gripperClose();
        robot.wait(1000L,this);
        robot.raiseGripper(850);

        robot.GoDistanceCM2(69, .5, false,this);
     //   while (opModeIsActive()) {}// debug stop prgram here



        // Detect the rings here and return A, B, C, or E for Error
        ringAt = robot.ringFinderDistance();
        telemetry.addData("Target zone", ringAt);
        telemetry.update();
        if (ringAt == 'E') {  // Top saw a ring but bottom didn't somehow, try one more time
            char tryAgain = robot.ringFinder();  // If this fails it will take C path
            if (tryAgain == 'E') {
                ringAt = 'C';
            }
            else {
                ringAt = tryAgain;
            }
        }

        // Gets us to the target zone
        robot.GoDistanceCM2(65, .5, false, this);

        robot.storage(true, this);
        robot.flywheel(true, 0.8);
        robot.wait(3000,this);

        for (int i = 0; i<3; i++) {
            robot.pushRing(0.5, this);
            robot.wait(1500, this);
        }

        if (ringAt == 'A') {
            robot.GoDistanceCM2(80, .7, false, this);
        }
        else if (ringAt == 'B') {
            robot.GoDistanceCM2(140, .7, false,this);
            robot.strafeDistanceCM2(40, .7,false, this);
        }
        else {
            robot.GoDistanceCM2(205, .7, false,this);
        }

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
       */


        // Comment out the below if we don't have time!!!
        // Use our sensor to make sure we are on the line
        // Backup to make sure we are behind the line
        //robot.GoDistanceCM(-15, .8, this);
        // Line up to the line
        //robot.findLine(.5);
        // Go forward a tiny bit that way we are more centered on the line
        //robot.GoDistanceCM(5, .8, this);
        //end of Debug: comment out rest of method  FEM 12-24-2020
        //*/
        telemetry.addLine();
        //telemetry.addData("Final", "Starting at %7d :%7d :%7d :%7d",
        //        robot.frontLeftMotor.getCurrentPosition(),
        //        robot.frontRightMotor.getCurrentPosition(), robot.backLeftMotor.getCurrentPosition(), robot.backRightMotor.getCurrentPosition());

        telemetry.update();
        while (opModeIsActive()) {}  //  Empty while loop - program waits until user terminates op-mode

    }


}

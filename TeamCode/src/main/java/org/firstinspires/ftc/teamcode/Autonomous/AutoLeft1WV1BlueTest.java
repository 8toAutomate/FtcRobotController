package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Autonomous(name="Left1WobbleTargetBlueTest", group="Motion")
public class AutoLeft1WV1BlueTest extends LinearOpMode {
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
        robot.GoDistanceCM2(69, .7, false,this);
     //   while (opModeIsActive()) {}// debug stop prgram here



        // Detect the rings here and return A, B, C, or E for Error
        ringAt = robot.ringFinder();
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
        robot.StrafeCM2(-22, .7, this);
        if (ringAt == 'A') {
            robot.GoDistanceCM2(80, .7, false, this);
        }
        else if (ringAt == 'B') {
            robot.GoDistanceCM2(140, .7, false,this);
            robot.StrafeCM2(40, .7, this);
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
        robot.moveGripper(true);
        robot.lowerGripper();

        //end of Debug: comment out rest of method  FEM 12-24-2020
        //*/
        telemetry.addLine();
        telemetry.addData("Final", "Starting at %7d :%7d :%7d :%7d",
                robot.frontLeftMotor.getCurrentPosition(),
                robot.frontRightMotor.getCurrentPosition(), robot.backLeftMotor.getCurrentPosition(), robot.backRightMotor.getCurrentPosition());
        telemetry.update();
        //while (opModeIsActive()) {}  //  Empty while loop - program waits until user terminates op-mode

    }


}

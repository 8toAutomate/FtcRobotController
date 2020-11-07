package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Autonomous(name="Left2WobbleTargets", group="Motion")
public class AutoLeft2WV1 extends LinearOpMode {
    // This program starts on the right line, drops off a wobble goal in it's target,
    // Goes to shoot at the center shooting spot, grabs second wobble goal and drops in it's target,
    // than drives to center shooting spot to park at the end.
    char ringAt;
    ProgrammingFrame robot   = new ProgrammingFrame();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        waitForStart();
        // Have method(s) that shoot 3 rings here, find out if the team want the 3 rings shot in
        // the Tower Goal, Power Shot Targets, or some mixture of both
        // Driving to the starter stack
        robot.GoDistanceCM(59, .8, this);
        robot.StrafeCM(35, .8);
        // Detect the rings here and return A, B, C, or E for Error
        ringAt = robot.ringFinder();
        if (ringAt == 'E') {  // Top saw a ring but bottom didn't somehow, taking C path is safest
            ringAt = 'C';
        }
        // Gets us to the target zone
        robot.StrafeCM(59, .8);
        if (ringAt == 'A') {
            robot.GoDistanceCM(59, .8, this);
        }
        else if (ringAt == 'B') {
            robot.GoDistanceCM(118, .8, this);
            robot.StrafeCM(-59, .8);
        }
        else {
            robot.GoDistanceCM(177, .8, this);
        }
        // Add function that drops a wobble goal
        // Move to the launch line
        if (ringAt == 'A') {
            robot.StrafeCM(-59, .8);
            robot.GoDistanceCM(27, .8, this);
        }
        else if (ringAt == 'B') {
            robot.GoDistanceCM(-27, .8, this);
        }
        else {
            robot.GoDistanceCM(-86, .8, this);
            robot.StrafeCM(-59, .8);
        }
        // Go to pick up other wobble goal
        robot.StrafeCM(-59, .8);
        robot.GoDistanceCM(-144, .8, this);
        robot.RotateDEG(90, .8, this);
        // Add function that picks up that wobble goal
        // Drive back to the target zones to drop off the second wobble goal
        robot.RotateDEG(-90, .8, this);
        robot.GoDistanceCM(144, .8, this);
        if (ringAt == 'A') {
            robot.StrafeCM(118, .8);
        }
        else if (ringAt == 'B') {
            robot.GoDistanceCM(59, .8, this);
            robot.StrafeCM(59, .8);
        }
        else {
            robot.GoDistanceCM(118, .8, this);
            robot.StrafeCM(118, .8);
        }
        // use the method(s) to drop the wobble goal
        // Drive to the launch line (in particular the place we shot from)
        if (ringAt == 'A') {
            robot.StrafeCM(-59, .8);  // Could be less if time is real close
            robot.GoDistanceCM(27, .8, this);
        }
        else if (ringAt == 'B') {
            robot.GoDistanceCM(-27, .8, this);
        }
        else {
            robot.GoDistanceCM(-86, .8, this);
            robot.StrafeCM(-59, .8);  // Can be removed if not enough time
        }
        // Comment out the below if we don't have time!!!
        // Use our sensor to make sure we are on the line
        // Backup to make sure we are behind the line
        //robot.GoDistanceCM(-15, .8, this);
        // Line up to the line
        //robot.findLine(.5);
        // Go forward a tiny bit that way we are more centered on the line
        //robot.GoDistanceCM(5, .8, this);
    }
}

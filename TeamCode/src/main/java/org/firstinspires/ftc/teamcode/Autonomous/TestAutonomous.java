package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Autonomous(name="TestAutonomous", group="Motion")
public class TestAutonomous extends LinearOpMode {
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

        robot.goDistanceAcceleration(100,0.8,false, 10.0, 90.0, this);

        telemetry.addData("Target zone", ringAt);
        telemetry.update();
       // while (opModeIsActive()) {}  //  Empty while loop - program waits until user terminates op-mode

    }

}

package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Autonomous(name="ScrimmageAutonomousV2", group="Motion")
public class ScrimmageAutonomousV2 extends LinearOpMode {
    // This program starts on the left red line, shoots at the high goal, drops off a wobble goal in
    // it's target, than drives to center shooting spot to park at the end.
    char ringAt;
    ProgrammingFrame robot   = new ProgrammingFrame();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, this);
        waitForStart();

        robot.flywheel(true, 0.8);

        robot.storage(true);

        for (int i = 0; i < 3; i++) {
            robot.pushRing(0.5, this);
        }

        robot.flywheel(false, 0.8);

        robot.GoDistanceCM(160, .7, this);
    }
}

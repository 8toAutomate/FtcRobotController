package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Autonomous(name="RotateTest", group="Motion")
public class RotateTest extends LinearOpMode{

    ProgrammingFrame robot   = new ProgrammingFrame();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        waitForStart();
        robot.RotateDEG(360, .5, this);
        //robot.wait(300, this);
        //robot.RotateDEG(-360, .8, this);
    }
}

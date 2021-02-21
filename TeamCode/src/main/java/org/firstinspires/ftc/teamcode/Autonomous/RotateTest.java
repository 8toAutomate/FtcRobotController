package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
        double initial = getRuntime();
        robot.RotateDEG(7, 0.2, this);
        telemetry.addData("Final time", getRuntime() - initial);
        //robot.wait(300, this);
        //robot.RotateDEG(-360, .8, this);
        telemetry.update();
        while (opModeIsActive()) {}
    }
}

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Disabled
@Autonomous(name="GyroTurn", group="Motion")

public class GyroTurn extends LinearOpMode {

    ProgrammingFrame robot   = new ProgrammingFrame();
    static final double conversion_factor = 21.1;
    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode() {
        public void RotateDEG(int degrees, double power, double angle) {

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            // conversion for degrees to ticks
            final double conversion_factor = 12.73;

            // if degrees are negative, set the power negative
            if (degrees < 0 && power > 0) {
                power = power * -1;
            }

            int TICKS = (int) Math.round(degrees * conversion_factor);

            robot.resetDriveEncoders();

            // Send telemetry message to indicate successful Encoder reset
            // systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
            //         frontLeftMotor.getCurrentPosition(),
            //         frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
            // systemTools.telemetry.update();

            // set target position for all the motor encoders
            int FLtarget = robot.frontLeftMotor.getCurrentPosition() + TICKS;
            int FRtarget = robot.frontRightMotor.getCurrentPosition() - TICKS;
            int BLtarget = robot.backLeftMotor.getCurrentPosition() + TICKS;
            int BRtarget = robot.backRightMotor.getCurrentPosition() - TICKS;

            robot.frontLeftMotor.setTargetPosition(FLtarget);
            robot.frontRightMotor.setTargetPosition(FRtarget);
            robot.backLeftMotor.setTargetPosition(BLtarget);
            robot.backRightMotor.setTargetPosition(BRtarget);

            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            robot.frontLeftMotor.setPower(power);
            robot.frontRightMotor.setPower(-power);
            robot.backRightMotor.setPower(-power);
            robot.backLeftMotor.setPower(power);

            // keep looping while we are still active, and there is time left, and all motors are running.
            while (opModeIsActive() &&
                    (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy())
            ) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                if (angles.firstAngle == angle) {
                    break;
                }
            }

            robot.stopDriveMotors();
            robot.startDriveEncoders();

        }   //end RotateDeg
    }

}

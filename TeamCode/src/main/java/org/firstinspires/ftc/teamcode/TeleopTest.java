package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.localization.Kinematics;
import org.firstinspires.ftc.teamcode.localization.MecanumKinematics;
import org.firstinspires.ftc.teamcode.localization.Pose2d;

import java.util.List;

@TeleOp(name="teleoptest")
public class TeleopTest extends LinearOpMode {
    DcMotorEx frontLeft, rearLeft, rearRight, frontRight;

    @Override
    public void runOpMode() {
        frontLeft = (DcMotorEx) hardwareMap.get(DcMotor.class, "left_front");
        rearLeft = (DcMotorEx) hardwareMap.get(DcMotor.class, "left_back");
        frontRight = (DcMotorEx) hardwareMap.get(DcMotor.class, "right_front");
        rearRight = (DcMotorEx)  hardwareMap.get(DcMotor.class, "right_back");

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            double x = Math.pow(gamepad1.left_stick_x, 3);
            double y = Math.pow(gamepad1.left_stick_y, 3);
            double heading = -1 * Math.pow(gamepad1.right_stick_x, 3);
            Pose2d gamepadVelocity = new Pose2d(
                    x,
                    y,
                    heading
            );
            telemetry.addData("x: ",x);
            telemetry.addData("y: ", y);
            telemetry.addData("heading: ", heading);

            List<Double> wheelVelocities = MecanumKinematics.robotToWheelVelocites(gamepadVelocity, 13);
            telemetry.addData("Velo 1: ", wheelVelocities.get(0));
            telemetry.addData("Velo 2: ", wheelVelocities.get(1));
            telemetry.update();
            frontLeft.setPower(wheelVelocities.get(0));
            rearLeft.setPower(wheelVelocities.get(1));
            rearRight.setPower(wheelVelocities.get(2));
            frontRight.setPower(wheelVelocities.get(3));
        }
    }
}

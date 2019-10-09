package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
public class TeleopTest extends OpMode {
	Drivetrain drivetrain;

	@Override
	public void init() {
		drivetrain = Drivetrain.getInstance(hardwareMap);
		drivetrain.setUseEncoders(true);
	}

	@Override
	public void loop() {

		double x = Math.pow(gamepad1.left_stick_x, 3);
		double y = Math.pow(gamepad1.left_stick_y, 3);
		double heading = -1 * Math.pow(gamepad1.right_stick_x, 3);
		Pose2d gamepadVelocity = new Pose2d(x, y, heading);
		telemetry.addData("x: ",x);
		telemetry.addData("y: ", y);
		telemetry.addData("heading: ", heading);

		List<Double> wheelVelocities = MecanumKinematics.robotToWheelVelocites(gamepadVelocity, 13);
		telemetry.addData("Velo 1: ", wheelVelocities.get(0));
		telemetry.addData("Velo 2: ", wheelVelocities.get(1));
		telemetry.update();

		drivetrain.setPower(wheelVelocities.get(0), wheelVelocities.get(3), wheelVelocities.get(1), wheelVelocities.get(2));
	}
}

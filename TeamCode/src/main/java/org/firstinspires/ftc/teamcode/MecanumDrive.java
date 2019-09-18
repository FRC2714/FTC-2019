package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Mecanum Drive", group = "Prototype")

public class MecanumDrive extends OpMode {

	final static double DEADZONE = 0.01;
	Drivetrain drivetrain;
	static Telemetry tele;

	/**
	 * First function loaded at beginning
	 */
	@Override
	public void init() {
		drivetrain = Drivetrain.getInstance(hardwareMap);
		drivetrain.setUseEncoders(true);
		tele = telemetry;
	}

	/**
	 * Continuously loop during play
	 */
	@Override
	public void loop() {
		double x = gamepad1.left_stick_x,
				y = gamepad1.left_stick_y,
				turn = gamepad1.right_stick_x;
		if (x * x + y * y > DEADZONE || Math.abs(turn) > DEADZONE) {
			drivetrain.setDirectionVector(x, y, turn);
		} else {
			drivetrain.setDirectionVector(0, 0);
		}
		telemetry.addLine("" + x + ", " + y + ", " + turn);

	}
}

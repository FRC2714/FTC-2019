package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Timer;

@TeleOp(name = "Mecanum Drive", group = "Prototype")

public class MecanumDrive extends OpMode {

	final static double DEADZONE = 0.01;
	Drivetrain drivetrain;
	static Telemetry tele;
	double currTime;
	double x;
	double y;

	Timer timer = new Timer();

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
		double deltaTime = (System.nanoTime() / 1e9) - currTime;
		currTime = System.nanoTime() / 1e9;
		double x = gamepad1.left_stick_x,
				y = gamepad1.left_stick_y,
				turn = gamepad1.right_stick_x;
		if (x * x + y * y > DEADZONE || Math.abs(turn) > DEADZONE) {
			drivetrain.setDirectionVector(x, y, turn);
		} else {
			drivetrain.setDirectionVector(0, 0);
		}
		drivetrain.updatePosition();
		telemetry.addLine("" + x + ", " + y + ", " + turn);
		telemetry.addLine("Encoder Position" + drivetrain.getLeftEncoder());
		telemetry.addLine("L Encoder Velocity" + drivetrain.getLeftVelocity());
		telemetry.addLine("Heading Angle" + drivetrain.odometer.getHeading());
	}
}

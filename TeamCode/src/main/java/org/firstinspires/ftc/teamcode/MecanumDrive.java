package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Mecanum Drive", group = "Prototype")
public class MecanumDrive extends OpMode {

	final static double DEADZONE = 0.01;
	Drivetrain drivetrain;

	@Override
	public void init() {
		drivetrain = Drivetrain.getInstance(hardwareMap);
	}

	@Override
	public void loop() {
		double x = gamepad1.left_stick_x,
				y = gamepad1.left_stick_y,
				turn = gamepad1.right_stick_x;
		if (x * x + y * y > DEADZONE || Math.abs(turn) > DEADZONE) {
			drivetrain.setDirectionVector(x, y, turn);
		}
	}
}

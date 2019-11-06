package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Timer;

@TeleOp(name = "Mecanum Drive", group = "Prototype")

public class MecanumDrive extends OpMode {

	final static double DEADZONE = 0.01;
	Drivetrain drivetrain;
	Arm arm;
	static Telemetry tele;
	double currTime;

	Timer timer = new Timer();

	boolean resetArmAtStartup = true;

	/**
	 * First function loaded at beginning
	 */
	@Override
	public void init() {
		drivetrain = Drivetrain.getInstance(hardwareMap);
		arm = Arm.getInstance(hardwareMap);
		drivetrain.resetEncoders();
		drivetrain.setEncoderState(true);
		tele = telemetry;
	}

	/**
	 * Continuously loop during play
	 */
	@Override
	public void loop() {
		while (resetArmAtStartup && arm.getArmPotentiometerPosition() < 0.75){
			arm.setArmMotor(0.4);
		}
		resetArmAtStartup = false;

		double deltaTime = (System.nanoTime() / 1e9) - currTime;
		currTime = System.nanoTime() / 1e9;
		double forwardInput = gamepad1.left_stick_x,
				strafeInput = gamepad1.left_stick_y,
				turnInput = gamepad1.right_stick_x;
		if (forwardInput * forwardInput + strafeInput * strafeInput > DEADZONE || Math.abs(turnInput) > DEADZONE) {
			drivetrain.setDirectionVector(forwardInput, strafeInput, turnInput);
		} else {
			drivetrain.setDirectionVector(0, 0);
		}
		drivetrain.updatePosition();

		boolean a = gamepad2.a,
				b = gamepad2.b,
				x = gamepad2.x,
				y = gamepad2.y,
				lb = gamepad2.left_bumper,
				rb = gamepad2.right_bumper;

		double manualArmInput = -gamepad2.left_stick_y;

		/*
		if (a) { arm.goToPosition(carry); }
		if (b) { arm.goToPosition(pickUp); }
		if (x) { arm.goToPosition(tuck); }
		if (y) { arm.goToPosition(stack); }
		 */

		if (a)
			arm.setServo(1);
		if (b)
			arm.setServo(0.68);

		if (rb)
			arm.setIntakeMotor(-1.0);
		else if(lb)
			arm.setIntakeMotor(1.0);
		else
			arm.setIntakeMotor(0.0);


		if(!(arm.getArmPotentiometerPosition() > 0.67)) {
			if (manualArmInput > 0.07)
				arm.setArmMotor(-1);
			else if (manualArmInput < -0.07)
				arm.setArmMotor(1);
			else
				arm.setArmMotor(0.0);
		} else {
			if (manualArmInput > 0.07)
				arm.setArmMotor(-1);
			else
				arm.setArmMotor(0);
		}

		telemetry.addData("Arm Motor Encoder Position", arm.getArmMotorEncoderPosition());
		telemetry.addData("Arm Potentiometer Voltage", arm.getArmPotentiometerPosition());
		telemetry.addData("Y Stick", manualArmInput);
	}
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.Timer;

import static org.firstinspires.ftc.teamcode.subsystems.Arm.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Prototype")
public class DriverControl extends OpMode {

	private final static double DEADZONE = 0.01;
	private Drivetrain drivetrain;
	private Arm arm;
	static Telemetry tele;
	double currTime;

	Timer timer = new Timer();

	private boolean resetArmAtStartup = true;

	/**
	 * First function loaded at beginning
	 */
	@Override
	public void init() {
		drivetrain = Drivetrain.getInstance(hardwareMap);
		arm = getInstance(hardwareMap);
		drivetrain.resetEncoders();
		drivetrain.setEncoderState(true);
		tele = telemetry;
	}

	/**
	 * Continuously loop during play
	 */
	@Override
	public void loop() {
		if(resetArmAtStartup)
			setupArm();

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
				rb = gamepad2.right_bumper,
                dpad_down = gamepad2.dpad_down;


		double manualArmInput = -gamepad2.left_stick_y;


		if (dpad_down) {
			arm.setServo(ServoPosition.RELAXED);
			arm.goToPosition(ArmPosition.STONE_PICKUP, 1);
			arm.setServo(ServoPosition.PRESSURE_STONE);
			arm.setIntakeMotor(-1, 0.5);
			arm.goToPosition(ArmPosition.SAFE_HOLD, 1);
		}

//		if (y) {
//			arm.setServo(ServoPosition.PRESSURE_STONE);
//			arm.goToPosition(ArmPosition.STONE_PICKUP, 1);
//			arm.setServo(ServoPosition.RELAXED);
//			arm.setIntakeMotor(1, 0.3);
//			arm.goToPosition(ArmPosition.SAFE_HOLD, 1);
//		}



		if (a)
			arm.goToPosition(ArmPosition.STONE_PICKUP, 1);
		if (b)
			arm.goToPosition(ArmPosition.SAFE_HOLD, 1);
		if(x)
		    arm.goToPosition(ArmPosition.HIGH_HOLD, 1);

//        if(a)
//            drivetrain.setLinearMotion(-0.7,-0.7,-800,0,true);
//        if(b)
//            drivetrain.setRunToEncoderMotion(0.5,-800,0,true);

//		if(a)
//			arm.setIntakeServo();

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
		} else if((arm.getArmPotentiometerPosition() < 0.02)){
			if (manualArmInput < -0.07)
				arm.setArmMotor(1);
			else
				arm.setArmMotor(0);
		} else {
			if (manualArmInput > 0.07)
				arm.setArmMotor(-1);
			else
				arm.setArmMotor(0);
		}

		arm.update();

		telemetry.addData("Arm Motor Encoder Position", arm.getArmMotorEncoderPosition());
		telemetry.addData("Arm Potentiometer Voltage", arm.getArmPotentiometerPosition());
        telemetry.addData("Average Position", (drivetrain.getRightEncoder() + drivetrain.getLeftEncoder())/2);
        telemetry.addData("Heading Angle", drivetrain.getHeadingAngle());
    }

	public void setupArm(){
		while (resetArmAtStartup && arm.getArmPotentiometerPosition() < 0.75){
			arm.setArmMotor(0.4);
		}
		if (resetArmAtStartup) {
			arm.setArmMotor(0);
			arm.resetEncoders();
		}
		resetArmAtStartup = false;
	}

}

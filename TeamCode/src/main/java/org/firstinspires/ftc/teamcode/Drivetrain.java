package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Timer;

public class Drivetrain {

	final String[] wheelNames = { "left_front", "left_back", "right_front", "right_back" };

	private static Drivetrain mecanumDrive;

	private HardwareMap hardwareMap;

	private DcMotorEx[] wheels;

	public BasicOdometer odometer;

	/**
	 * Ensures that only one instance of `Drivetrain` exists and that it
	 * is used globally
	 * @param hm Hardware map provided by `OpMode` class
	 * @return universal mecanum drivetrain
	 */
	public static Drivetrain getInstance(HardwareMap hm) {
		if (mecanumDrive == null) mecanumDrive = new Drivetrain(hm);
			return mecanumDrive;
	}

	private Drivetrain(HardwareMap hm) {
		hardwareMap = hm;
		odometer = new BasicOdometer(hardwareMap);
		wheels = new DcMotorEx[wheelNames.length];

		for (int i = 0; i < wheelNames.length; i++) {
			wheels[i] = hardwareMap.get(DcMotorEx.class, wheelNames[i]);
			wheels[i].setPower(0);
		}

		wheels[2].setDirection(DcMotorSimple.Direction.REVERSE);
		wheels[3].setDirection(DcMotorSimple.Direction.REVERSE);
	}

	/**
	 * Set the encoder mode of each motor
	 * @param useEncoders whether or not to allow encoder use
	 */
	public void setUseEncoders(boolean useEncoders) {
		for (int i = 0; i < wheelNames.length; i++) {
			wheels[i].setMode(useEncoders ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		}
	}

	/**
	 * Sets the powers of each wheel
	 * @param fl front-left motor
	 * @param fr front-right motor
	 * @param bl back-left motor
	 * @param br back-right motor
	 */
	public void setPower(double fl, double fr, double bl, double br) {
		wheels[0].setPower(clamp(fl, -1, 1));
		wheels[1].setPower(clamp(fr, -1, 1));
		wheels[2].setPower(clamp(bl, -1, 1));
		wheels[3].setPower(clamp(br, -1, 1));
	}

	/**
	 * Sets the robot on a course headed in the direction of vector
	 * (x, y) relative to its current orientation with the option to
	 * simultaneously turn
	 * @param x lateral movement
	 * @param y forward movement
	 * @param turn rotation amount
	 */
	public void setDirectionVector(double x, double y, double turn) {
		double m = Math.sqrt(x * x + y * y);
		double nx, ny;

		// De-normalize vector (x,y) into square space
		if (x == 0 || y == 0) {
			nx = x;
			ny = y;
		} else if (Math.abs(x) > Math.abs(y)) {
			nx = Math.signum(x) * m;
			ny = Math.signum(x) * y / x * m;
		} else {
			nx = Math.signum(y) * x / y * m;
			ny = Math.signum(y) * m;
		}
		setPower(ny - nx - turn, ny + nx - turn, ny + nx + turn, ny - nx + turn);
	}

	/**
	 * Sets the robot on a course headed in the direction of vector
	 * (x, y) relative to its current orientation with the option to
	 * simultaneously turn
	 * @param x lateral movement
	 * @param y forward movement
	 */
	public void setDirectionVector(double x, double y) {
		setDirectionVector(x, y, 0);
	}

	/**
	 * Restricts a number within the interval [min, max]
	 * @param num input number
	 * @param min range minimum
	 * @param max range maximum
	 * @return constrained number
	 */
	private double clamp(double num, double min, double max) {
		return Math.min(Math.max(num, min), max);
	}

	public double getLeftEncoder(){
		return wheels[0].getCurrentPosition();
	}

	public double getLeftVelocity(){
		return wheels[0].getVelocity();
	}

	public double getRightVelocity(){
		return wheels[2].getVelocity();
	}

	public double getRightEncoder(){
		return wheels[2].getCurrentPosition();
	}

	public void updatePosition(){

	}

}

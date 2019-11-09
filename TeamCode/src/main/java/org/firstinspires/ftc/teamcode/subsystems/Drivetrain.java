package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OrientationSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.localization.BasicOdometer;

public class Drivetrain {

	final String[] wheelNames = { "left_front", "left_back", "right_front", "right_back" };

	private static Drivetrain mecanumDrive;

	private HardwareMap hardwareMap;

	public DcMotorEx[] wheels;

	public BasicOdometer odometer;

	OrientationSensor imu;

	boolean isAutonEnabled = false;

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
			wheels[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		}

		wheels[0].setDirection(DcMotorSimple.Direction.REVERSE);
		wheels[1].setDirection(DcMotorSimple.Direction.REVERSE);

	}


	/**
	 * Set the encoder mode of each motor
	 * @param useEncoders whether or not to allow encoder use
	 */
	public void setEncoderState(boolean useEncoders) {
		for (int i = 0; i < wheelNames.length; i++) {
			wheels[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
		if (!isAutonEnabled) {
			wheels[0].setPower(clamp(fl, -1, 1));
			wheels[1].setPower(clamp(fr, -1, 1));
			wheels[2].setPower(clamp(bl, -1, 1));
			wheels[3].setPower(clamp(br, -1, 1));
		}
	}
	/**
	 * Sets the powers of each wheel
	 * @param fl front-left motor
	 * @param fr front-right motor
	 * @param bl back-left motor
	 * @param br back-right motor
	 */
	public void setAutonPower(double fl, double fr, double bl, double br) {
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

	public void setRunToEncoderMode(){
		for (int i = 0; i < wheelNames.length; i++) {
			wheels[i].setTargetPosition(-500);
			wheels[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
		}
	}

	public void resetEncoders(){
		for (int i = 0; i < wheelNames.length; i++) {
			wheels[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		}
	}

	public double getHeadingAngle(){
		return odometer.getHeading();
	}

	public void setLinearMotion(double leftMotor, double rightMotor, double finalPosition, double angle, boolean isGyroEnabled){
		isAutonEnabled = true;
	    while (Math.abs(getRightEncoder() + getLeftEncoder())/2 < Math.abs(finalPosition)){
            double angularError = getHeadingAngle() - angle;
            double angularCorrection = angularError * 0.02;
            setAutonPower(leftMotor - angularCorrection, rightMotor + angularCorrection, leftMotor - angularCorrection, rightMotor + angularCorrection);
        }
	    setAutonPower(0,0,0,0);
	    isAutonEnabled = false;
    }

    public void turnToAngle(double expectedAngle){
		double angleDiff = expectedAngle;
		isAutonEnabled = true;

		while (Math.abs(angleDiff) > 3) {
			double kP = 0.007;
			angleDiff = getHeadingAngle() - expectedAngle;
			double corection = angleDiff * kP;
			setAutonPower(corection, corection, -corection, -corection);
		}
		setAutonPower(0,0,0,0);
		isAutonEnabled = false;
	}

    public void setRunToEncoderMotion(double power, int finalPosition, double angle, boolean isGyroEnabled){
		isAutonEnabled = true;
        for (int i = 0; i < wheelNames.length; i++) {
            wheels[i].setTargetPosition(finalPosition);
            wheels[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
		setAutonPower(power,power,power,power);
    }

    public void setStrafeTime(double power, double time){
		isAutonEnabled = true;
		ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
		while (elapsedTime.seconds() > time){
			setAutonPower(-power, power,power,-power);
		}
		setAutonPower(0,0,0,0);
		isAutonEnabled = false;
	}

	public void setStrafePosition(int position, double power){
		isAutonEnabled = true;
		wheels[0].setTargetPosition(-position);
		wheels[1].setTargetPosition(position);
		wheels[2].setTargetPosition(position);
		wheels[3].setTargetPosition(-position);

		for (int i = 0; i < wheelNames.length; i++) {
			wheels[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
			wheels[i].setPower(power);
		}

	}

}

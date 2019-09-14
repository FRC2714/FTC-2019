package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive {

	final String[] wheelNames = { "FrontL", "FrontR", "BackL", "BackR" };

	public static boolean useEncoders = false;

	private MecanumDrive mecanumDrive;

	private HardwareMap hardwareMap;

	private DcMotor[] wheels;

	public MecanumDrive getInstance(HardwareMap hm) {
		if (mecanumDrive == null) mecanumDrive = new MecanumDrive(hm);
		return mecanumDrive;
	}

	private MecanumDrive(HardwareMap hm) {
		hardwareMap = hm;

		wheels = new DcMotor[wheelNames.length];

		for (int i = 0; i < wheelNames.length; i++) {
			wheels[i] = hardwareMap.dcMotor.get(wheelNames[i]);
			wheels[i].setPower(0);
			wheels[i].setMode(useEncoders ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		}
	}

	public void setPower(double fl, double fr, double bl, double br) {
		wheels[0].setPower(fl);
		wheels[1].setPower(fr);
		wheels[2].setPower(bl);
		wheels[3].setPower(br);
	}

	public void setDirectionVector(double x, double y, double turn) {

	}
}

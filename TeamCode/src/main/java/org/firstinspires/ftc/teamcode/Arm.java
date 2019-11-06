package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    private static Arm arm;

    private HardwareMap hardwareMap;

    private DcMotorEx armMotor;
    private DcMotorEx intakeMotor;
    private Servo intakeServo;
    private AnalogInput arm_potentiometer;

    private boolean servoState, firstTime;

    public static Arm getInstance(HardwareMap hm) {
        if (arm == null) arm = new Arm(hm);
        return arm;
    }

    public Arm(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        armMotor = this.hardwareMap.get(DcMotorEx.class, "arm_motor");
        intakeMotor = this.hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeServo = this.hardwareMap.get(Servo.class, "intake_servo");
        arm_potentiometer = this.hardwareMap.get(AnalogInput.class, "arm_potentiometer");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoState = false;
        firstTime = true;
    }

    public void setArmMotor(double speed){
        armMotor.setPower(speed);
    }

    public void setIntakeMotor(double speed){
        intakeMotor.setPower(speed);
    }

    public void goToPosition(double position) {

    }

    public void setServo(double servoVal){
        intakeServo.setPosition(servoVal);
    }

    public void setIntakeServo() {
        if (firstTime || !servoState) {
            firstTime = false;
            servoState = true;
            intakeServo.setPosition(1.0);
        } else {
            servoState = false;
            intakeServo.setPosition(0.0);
        }
    }

    public double getArmMotorEncoderPosition(){
        return armMotor.getCurrentPosition();
    }

    public double getArmPotentiometerPosition(){
        return arm_potentiometer.getVoltage();
    }

    public void resetEncoders(){
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}

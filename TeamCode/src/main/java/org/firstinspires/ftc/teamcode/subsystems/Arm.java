package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm {

    private static Arm arm;
    private boolean isArmMotionProfiling = false;
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
        if(!isArmMotionProfiling)
            armMotor.setPower(speed);
    }

    public void setIntakeMotor(double speed){
        intakeMotor.setPower(speed);
    }

    public void setIntakeMotor(double speed, double time){
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        intakeMotor.setPower(speed);
        while (timer.seconds() < time){

        }
        intakeMotor.setPower(0);
    }

    public void goToPosition(ArmPosition position, double power) {
        isArmMotionProfiling = true;
        switch (position){
            case STARTING:
                armMotor.setTargetPosition(0);
                break;
            case SAFE_HOLD:
                armMotor.setTargetPosition(-330);
                break;
            case HIGH_HOLD:
                armMotor.setTargetPosition(-250);
                break;
            case STONE_PICKUP:
                armMotor.setTargetPosition(-1300);
                break;
        }
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(power);
        while (!(Math.abs(armMotor.getTargetPosition() - armMotor.getCurrentPosition()) < 50)){

        }

    }

    public void setServo(ServoPosition servoPosition){
        switch (servoPosition){
            case RELAXED:
                intakeServo.setPosition(1);
                break;
            case PRESSURE_STONE:
                intakeServo.setPosition(0.68);
                break;
        }
    }

    public void setIntakeServo() {
        if (firstTime || !servoState) {
            firstTime = false;
            servoState = true;
            intakeServo.setPosition(1.0);
        } else {
            servoState = false;
            intakeServo.setPosition(0.68);
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
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update(){
        if(isArmMotionProfiling)
            if(Math.abs(armMotor.getTargetPosition() - armMotor.getCurrentPosition()) < 50) {
                isArmMotionProfiling = false;
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
    }

    public enum ArmPosition {
        STARTING,
        SAFE_HOLD,
        HIGH_HOLD,
        STONE_PICKUP
        }

    public enum ServoPosition {
        PRESSURE_STONE,
        RELAXED,
    }

}

package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class BasicOdometer extends Thread{

    private DcMotor xMotor;
    private DcMotor yMotor;
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    private double initAngle;

    public BasicOdometer(DcMotor xMotor, DcMotor yMotor, HardwareMap map) {
        this.xMotor = xMotor;
        this.yMotor = yMotor;
        imu = map.get(BNO055IMU.class, "imu");
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(parameters);
        initAngle = imu.
                getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public BasicOdometer(HardwareMap map) {
        this.xMotor = xMotor;
        this.yMotor = yMotor;
        imu = map.get(BNO055IMU.class, "imu");
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(parameters);
        initAngle = imu.
                getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }



    public double getHeading(){
        Orientation orientation =  imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return orientation.firstAngle;
    }

    


}

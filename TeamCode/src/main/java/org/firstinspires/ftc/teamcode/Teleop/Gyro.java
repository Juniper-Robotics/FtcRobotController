package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Gyro {
    private double p, i,d;
    private double porportional, integral, derivate, error, totalError = 0, lastError, integralActiveZone =2;
   private double current = 0;
    private helpDrive robot;
    private BNO055IMU imu;
    private Orientation angles;
    BNO055IMU.Parameters parameters;

    public Gyro( BNO055IMU imu, Orientation angles, double p, double i, double d, helpDrive robot){

        this.p = p;
        this.i = i;
        this.d = d;
        this.robot = robot;
        this.imu = imu;
        this.angles = angles;

        parameters = new BNO055IMU.Parameters();//new parameters opbejct
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;//sertting parameter to degrees
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu.initialize(parameters);

    }

    public void rotate(double desired) throws InterruptedException {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        current = porportional + integral + derivate;
        error = Math.abs(angles.firstAngle - desired);

        while(error !=0){

            current = porportional + integral + derivate;
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            error = angles.firstAngle - desired;

            if(error < integralActiveZone && error!=0){
                totalError += error;
            }else{totalError = 0;}
            if(error == 0){
                derivate = 0;
            }

            porportional = error*p;
            integral = totalError*i;
            derivate = (error - lastError) * d;
            lastError = error;
            robot.setPowers(current, current, -current, - current);
            Thread.sleep(30);
    }


}}


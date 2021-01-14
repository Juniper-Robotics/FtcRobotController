package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Gyro {
    private double p, i,d;
    private double porportional, integral, derivate, error, totalError = 0, lastError, integralActiveZone =2;
   private double current = 0;
    private helpDrive Gerald;
    private BNO055IMU spinyboy;
    private Orientation angles;
    BNO055IMU.Parameters parameters;


    public Gyro( BNO055IMU spinyboi, Orientation angles, double p, double i, double d, helpDrive Gerald){

        this.p = p;
        this.i = i;
        this.d = d;
        this.Gerald = Gerald;
        this.spinyboy = spinyboy;
        this.angles = angles;

        parameters = new BNO055IMU.Parameters();//new parameters opbejct
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;//sertting parameter to degrees
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        spinyboy.initialize(parameters);

    }

    //attempting PID idk how to even tune so dunno why im coding this
    // p is times
    //i is pluses
    //d is minuses
    //code for doing the roatet
    public void rotate(double desired) throws InterruptedException {
        angles = spinyboy.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        error = desired - angles.firstAngle ;

        while(error < 0.01){
            angles = spinyboy.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            error = desired - angles.firstAngle ;

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
            current = porportional + integral + derivate;
            Gerald.setPowers(current, current, -current, - current);
            Thread.sleep(30);
    }
}}


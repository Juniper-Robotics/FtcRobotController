package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Gyro {
    private double p, i,d;
    private double porportional, integral, derivate, error, totalError = 0, lastError, integralActiveZone =11;
   private double current = 0;
    private helpDrive Gerald;
    private BNO055IMU spinyboy;
    private Orientation angles;
    BNO055IMU.Parameters parameters;
    PIDCoefficients pid;
    Telemetry tele;

    public Gyro(BNO055IMU spinyboii, Orientation angles, double pi, double ii, double di, helpDrive Gerald, Telemetry telemetry){

        p = pi;
        i = ii;
        d = di;
        this.Gerald = Gerald;
        spinyboy = spinyboii;
        this.angles = angles;
        tele = telemetry;

        parameters = new BNO055IMU.Parameters();//new parameters opbejct
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;//sertting parameter to degrees
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        spinyboy.initialize(parameters);

    }


    public Gyro(BNO055IMU spinyboi, Orientation angles, helpDrive Gerald, PIDCoefficients pid, Telemetry telemetry){
        this.Gerald = Gerald;
        this.spinyboy = spinyboi;
        this.angles = angles;

        tele = telemetry;
        this.pid = pid;
        parameters = new BNO055IMU.Parameters();//new parameters opbejct
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;//sertting parameter to degrees
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        spinyboy.initialize(parameters);
    }

    public float returnAngle(){
        angles = spinyboy.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    //attempting PID idk how to even tune so dunno why im coding this
    // p is times
    //i is pluses
    //d is minuses
    //code for doing the roatet
    public void rotate(double desired) throws InterruptedException {
        angles = spinyboy.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        error = desired - angles.firstAngle ;
        long start = System.currentTimeMillis();

        while(Math.abs(error) > 2.5){
            tele.addData("p",p);
            tele.addData("i",i);
            tele.addData("desired",desired);
            angles = spinyboy.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            error = desired - angles.firstAngle ;
            tele.addData("error",error);
            if(Math.abs(error) < integralActiveZone && error!=0){
                totalError += error;
            }else{totalError = 0;}
            if(error == 0){
                derivate = 0;
            }

            porportional = error*p;
            integral = totalError*i;
            tele.addData("total error", totalError);
            long end = System.currentTimeMillis();
            long elapsedTime = end - start;
            derivate = ((error-lastError)/elapsedTime)*d;
           // derivate = (error - lastError) * d;//time divide becasue change of rate of error
            //x is time adn y is error (error current-error last)/time current/tiem last
            tele.addData("angle1",angles.firstAngle);
            lastError = error;
            current = porportional + integral + derivate;
            if(angles.firstAngle>desired){
            Gerald.setPowers(current, current, -current, - current);
            tele.addData("hi",0);
            }
            else if (angles.firstAngle<desired){
                tele.addData("bye",0);
                Gerald.setPowers(-current, -current, current,  current);
            }
            tele.update();
            //way to breakout
    }
}
    public void rotate2(double desired) throws InterruptedException {
        angles = spinyboy.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        error = desired - angles.firstAngle ;
        long start = System.currentTimeMillis();

        while(Math.abs(error) > 2.5){
            tele.addData("p",p);
            tele.addData("i",i);
            tele.addData("desired",desired);
            angles = spinyboy.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            error = desired - angles.firstAngle ;
            tele.addData("error",error);
            if(Math.abs(error) < integralActiveZone && error!=0){
                totalError += error;
            }else{totalError = 0;}
            if(error == 0){
                derivate = 0;
            }

            porportional = error*p;
            integral = totalError*i;
            tele.addData("total error", totalError);
            long end = System.currentTimeMillis();
            long elapsedTime = end - start;
            derivate = ((error-lastError)/elapsedTime)*d;
            // derivate = (error - lastError) * d;//time divide becasue change of rate of error
            //x is time adn y is error (error current-error last)/time current/tiem last
            tele.addData("angle1",angles.firstAngle);
            lastError = error;
            current = porportional + integral + derivate;

                Gerald.setPowers(-current, -current, current, current);
                tele.addData("hi",0);

            tele.update();
            //way to breakout
        }
    }}

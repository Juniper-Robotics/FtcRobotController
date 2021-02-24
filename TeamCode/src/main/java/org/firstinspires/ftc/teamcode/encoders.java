package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

//@Autonomous

public class encoders  {

    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private BNO055IMU imu;
    private myMecnam mecam;
    private double integralActiveZone = 2.5;


    public encoders(DcMotor leftBackMotor, DcMotor rightBackMotor, DcMotor leftFrontMotor, DcMotor rightFrontMotor , BNO055IMU imu, myMecnam mecam){
        this.leftBackMotor = leftBackMotor;
        this.rightBackMotor = rightBackMotor;
        this.leftFrontMotor = leftFrontMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.imu = imu;
        this.mecam = mecam;
    }

    //pass in opmode
    public encoders(DcMotor leftBackMotor, DcMotor rightBackMotor, DcMotor leftFrontMotor, DcMotor rightFrontMotor){
        this.leftBackMotor = leftBackMotor;
        this.rightBackMotor = rightBackMotor;
        this.leftFrontMotor = leftFrontMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.imu = imu;
        this.mecam = mecam;
    }
    public void runOpMode() throws InterruptedException {
        //sets Motor equal to "motorOne" which is announce on the robot
        rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runToPosition(){
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

  /*  public void reset(){
        leftBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBackMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
    }*/

    public void forward(int ticks){
        rightBackMotor.setTargetPosition(ticks);
        leftBackMotor.setTargetPosition(ticks);
        rightFrontMotor.setTargetPosition(ticks);
        leftFrontMotor.setTargetPosition(ticks);
        this.runToPosition();

        while(rightBackMotor.isBusy() && leftBackMotor.isBusy() && rightFrontMotor.isBusy() && leftFrontMotor.isBusy()){

        }

        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void backward(int ticks){
        ticks = -ticks;
        rightBackMotor.setTargetPosition(ticks);
        leftBackMotor.setTargetPosition(ticks);
        rightFrontMotor.setTargetPosition(ticks);
        leftFrontMotor.setTargetPosition(ticks);
        this.runToPosition();
    }

    public void right(int ticks){
            rightBackMotor.setTargetPosition(ticks);
            leftBackMotor.setTargetPosition(-ticks);
            rightFrontMotor.setTargetPosition(-ticks);
            leftFrontMotor.setTargetPosition(ticks);
            this.runToPosition();

    }

    public void left(int ticks){
        rightBackMotor.setTargetPosition(-ticks);
        leftBackMotor.setTargetPosition(ticks);
        rightFrontMotor.setTargetPosition(ticks);
        leftFrontMotor.setTargetPosition(-ticks);
        this.runToPosition();
    }
    public void stop(){
        rightBackMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftFrontMotor.setPower(0);
    }

    public void turnLeft(int ticks){
        rightBackMotor.setTargetPosition(ticks);
        leftBackMotor.setTargetPosition(-ticks);
        rightFrontMotor.setTargetPosition(ticks);
        leftFrontMotor.setTargetPosition(-ticks);
        this.runToPosition();
    }

    public void turnRight(int ticks){
        rightBackMotor.setTargetPosition(-ticks);
        leftBackMotor.setTargetPosition(ticks);
        rightFrontMotor.setTargetPosition(-ticks);
        leftFrontMotor.setTargetPosition(ticks);
        this.runToPosition();
    }


    //going spin and then
    public void goTo(Pose2d distance, PIDCoefficients pid, Telemetry telemetry) throws InterruptedException {

        Pose2d now = mecam.getPoseEstimate();
        Pose2d go = distance;
        double current = 0;
        double x = go.getX() - now.getX();//difference at start
        double y = go.getY() - now.getY();//difference at start
        boolean backorfor;
        if(x<0){
            backorfor = true;
        }else{backorfor=false;}

        double error = Math.hypot(x,y);
        double derivate = 0, porportional = 0, integral = 0;
        long start = System.currentTimeMillis();
        double totalError = 0;
        double lastError = 0;

        double angle = Math.atan(x/y);
        telemetry.addData("angle want", angle);

        //angle
        mecam.spinyBoi.rotate(angle);

        telemetry.addData("angle",angle);
        float startAngle = mecam.spinyBoi.returnAngle();
        double xError = x;
        double yError = y;
        while (/*Math.abs(error) > 7*/  xError>6||yError>8)
        {
           telemetry.addData("x",now.getX());
           telemetry.addData("y",now.getY());
            now = mecam.getPoseEstimate();
            mecam.updatePoseEstimate();
            //distance of change with x and y now is origanl starting

            x = distance.getX() - now.getX();//x difference
            y = distance.getY() - now.getY();//y edifference
            telemetry.addData("xerror",x);
            telemetry.addData("yerror",y);

            xError = x;
            yError = y;

            error = Math.hypot(x,y); //noww is always the total error from begiing; disatcn
            //error = distanc-noww;
            telemetry.addData("error",error);
            if (Math.abs(error) < integralActiveZone && error != 0) {
                totalError += Math.abs(x);
            } else {
                totalError = 0;
            }
            if (x == 0) {
                derivate = 0;
            }

            //create pidf controller
            //paramters are the PIDCOefficants
            porportional = error * pid.p;
            integral = totalError * pid.i;
            long end = System.currentTimeMillis();
            long elapsedTime = end - start;
            start = end;
            derivate = ((x-lastError)/elapsedTime) * pid.d;
            lastError = error;
            telemetry.addData("p",porportional);
            telemetry.addData("i",integral);
            telemetry.addData("total Error",totalError);
            current = porportional + integral + derivate;
            telemetry.addData("lastError", lastError);
            telemetry.addData("current",current);
            if(backorfor)
            {
                if(error>lastError+3){
                    telemetry.addData("bigeer","ew");
                 mecam.gerlad.setPowers(-current,-current, -current,-current);
                 }else{
                    telemetry.addData("smaller",9);
                     mecam.gerlad.setPowers(current/2,current/2, current/2, current/2);
                    }
            }else
             {
                if(error>lastError+3){
                    telemetry.addData("bigeer","ew");
                mecam.gerlad.setPowers(current,current, current,current);
                }else{
                    telemetry.addData("smaller",9);
                mecam.gerlad.setPowers(-current/2, -current/2, -current/2, -current/2);
                }
            }
            //Thread.sleep(30);

            //mecam.gerlad.setPowers(current, current, current, current);
            telemetry.update();
        }


    }

    public void goTo2(Pose2d distance, PIDCoefficients pid, Telemetry telemetry) throws InterruptedException {

        Pose2d now = mecam.getPoseEstimate();
        Pose2d go = distance;
        double current = 0;
        double x = go.getX() - now.getX();//difference at start
        double y = go.getY() - now.getY();//difference at start
        boolean backorfor;
        if(x<0){
            backorfor = true;
        }else{backorfor=false;}

        double error = Math.hypot(x,y);
        double derivate = 0, porportional = 0, integral = 0;
        long start = System.currentTimeMillis();
        double totalError = 0;
        double lastError = 0;

        double angle = Math.atan(x/y);
        telemetry.addData("angle want", angle);


        telemetry.addData("angle",angle);
        float startAngle = mecam.spinyBoi.returnAngle();
        double xError = x;
        double yError = y;
        while (/*Math.abs(error) > 7*/  xError>5)
        {
            telemetry.addData("x",now.getX());
            telemetry.addData("y",now.getY());
            now = mecam.getPoseEstimate();
            mecam.updatePoseEstimate();
            //distance of change with x and y now is origanl starting

            x = distance.getX() - now.getX();//x difference
            y = distance.getY() - now.getY();//y edifference
            telemetry.addData("xerror",x);
            telemetry.addData("yerror",y);

            xError = x;
            yError = y;

            error = Math.hypot(x,y); //noww is always the total error from begiing; disatcn
            //error = distanc-noww;
            telemetry.addData("error",error);
            if (Math.abs(error) < integralActiveZone && error != 0) {
                totalError += Math.abs(x);
            } else {
                totalError = 0;
            }
            if (x == 0) {
                derivate = 0;
            }

            //create pidf controller
            //paramters are the PIDCOefficants
            porportional = error * pid.p;
            integral = totalError * pid.i;
            long end = System.currentTimeMillis();
            long elapsedTime = end - start;
            start = end;
            derivate = ((x-lastError)/elapsedTime) * pid.d;
            lastError = error;
            telemetry.addData("p",porportional);
            telemetry.addData("i",integral);
            telemetry.addData("total Error",totalError);
            current = porportional + integral + derivate;
            telemetry.addData("lastError", lastError);
            telemetry.addData("current",current);
            if(backorfor)
            {
                if(error>lastError+3){
                    telemetry.addData("bigeer","ew");
                    mecam.gerlad.setPowers(-current,-current, -current,-current);
                }else{
                    telemetry.addData("smaller",9);
                    mecam.gerlad.setPowers(current/2,current/2, current/2, current/2);
                }
            }else
            {
                if(error>lastError+3){
                    telemetry.addData("bigeer","ew");
                    mecam.gerlad.setPowers(current,current, current,current);
                }else{
                    telemetry.addData("smaller",9);
                    mecam.gerlad.setPowers(-current/2, -current/2, -current/2, -current/2);
                }
            }
            //Thread.sleep(30);

            //mecam.gerlad.setPowers(current, current, current, current);
            telemetry.update();
        }


    }


}

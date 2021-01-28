package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

//@Autonomous

public class encoders  {

    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private BNO055IMU imu;
    private myMecnam mecam;
    private double integralActiveZone = 2;


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

    public void forward(int ticks){
        rightBackMotor.setTargetPosition(ticks);
        leftBackMotor.setTargetPosition(ticks);
        rightFrontMotor.setTargetPosition(ticks);
        leftFrontMotor.setTargetPosition(ticks);
        this.runToPosition();
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

    //two approaches:
    //-going y and then x
    //going spin and then
    public void goTo(Pose2d distance, PIDCoefficients pid) throws InterruptedException {
        Pose2d now = mecam.getPoseEstimate();
        Pose2d go = distance;
        double current = 0;
        double x = go.getX() - now.getX();
        double y = go.getY() - now.getY();

        double distanc = Math.hypot(x,y);
        double noww = 0;
        double error = distanc-noww;
        double derivate = 0, porportional = 0, integral = 0;
        long start = System.currentTimeMillis();
        double totalError = 0;
        double lastError = 0;

        double angle = Math.atan(x/y);
        mecam.spinyBoi.rotate(angle);
        float startAngle = mecam.spinyBoi.returnAngle();
        float nowAngle;

        while (Math.abs(error) > 0.01 )
        {
            nowAngle = mecam.spinyBoi.returnAngle();
            //distance of change with x and y now is origanl starting
            now = mecam.getPoseEstimate();
            x = go.getX() - now.getX();
            y = go.getY() - now.getY();
            noww = Math.hypot(x,y);
            error = distanc-noww;

            if (error < integralActiveZone && error != 0) {
                totalError += x;
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
            current = porportional + integral + derivate;
            mecam.gerlad.setPowers(current, current, current, current);
            //Thread.sleep(30);
            if(Math.abs(nowAngle-startAngle)>1){
                mecam.spinyBoi.rotate(angle);
            }
            mecam.gerlad.setPowers(current, current, current, current);
        }

    }


}

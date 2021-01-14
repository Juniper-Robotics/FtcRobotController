package org.firstinspires.ftc.teamcode.Autanomos;

import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

//@Autonomous
public class encoders  {

    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    public encoders(DcMotor leftBackMotor, DcMotor rightBackMotor, DcMotor leftFrontMotor, DcMotor rightFrontMotor){
        this.leftBackMotor = leftBackMotor;
        this.rightBackMotor = rightBackMotor;
        this.leftFrontMotor = leftFrontMotor;
        this.rightFrontMotor = rightFrontMotor;
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


}

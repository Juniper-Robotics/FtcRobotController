package org.firstinspires.ftc.teamcode.Autanomos;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class encodr extends LinearOpMode {
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private BNO055IMU imu;

    public void runOpMode() throws InterruptedException {

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

        waitForStart();
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //MAKE SURE ALL BACKWARDS
        rightBackMotor.setTargetPosition(-500);
        leftBackMotor.setTargetPosition(-500);
        rightFrontMotor.setTargetPosition(-500);
        leftFrontMotor.setTargetPosition(-500);

        int leftFrontPos = leftBackMotor.getCurrentPosition();
        int leftBackPos = leftBackMotor.getCurrentPosition();
        int rightFrontPos = rightFrontMotor.getCurrentPosition();
        int rightBackPos = rightBackMotor.getCurrentPosition();

        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);




       while(leftBackPos<500 && leftFrontPos<500&&rightBackPos<500&&rightFrontPos<500 && opModeIsActive()){
           rightBackMotor.setPower(0.005);
           leftBackMotor.setPower(0.005);
           rightFrontMotor.setPower(0.005);
           leftFrontMotor.setPower(0.005);

           leftFrontPos = leftBackMotor.getCurrentPosition();
            leftBackPos = leftBackMotor.getCurrentPosition();
            rightFrontPos = rightFrontMotor.getCurrentPosition();
            rightBackPos = rightBackMotor.getCurrentPosition();


           telemetry.addData("poslb",leftBackMotor.getCurrentPosition());
           telemetry.addData("poslf",leftFrontMotor.getCurrentPosition());
           telemetry.addData("posrb",rightBackMotor.getCurrentPosition());
           telemetry.addData("poslb",leftBackMotor.getCurrentPosition());

           telemetry.update();
        }

        rightBackMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftFrontMotor.setPower(0);



        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}}

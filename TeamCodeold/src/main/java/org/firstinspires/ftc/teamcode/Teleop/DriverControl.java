package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Teleop.Direction;
import org.firstinspires.ftc.teamcode.Teleop.slowDown;

@TeleOp
public class DriverControl extends LinearOpMode
{
    //this is initializing ex)int number = DcMotor Motor
   private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;


    @Override
    public void runOpMode() //throws InterruptedException
    {
        //sets Motor equal to "motorOne" which is announce on the robot
       rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");


        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
         waitForStart();

             while(opModeIsActive())
    {
        rightBackMotor.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
        rightFrontMotor.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
        leftBackMotor.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
        leftFrontMotor.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);


    }


    }}









 /*waitForStart();

        while(opModeIsActive()){
            while(gamepad1.dpad_down){
                Direction.backward(leftBackMotor,rightBackMotor, leftFrontMotor, rightFrontMotor); }
            while(gamepad1.dpad_up){
                Direction.forward(leftBackMotor,rightBackMotor, leftFrontMotor, rightFrontMotor);}
            while(gamepad1.dpad_right){
                Direction.right(leftBackMotor,rightBackMotor, leftFrontMotor, rightFrontMotor);}
            while(gamepad1.dpad_left){
                Direction.left(leftBackMotor,rightBackMotor, leftFrontMotor, rightFrontMotor);}

            while(gamepad1.right_trigger>0){
                Direction.setPower(leftBackMotor,rightBackMotor, leftFrontMotor, rightFrontMotor, gamepad1.right_trigger);}
            while(gamepad1.right_trigger>0){
                Direction.setPower(leftBackMotor,rightBackMotor, leftFrontMotor, rightFrontMotor, -gamepad1.left_trigger);}

            while(gamepad1.right_bumper){
                slowDown.slowRight(leftBackMotor, rightBackMotor, leftFrontMotor, rightFrontMotor);}
            while(gamepad1.left_bumper){
                slowDown.slowLeft(leftBackMotor,rightBackMotor, leftFrontMotor, rightFrontMotor);}

            }
        }*/
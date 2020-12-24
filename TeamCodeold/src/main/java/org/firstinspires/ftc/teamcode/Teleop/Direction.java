package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Direction{
    public static void forward(DcMotor leftBackMotor, DcMotor rightBackMotor, DcMotor leftFrontMotor, DcMotor rightFrontMotor){
        rightBackMotor.setPower(0.8);
        rightFrontMotor.setPower(0.8);
        leftBackMotor.setPower(0.8);
        leftFrontMotor.setPower(0.8);
    }

    public static void backward(DcMotor leftBackMotor, DcMotor rightBackMotor, DcMotor leftFrontMotor, DcMotor rightFrontMotor){
        rightBackMotor.setPower(-0.8);
        rightFrontMotor.setPower(-0.8);
        leftBackMotor.setPower(-0.8);
        leftFrontMotor.setPower(-0.8);
    }

    public static void right(DcMotor leftBackMotor, DcMotor rightBackMotor, DcMotor leftFrontMotor, DcMotor rightFrontMotor){
        rightBackMotor.setPower(-0.8);
        rightFrontMotor.setPower(0.8);
        leftBackMotor.setPower(0.8);
        leftFrontMotor.setPower(-0.8);
    }

    public static void left(DcMotor leftBackMotor, DcMotor rightBackMotor, DcMotor leftFrontMotor, DcMotor rightFrontMotor){
        rightBackMotor.setPower(0.8);
        rightFrontMotor.setPower(-0.8);
        leftBackMotor.setPower(-0.8);
        leftFrontMotor.setPower(0.8);
    }

    public static void setPower(DcMotor leftBackMotor, DcMotor rightBackMotor, DcMotor leftFrontMotor, DcMotor rightFrontMotor, double speed){
        rightBackMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        leftFrontMotor.setPower(speed);
    }


}

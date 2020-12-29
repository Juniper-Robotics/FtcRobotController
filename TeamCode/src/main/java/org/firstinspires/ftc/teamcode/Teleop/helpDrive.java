package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.hardware.DcMotor;

public class helpDrive {
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;

    public helpDrive(DcMotor leftBackMotors, DcMotor rightBackMotors, DcMotor leftFrontMotors,  DcMotor rightFrontMotors){
        leftBackMotor = leftBackMotors;
        rightBackMotor = rightBackMotors;
        leftFrontMotor = leftFrontMotors;
        rightFrontMotor = rightFrontMotors;
    }

    public void forward(double speed){
        rightBackMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        leftFrontMotor.setPower(speed);
    }

    public void backward(double speed){
        rightBackMotor.setPower(-speed);
        rightFrontMotor.setPower(-speed);
        leftBackMotor.setPower(-speed);
        leftFrontMotor.setPower(-speed);
    }

    public void right(double speed){
        rightBackMotor.setPower(speed);
        rightFrontMotor.setPower(-speed);
        leftBackMotor.setPower(-speed);
        leftFrontMotor.setPower(speed);
    }

    public void left(double speed){
        rightBackMotor.setPower(-speed);
        rightFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        leftFrontMotor.setPower(-speed);
    }

    public void turnRight(double speed){
            rightBackMotor.setPower(-speed/2);
            rightFrontMotor.setPower(-speed/2);
            leftBackMotor.setPower(speed/2);
            leftFrontMotor.setPower(speed/2);
    }

    public void turnLeft(double speed){
            rightBackMotor.setPower(speed/2);
            rightFrontMotor.setPower(speed/2);
            leftBackMotor.setPower(-speed/2);
            leftFrontMotor.setPower(-speed/2);
    }

    public void setPowers(double rbp, double rfp, double lbp, double lfp){
        rightBackMotor.setPower(rbp);
        rightFrontMotor.setPower(rfp);
        leftBackMotor.setPower(lbp);
        leftFrontMotor.setPower(lfp);
    }
}

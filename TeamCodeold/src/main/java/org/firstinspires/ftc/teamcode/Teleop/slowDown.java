package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.hardware.DcMotor;

public class slowDown {
    public static void slowRight(DcMotor leftBackMotor, DcMotor rightBackMotor, DcMotor leftFrontMotor, DcMotor rightFrontMotor){
        rightBackMotor.setPower(-0.3);
        rightFrontMotor.setPower(-0.3);
        leftBackMotor.setPower(0.3);
        leftFrontMotor.setPower(0.3);
    }

    public static void slowLeft(DcMotor leftBackMotor, DcMotor rightBackMotor, DcMotor leftFrontMotor, DcMotor rightFrontMotor){
        rightBackMotor.setPower(0.3);
        rightFrontMotor.setPower(0.3);
        leftBackMotor.setPower(-0.3);
        leftFrontMotor.setPower(-0.3);
    }


}

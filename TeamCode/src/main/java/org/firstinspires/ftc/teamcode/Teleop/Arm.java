package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

@TeleOp
public class Arm extends LinearOpMode{
    private DcMotor arm;
    private DcMotor arm2;
    private  DcMotor arm3;

    public void runOpMode(){
        arm = hardwareMap.dcMotor.get("motorOne");
       arm2 = hardwareMap.dcMotor.get("motorTwo");
       arm3 = hardwareMap.dcMotor.get("motorThree");
        double speed = 0.31;
        waitForStart();

        while(opModeIsActive()) {
            arm.setPower(-speed);
            arm2.setPower(speed*2.5);
            arm3.setPower(-speed*2.5);


        }
    }
}

/*while(gamepad1.right_bumper){
                speed+=0.01;
                sleep(500);
            }

            while(gamepad1.left_bumper){
                speed-=0.01;
                sleep(500);
            }
            telemetry.addData("Speed: ", speed);*/

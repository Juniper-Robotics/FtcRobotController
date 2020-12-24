package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

@TeleOp
public class Arm extends LinearOpMode{
    private DcMotor arm;

    public void runOpMode(){
        arm = hardwareMap.dcMotor.get("armMotor");
        double speed = 0.31;
        waitForStart();

        while(opModeIsActive()) {
            arm.setPower(speed);


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

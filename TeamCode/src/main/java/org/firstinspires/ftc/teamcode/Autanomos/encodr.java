package org.firstinspires.ftc.teamcode.Autanomos;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.WobbleGoalArm;

@Autonomous
public class encodr extends LinearOpMode {
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private BNO055IMU imu;



    public void runOpMode() throws InterruptedException {
        Shooter shooter = new Shooter(hardwareMap, telemetry);
        WobbleGoalArm wobbleArm= new WobbleGoalArm(hardwareMap);
        PIDCoefficients PID= new PIDCoefficients(0.0252,0.003,0);
        boolean shootOn = true;
        boolean launch = true;
        waitForStart();

        shooter.setShooterServoLaunch();
        int i = 1;
        while(shootOn){
            shooter.on();
            sleep(1500);

            shooter.setShooterServoLaunch();

            sleep(1000);
            shooter.setShooterServoReset();
            i++;
            if(i==3)
            {
                shootOn = false;
            }


        }
        telemetry.addData("i",0);
        wobbleArm.off();
        telemetry.addData("e",0);
        sleep(4000);
    }
}
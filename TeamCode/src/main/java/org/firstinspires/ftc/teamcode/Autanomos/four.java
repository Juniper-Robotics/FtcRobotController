package org.firstinspires.ftc.teamcode.Autanomos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Gyro;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.WobbleGoalArm;
import org.firstinspires.ftc.teamcode.helpDrive;
import org.firstinspires.ftc.teamcode.encoders;

import static java.lang.Thread.sleep;

public class four{

    public static void BlueOne(encoders robot, Telemetry telemetry, Gyro spinyboi, HardwareMap hardwareMap) throws InterruptedException {
        Shooter shooter = new Shooter(hardwareMap, telemetry);
        WobbleGoalArm wobbleArm= new WobbleGoalArm(hardwareMap);
        PIDCoefficients PID= new PIDCoefficients(0.025,0.003,0);
        boolean shootOn = false;
        Pose2d pose1 = new Pose2d(20,7);
        Pose2d pose2 = new Pose2d(50,7);


        robot.goTo(pose1, PID,telemetry);
        robot.stop();
        spinyboi.rotate(10);
        shootOn = true;
        boolean launch = true;
        int i = 0;
        while(shootOn){
            shooter.on2();
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
        shooter.off();
        robot.goTo(pose2,PID,telemetry);
        robot.stop();
        spinyboi.rotate(0);

    }

    public static void BlueOne(helpDrive robot) throws InterruptedException {
        robot.forward(0.4,3300);
        robot.StopDriving();
        sleep(1500);
        robot.turnLeft(0.6);
        sleep(1750);
        robot.StopDriving();
        sleep(1500);
        robot.forward(0.4,900);
        robot.StopDriving();
        sleep(400);
        robot.backward(0.4,1000);
        robot.StopDriving();
        sleep(1000);
        robot.turnLeft(0.6);
        sleep(1600);
        robot.StopDriving();
        sleep(1550);
        robot.forward(0.4,1500);
        robot.StopDriving();
    }


}
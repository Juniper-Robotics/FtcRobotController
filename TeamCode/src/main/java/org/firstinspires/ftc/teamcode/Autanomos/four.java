package org.firstinspires.ftc.teamcode.Autanomos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Gyro;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.WobbleGoalArm;
import org.firstinspires.ftc.teamcode.encoders;
import org.firstinspires.ftc.teamcode.helpDrive;
import org.firstinspires.ftc.teamcode.myMecnam;

import static java.lang.Thread.sleep;

public class four{

    public static void BlueOne(encoders robot, Telemetry telemetry, Gyro spinyboi, HardwareMap hardwareMap , myMecnam mecam) throws InterruptedException {
        telemetry.addData("four","i hope this is four");
        Shooter shooter = new Shooter(hardwareMap, telemetry);
        WobbleGoalArm wobbleArm= new WobbleGoalArm(hardwareMap);
        PIDCoefficients PID= new PIDCoefficients(0.025,0.003,0);
        boolean shootOn = false;
        Pose2d pose1 = new Pose2d(22,7);


        robot.goTo(pose1, PID,telemetry);
        spinyboi.rotate(7.5);
        robot.stop();

        shootOn = true;
        boolean launch = true;
        int i = 0;
        shooter.on2();
        while(shootOn){
            shooter.on2();
            telemetry.addData("four","i hope this is four");

            sleep(1800);

            shooter.setShooterServoLaunch();

            sleep(1000);
            shooter.setShooterServoReset();
            i++;
            if(i==4)
            {
                shootOn = false;
            }
            telemetry.update();
        }
        shooter.off();
        Pose2d now = mecam.getPoseEstimate();
        Pose2d pose2 = new Pose2d(80.5,now.getY());
        robot.goTo2(pose2,PID,telemetry);
        robot.stop();
        wobbleArm.off();
        sleep(4000);
        robot.stop();


        //spinyboi.rotate(0);
        //robot.goTo(pose2,PID,telemetry);


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
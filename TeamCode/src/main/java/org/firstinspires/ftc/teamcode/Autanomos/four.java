package org.firstinspires.ftc.teamcode.Autanomos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpDrive;
import org.firstinspires.ftc.teamcode.encoders;

import static java.lang.Thread.sleep;

public class four{

    public static void BlueOne(encoders joe, Telemetry telemetry) throws InterruptedException {
       // joe.reset();
        Pose2d distance = new Pose2d(0,0);
        com.qualcomm.robotcore.hardware.PIDCoefficients PID = new PIDCoefficients(0,0,0);
        joe.goTo(distance,PID, telemetry);

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
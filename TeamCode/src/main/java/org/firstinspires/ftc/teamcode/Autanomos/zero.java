package org.firstinspires.ftc.teamcode.Autanomos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Gyro;
import org.firstinspires.ftc.teamcode.encoders;
import org.firstinspires.ftc.teamcode.helpDrive;

import static java.lang.Thread.sleep;

public class zero {
    //TELEMETRY INTO SPINYBOI
    public static void blueOne(encoders robot, Telemetry telemetry, Gyro spinyboi) throws InterruptedException {
        PIDCoefficients PID= new PIDCoefficients(0.013,0.001,0);
        Pose2d pose1 = new Pose2d(0,10);
        Pose2d pose2 = new Pose2d(0,0);
       // spinyboi.rotate(20);

        robot.goTo(pose1, PID,telemetry);

        //robot.goTo(pose2,PID,telemetry);


    }

    public static void blueOne(helpDrive robot) throws InterruptedException {
        robot.forward(0.4, 2000);
        robot.StopDriving();
        sleep(1000);
        robot.turnRight(0.5,1750);
        robot.forward(0.5,500);
        robot.StopDriving();
        sleep(800);
    }

}

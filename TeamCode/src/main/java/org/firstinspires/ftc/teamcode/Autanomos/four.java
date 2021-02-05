package org.firstinspires.ftc.teamcode.Autanomos;

import org.firstinspires.ftc.teamcode.helpDrive;
import org.firstinspires.ftc.teamcode.encoders;

import static java.lang.Thread.sleep;

public class four{
    public static void BlueOne(encoders joe) throws InterruptedException {
       // joe.reset();
        joe.forward(1000);

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
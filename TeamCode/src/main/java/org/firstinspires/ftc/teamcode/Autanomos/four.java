package org.firstinspires.ftc.teamcode.Autanomos;

import org.firstinspires.ftc.teamcode.Teleop.helpDrive;

import static java.lang.Thread.sleep;

public class four{
    public static void BlueOne(helpDrive robot) throws InterruptedException {
        robot.forward(0.4,2900);
        robot.StopDriving();
        sleep(1500);
        robot.turnLeft(0.4);
        sleep(1750);
        robot.StopDriving();
        sleep(1500);
        robot.forward(0.4,600);
        robot.turnRight(0.4);
        sleep(1550);
        robot.backward(0.4,1060);
        robot.StopDriving();
    }

    public static void BlueTwo(helpDrive robot) throws InterruptedException {
        robot.forward(0.4,3000);
        robot.StopDriving();
        sleep(1500);
        robot.turnLeft(0.4);
        sleep(1750);
        robot.StopDriving();
        sleep(1500);
        robot.forward(0.4,600);
        robot.StopDriving();
        sleep(1750);
        robot.turnRight(0.4);
        sleep(1750);
        robot.backward(0.4,1260);
    }

    public static void RedTwo(helpDrive robot) throws InterruptedException {
        robot.forward(0.4,3100);
        robot.StopDriving();
        sleep(1500);
        robot.backward(0.4,1460);
        robot.StopDriving();
    }

    public static void RedOne(helpDrive robot) throws InterruptedException {
        robot.forward(0.4,3050);
        robot.StopDriving();
        sleep(1500);
        robot.turnRight(0.4);
        sleep(1550);
        robot.StopDriving();
        sleep(1500);
        robot.forward(0.4,600);
        robot.StopDriving();
        sleep(1500);
        robot.turnLeft(0.4);
        sleep(1300);
        robot.backward(0.4,1460);
    }




}
package org.firstinspires.ftc.teamcode.Autanomos;

import org.firstinspires.ftc.teamcode.Teleop.helpDrive;

import static java.lang.Thread.sleep;

public class four{
    public static void BlueOne(helpDrive robot) throws InterruptedException {
        robot.forward(0.2,8500);
        robot.backward(0.2,4200);
        robot.StopDriving();
    }

    public static void BlueTwo(helpDrive robot) throws InterruptedException {
        robot.forward(0.2,7500);
        robot.right(0.4);
        sleep(200);
        robot.backward(0.2,3500);
    }

    public static void RedTwo(helpDrive robot) throws InterruptedException {
        robot.forward(0.2,7500);
        robot.backward(0.2,3500);
    }

    public static void RedOne(helpDrive robot) throws InterruptedException {
        robot.forward(0.2,7500);
        robot.left(0.4);
        sleep(200);
        robot.backward(0.2,3500);
    }




}
package org.firstinspires.ftc.teamcode.Autanomos;

import org.firstinspires.ftc.teamcode.Teleop.helpDrive;

import static java.lang.Thread.sleep;

public class one {
    public static void blueOne(encoders robot) throws InterruptedException {
        robot.forward(10);
        robot.backward(500);
    }
    /*
    public static void blueOne(helpDrive robot) throws InterruptedException {
        robot.forward(0.4,2520);
        robot.StopDriving();
        sleep(500);
        robot.backward(0.4,500);
        robot.StopDriving();
    }

    public static void bluetwo(helpDrive robot) throws InterruptedException {
        robot.forward(0.2,6000);
        //robot.left(0.4);
        sleep(200);
    }

    public static void redTwo(helpDrive robot) throws InterruptedException {
        robot.forward(0.2,6000);
        robot.right(0.4);
        sleep(200);
    }

    public static void redOne(helpDrive robot) throws InterruptedException {
        robot.forward(0.2,6000);
        //robot.left(0.4);
        sleep(200);
    }*/
}

package org.firstinspires.ftc.teamcode.Autanomos;

import org.firstinspires.ftc.teamcode.Teleop.helpDrive;

import static java.lang.Thread.sleep;
public class zero {
    public static void blueOne(helpDrive robot) throws InterruptedException {
        robot.forward(0.5, 500);
    }

    public static void bluetwo(helpDrive robot) throws InterruptedException {
        robot.forward(0.5, 500);
        robot.right(0.5);
        sleep(500);
    }

    public static void redOne(helpDrive robot) throws InterruptedException {
        robot.forward(0.5, 500);
        robot.right(0.5);
        sleep(500);
    }

    public static void redtwo(helpDrive robot) throws InterruptedException {
       robot.forward(0.5, 500);
    }
}

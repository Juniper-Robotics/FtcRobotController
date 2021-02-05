package org.firstinspires.ftc.teamcode.Autanomos;

import org.firstinspires.ftc.teamcode.helpDrive;
import org.firstinspires.ftc.teamcode.encoders;

import static java.lang.Thread.sleep;

public class one {
    public static void blueOne(encoders robot) throws InterruptedException {
        robot.forward(10);
        robot.backward(500);
    }

    public static void blueOne(helpDrive robot) throws InterruptedException {
        robot.forward(0.4,2720);
        robot.StopDriving();
        sleep(500);
        robot.backward(0.4,500);
        robot.StopDriving();
    }

}

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

public class zero {
    //TELEMETRY INTO SPINYBOI


    public static void blueOne(encoders robot, Telemetry telemetry, Gyro spinyboi, HardwareMap hardwareMap, myMecnam mecam) throws InterruptedException {
        telemetry.addData("none","hopefully no rings or one ring");
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
            telemetry.addData("none","hopefully no rings or one ring");
            //telemetry.update();
            shooter.on2();
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
        Pose2d pose2 = new Pose2d(52,now.getY());
        robot.goTo2(pose2,PID,telemetry);
        robot.stop();
        wobbleArm.off();
        sleep(4000);
        robot.stop();

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

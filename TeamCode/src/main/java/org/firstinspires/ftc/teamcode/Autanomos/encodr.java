package org.firstinspires.ftc.teamcode.Autanomos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.encoders;
import org.firstinspires.ftc.teamcode.myMecnam;

@Autonomous
public class encodr extends LinearOpMode {
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private BNO055IMU imu;



    public void runOpMode() throws InterruptedException {
        rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();//new parameters opbejct
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;//sertting parameter to degrees
        imu = hardwareMap.get(BNO055IMU.class,"imu");//getting from hardware map
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu.initialize(parameters);

        myMecnam odo = new myMecnam(hardwareMap,0.00055,0,0, 13.6193231,13.250,1, telemetry);
        //updates x, y , roation
        //write loop later

        PIDCoefficients PID= new PIDCoefficients(0.03,0.005,0);
        encoders bob = new encoders(leftBackMotor, rightBackMotor, leftFrontMotor,rightFrontMotor, imu, odo);

        waitForStart();

        Pose2d pose1 = new Pose2d(-10,0);
        bob.goTo(pose1, PID,telemetry);
        bob.stop();
       /* Shooter shooter = new Shooter(hardwareMap, telemetry);
        WobbleGoalArm wobbleArm= new WobbleGoalArm(hardwareMap);
        PIDCoefficients PID= new PIDCoefficients(0.0252,0.003,0);
        boolean shootOn = true;
        boolean launch = true;
        waitForStart();

        shooter.setShooterServoLaunch();
        int i = 1;
        while(shootOn){
            shooter.on();
            sleep(1500);

            shooter.setShooterServoLaunch();

            sleep(1000);
            shooter.setShooterServoReset();
            i++;
            if(i==3)
            {
                shootOn = false;
            }


        }
        telemetry.addData("i",0);
        wobbleArm.off();
        telemetry.addData("e",0);
        sleep(4000);*/
    }
}
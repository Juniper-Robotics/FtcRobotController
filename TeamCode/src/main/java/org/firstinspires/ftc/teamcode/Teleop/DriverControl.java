package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Collector;
import org.firstinspires.ftc.teamcode.Gyro;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.WobbleGoalArm;
import org.firstinspires.ftc.teamcode.helpDrive;
import org.firstinspires.ftc.teamcode.myMecnam;


@TeleOp
public class DriverControl extends LinearOpMode {
    //this is initializing ex)int number = DcMotor Motor
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;


    private BNO055IMU imu;
    private Orientation angles;


    //frontLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    @Override
    public void runOpMode() throws InterruptedException {
        //sets Motor equal to "motorOne" which is announce on the robot
        rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        boolean collectorOn = false;
        boolean shooterOn = false;
        boolean wobbleOn = false;
        Shooter shooter = new Shooter(hardwareMap);
        WobbleGoalArm wobbleArm= new WobbleGoalArm(hardwareMap);
        Collector collector = new Collector((hardwareMap));
        helpDrive carl = new helpDrive(leftBackMotor, rightBackMotor, leftFrontMotor, rightFrontMotor, imu);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();//new parameters opbejct
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;//sertting parameter to degrees
        imu = hardwareMap.get(BNO055IMU.class, "imu");//getting from hardware map
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu.initialize(parameters);
        myMecnam mecam = new myMecnam(hardwareMap, 0, 0, 0, 13.6193231, 13.250, 1);
        Pose2d now = mecam.getPoseEstimate(); // position
       // List<Double> now; //for wheel position
        Gyro spinyboi = new Gyro(imu, angles, 0.0035, 0.0005, 0, carl);

        shooter.off();
        wobbleArm.off();



        waitForStart();

        while (opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            now = mecam.getPoseEstimate();
           // now = mecam.getWheelPositions();
            telemetry.addData("angle: ", angles.firstAngle);
            telemetry.addData("x spot: ", now.getX());
           telemetry.addData("y spot: ", now.getY());
           telemetry.addData("shooter speed", shooter.returnSpeed());
           //telemetry.addData("servo", wobbleArm.returnPos());
            //telemetry.addData("pos",now);
            telemetry.update();

            double speed = 0.8;

            if (gamepad1.dpad_left) {
                carl.left(speed);
            } else if (gamepad1.dpad_down) {
                carl.forward(speed);
            } else if (gamepad1.dpad_right) {
                carl.right(speed);
            } else if (gamepad1.dpad_up) {
                carl.backward(speed);
            } else if (gamepad1.right_bumper) {
                carl.turnRight(speed);
            } else if (gamepad1.left_bumper) {
                carl.turnLeft(speed);
            } else if (gamepad1.left_trigger != 0) {
                carl.backward((0.5));
            } else if (gamepad1.right_trigger != 0) {
                carl.forward((0.5));
            } else

             //shooter stuff
                if (gamepad2.right_trigger != 0) {
                    shooterOn = true;
                    shooter.reset();
                } else if (gamepad2.right_trigger == 0) {
                    shooterOn = false;
                }
            if (shooterOn) {
                shooter.on();
            } else {
                shooter.off();
            }

            //collecter stuff
            if (gamepad2.left_trigger != 0) {
                collectorOn = true;
            } else if (gamepad2.right_trigger == 0) {
                collectorOn = false;
            }
            if (collectorOn) {
                collector.on();
            } else {
                collector.off();
            }

            //arm stuff
            if (gamepad2.b) {
                wobbleOn = true;
                shooter.reset();
            } else if (gamepad2.a) {
                wobbleOn = false;
            }
            if (wobbleOn) {
                wobbleArm.on();
            } else {
                wobbleArm.off();
            }


            if (gamepad2.right_bumper) {
                shooter.setShooterServoLaunch();
            } else {
                shooter.setShooterServoReset();
            }


            if (gamepad1.a) {
                spinyboi.rotate(0);
            } else {
                rightBackMotor.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / 2);
                rightFrontMotor.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / 2);
                leftBackMotor.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) / 2);
                leftFrontMotor.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / 2);
            }
        }
    }
}




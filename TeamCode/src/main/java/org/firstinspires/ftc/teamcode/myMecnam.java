package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class myMecnam extends MecanumDrive {
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private List<DcMotor> motors;

    public static PIDCoefficients forward;
    public static PIDCoefficients sidewards;
    public static PIDCoefficients turn;

    private Pose2d whereNow;
    private Pose2d target;
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    private Orientation angles;

    public helpDrive gerlad;
    public Gyro spinyBoi;
    public encoders sam;
    Telemetry telemetry;

    //constructor
    public myMecnam(HardwareMap hardwareMap, double kv, double kA, double kStatic, double trackWidth, double wheelBase, double lateralMultiplier, Telemetry tele) {
        super(kv,kA,kStatic, trackWidth, wheelBase,lateralMultiplier);
        rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");

        forward = new PIDCoefficients(0.02,0.003,0);
        sidewards = new PIDCoefficients(0,0,0);
        turn = new PIDCoefficients(0.01, 0.001, 0.000); //turn

        motors = Arrays.asList(leftFrontMotor, leftBackMotor, rightBackMotor, rightFrontMotor);
        imu = hardwareMap.get(BNO055IMU.class,"imu");//getting from hardware map
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        telemetry = tele;

        gerlad = new helpDrive(leftBackMotor, rightBackMotor,  leftFrontMotor,  rightFrontMotor, imu);


        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        spinyBoi = new Gyro(imu,angles,0.008,0.001,0.0,gerlad, telemetry);

    }

    public double ticksToInches(int x){
        //cirmunferance 11.87374866142in
        //28 per rotation
        //0.0230068359

        return x*(0.0213463442);
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> positions = new ArrayList();
        //for(int i = 0; i<motors.size(); i++){
            positions.add(ticksToInches(motors.get(0).getCurrentPosition()));
        positions.add(ticksToInches(motors.get(1).getCurrentPosition()));
        positions.add(ticksToInches(motors.get(2).getCurrentPosition()));
        positions.add(ticksToInches(motors.get(3).getCurrentPosition()));
       // }
        return positions;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        gerlad.setPowers(v2,v3,-v1,-v);
    }

    @Override
    protected double getRawExternalHeading() {

        return Math.toRadians(imu.getAngularOrientation().firstAngle);
    }

    public void goThere(Pose2d there){
        updatePoseEstimate();


    }


}

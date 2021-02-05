package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

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
    public static PIDCoefficients notforward;

    private Pose2d whereNow;
    private Pose2d target;
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    private Orientation angles;

    public helpDrive gerlad;
    public Gyro spinyBoi;
    public encoders sam;

    //constructor
    public myMecnam(HardwareMap hardwareMap, double kv, double kA, double kStatic, double trackWidth, double wheelBase, double lateralMultiplier ) {
        super(kv,kA,kStatic, trackWidth, wheelBase,lateralMultiplier);
        rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");

        forward = new PIDCoefficients(0,0,0);
        sidewards = new PIDCoefficients(0,0,0);
        notforward = new PIDCoefficients(0,0,0);

        motors = Arrays.asList(leftFrontMotor, leftBackMotor, rightBackMotor, rightFrontMotor);
        imu = hardwareMap.get(BNO055IMU.class,"imu");//getting from hardware map
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        gerlad = new helpDrive(leftBackMotor, rightBackMotor,  leftFrontMotor,  rightFrontMotor, imu);


        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        spinyBoi = new Gyro(imu,angles,gerlad,notforward);

    }

    public double ticksToInches(int x){
        //do this stuf later
        //29 per rotation
        //38.4845 cm circumferance aka 15.151378 in
        //5.49778571 per tick
        return x*(0.0230068359);
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

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    DcMotor shooterMotor;
    Servo shooterServo;

    //Tune this stuff
    //0.00022,0,0
    final com.acmerobotics.roadrunner.control.PIDCoefficients shooterPidCoeff = new PIDCoefficients(0.00022,0,0.000);
    PIDFController shooterPID = new PIDFController(shooterPidCoeff);
    double shooterSpeed = 0;

    int lastPos = 0;
    ElapsedTime elapsedTime;
    double lastTime = 0;
    //figure this stuff out
    final double TICKS_PER_ROTATION = 28;
    final double TARGET_SPEED = 8000; //Update later
    final double SHOOTER_SERVO_LAUNCH_POSITION = 0;
    final double SHOOT_SERVO_RESET_POSITION = 0.8;
    Telemetry telemetry;

    public Shooter(HardwareMap hardwareMap,Telemetry telemetry) {
        //update name
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");

        elapsedTime = new ElapsedTime();
        shooterPID.setOutputBounds(-0.8, 0.8);
        this.telemetry = telemetry;
        //reverse motor

    }

    public void resetMotor(){
        shooterMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
    }


    //THIS IS WHERE YOU EDIT ARM
    public void on(){
        //commant out this stuff
       updateShooterSpeed();
        telemetry.addData("shooterspeed", shooterPID.update(shooterSpeed));
       shooterMotor.setPower(shooterPID.update(shooterSpeed));
        telemetry.addData("currentPos", shooterMotor.getCurrentPosition());
        //uncommint out this
        // |
        // V
       // shooterMotor.setPower(0.8);

    }

    public void updateShooterSpeed(){
        int currentEncoderPos = shooterMotor.getCurrentPosition();
        int deltaEncPos = currentEncoderPos - lastPos;
       // telemetry.addData("encPos", deltaEncPos);
        //was minutes but no method for that
        double currentTime = elapsedTime.seconds()/60.0;
        double deltaTime = currentTime - lastTime;
       // telemetry.addData("deltaTime", deltaTime);

        double deltaRot = deltaEncPos * (1/TICKS_PER_ROTATION);
       // telemetry.addData("deltaRot", deltaRot);
        shooterSpeed = deltaRot/deltaTime;

        lastTime = currentTime;
        lastPos = currentEncoderPos;


       /* int currentEncoderPos = shooterMotor.getCurrentPosition();
        int deltaEncPos = currentEncoderPos - lastPos;
        telemetry.addData("encPos", deltaEncPos);
        //was minutes but no method for that
        double currentTime = elapsedTime.seconds()/60.0;
        double deltaTime = currentTime - lastTime;
        telemetry.addData("deltaTime", deltaTime);

        double deltaRot = deltaEncPos * (1/TICKS_PER_ROTATION);
        telemetry.addData("deltaRot", deltaRot);
        shooterSpeed = deltaRot/deltaTime;

        lastTime = currentTime;
        lastPos = currentEncoderPos;*/
    }


    public void reset(){
        shooterMotor.setPower(0);
        shooterPID.reset();
        shooterPID.setTargetPosition(TARGET_SPEED);
    }

    public void off(){
        shooterMotor.setPower(0);
    }

    public void setShooterServoLaunch(){
        shooterServo.setPosition(SHOOTER_SERVO_LAUNCH_POSITION);
    }

    public void setShooterServoReset(){
        shooterServo.setPosition(SHOOT_SERVO_RESET_POSITION);
    }
}

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {
    DcMotor shooterMotor;
    Servo shooterServo;

    //Tune this stuff
    final com.acmerobotics.roadrunner.control.PIDCoefficients shooterPidCoeff = new PIDCoefficients(100,0,0);
    PIDFController shooterPID = new PIDFController(shooterPidCoeff);
    double shooterSpeed = 0;

    int lastPos = 0;
    ElapsedTime elapsedTime;
    double lastTime;
    //figure this stuff out
    final double TICKS_PER_ROTATION = 103.6;
    final double TARGET_SPEED = 3000; //Update later
    final double SHOOTER_SERVO_LAUNCH_POSITION = 0;
    final double SHOOT_SERVO_RESET_POSITION = 0.8;

    public Shooter(HardwareMap hardwareMap) {
        //update name
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");

        elapsedTime = new ElapsedTime();
        shooterPID.setOutputBounds(-1, 1);
        //reverse motor

    }

    public void on(){
        updateShooterSpeed();
        shooterMotor.setPower(shooterPID.update(shooterSpeed));
       // shooterMotor.setPower(0.7);
    }

    public double returnSpeed(){
        return shooterSpeed;
        //return shooterMotor.getCurrentPosition();
    }

    public double returnspeed(){
        return shooterMotor.getPower();
        //return shooterMotor.getCurrentPosition();
    }
    public void updateShooterSpeed(){
        int currentEncoderPos = shooterMotor.getCurrentPosition();
        int deltaEncPos = currentEncoderPos - lastPos;
        //was minutes but no method for that
        double currentTime = elapsedTime.seconds()/60;
        double deltaTime = currentTime - lastTime;

        double deltaRot = deltaEncPos * (1/TICKS_PER_ROTATION);

        shooterSpeed = deltaRot/deltaTime;

        lastTime = currentTime;
        lastPos = currentEncoderPos;
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

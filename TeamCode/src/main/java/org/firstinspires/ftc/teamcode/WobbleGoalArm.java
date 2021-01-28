package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleGoalArm {

    private Servo wobbleServo;

    public WobbleGoalArm(HardwareMap hardwareMap){
        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");
    }
    public double returnPos(){
        return wobbleServo.getPosition();
    }
    public void on(){ wobbleServo.setPosition(0.8); }

    public void off(){ wobbleServo.setPosition(0); }

}

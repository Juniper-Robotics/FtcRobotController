package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Collector {
    DcMotor collectorMotor;
    public Collector(HardwareMap hardwareMap){
        collectorMotor = hardwareMap.get(DcMotor.class, "collectorMotor");
    }

    public void on(double power){
        collectorMotor.setPower(-power);
    }

    public double returnSpeed(){
        //return shooterMotor.getPower();
        return collectorMotor.getCurrentPosition();
    }

    public void off(){
        collectorMotor.setPower(0);
    }
}



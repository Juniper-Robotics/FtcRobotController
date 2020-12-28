package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@TeleOp
public class DriverControl extends LinearOpMode
{
    //this is initializing ex)int number = DcMotor Motor
   private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;

    //frontLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    @Override
    public void runOpMode() throws InterruptedException {
        //sets Motor equal to "motorOne" which is announce on the robot
        rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");
        helpDrive robot = new helpDrive(leftBackMotor, rightBackMotor, leftFrontMotor,rightFrontMotor);

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            double speed = 0.8;
            if(gamepad1.dpad_left){
                robot.left(speed);
            }else if(gamepad1.dpad_down){
                robot.backward(speed);
            }else if(gamepad1.dpad_right){
                robot.right(speed);
            }else if(gamepad1.dpad_up){
                robot.forward(speed);
            }else if(gamepad1.right_bumper){
                robot.turnRight(speed);
            }else if(gamepad1.left_bumper){
                robot.turnLeft(speed);
            }else if(gamepad1.right_stick_x!=0){
                robot.setPower(gamepad1.right_stick_x,gamepad1.right_stick_x, -gamepad1.right_stick_x,-gamepad1.right_stick_x);
            }else{
                robot.setPower(0,0,0,0);
            }

        }
    }

    }
    /*rightBackMotor.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
            rightFrontMotor.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x +gamepad1.right_stick_x);
            leftBackMotor.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x-gamepad1.right_stick_x);
            leftFrontMotor.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x-gamepad1.right_stick_x);
*/


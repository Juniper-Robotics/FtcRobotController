package org.firstinspires.ftc.teamcode.Autanomos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.helpDrive;
import org.firstinspires.ftc.teamcode.encoders;
import org.firstinspires.ftc.teamcode.myMecnam;

import java.util.List;

@Autonomous(name = "tensor flow based")
public class Autanomos extends LinearOpMode{

    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private BNO055IMU imu;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "AS8A2vz/////AAABmYwqbCq8YkRYghpp509jV+1mOOEmiEeadQu6kpFf1/aK/SlWZ09AqyNjqEWmPsKY6Q+GMdhAkey9UmFn9Xf8+I6OK2ECl8RIY7DSygObRwR2Cm4L/h0WVkFTXsQGSUr5BMaNAnmzr6awVlRE5EcWc3RgOntqJiVSaIl8OpftG81O4zb5aYxMprNjo57/MKv7AMhWxz3Qr7u4IBeZ6Auda1QTt2Ti1MmzidFj+jmAg/ZBiZV1HgdR5ArsYJcoQRhIWGSLAzNrKTsSpRBqZ1KFy4m/TvvCu0GyLLIGMFhv0edz5VRbshiQBbcm8Xsst236I147dg9d5IsRWwFeAbvIn0WKtNxpq8hE69qnq2H3ndUO";

    private TFObjectDetector tfod;
    private VuforiaLocalizer Vuforia;


    @Override
    public void runOpMode() throws InterruptedException {

        rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        int placement = 0;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();//new parameters opbejct
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;//sertting parameter to degrees
        imu = hardwareMap.get(BNO055IMU.class,"imu");//getting from hardware map
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu.initialize(parameters);

        myMecnam odo = new myMecnam(hardwareMap,0,0,0, 13.6193231,13.250,1);
        odo.updatePoseEstimate();
        Pose2d currentPose = odo.getPoseEstimate();
        //updates x, y , roation
        //write loop later

        encoders bob = new encoders(leftBackMotor, rightBackMotor, leftFrontMotor,rightFrontMotor, imu, odo);
        helpDrive carl = new helpDrive(leftBackMotor, rightBackMotor, leftFrontMotor,rightFrontMotor, imu);

       initVuforia();
        initTfod();

        waitForStart();



            if (tfod != null) {
                tfod.activate();

                // The TensorFlow software will scale the input images from the camera to a lower resolution.
                // This can result in lower detection accuracy at longer distances (> 55cm or 22").
                // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
                // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
                // should be set to the value of the images used to create the TensorFlow Object Detection model
                // (typically 16/9).
                tfod.setZoom(2, 16.0 / 9.0);

                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    telemetry.update();
                    if (updatedRecognitions.size() == 0) {
                        // empty list.  no objects recognized.
                        telemetry.addData("TFOD", "No items detected.");
                        telemetry.addData("Target Zone", "A");
                        placement = 0;
                        telemetry.update();
                    } else {
                        // list is not empty.
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            telemetry.update();
                            // check label to see which target zone to go after.
                            if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                                telemetry.addData("Target Zone", "B");
                                telemetry.update();
                                placement = 2;
                            } else if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                                telemetry.addData("Target Zone", "C");
                                telemetry.update();
                                placement = 4;
                            } else {
                                telemetry.addData("Target Zone", "UNKNOWN");
                                telemetry.update();

                            }
                        }
                    }
                    telemetry.update();
                }

            }

         if(tfod != null){
            tfod.shutdown();
         }


        switch(placement){
            case 4:
                four.BlueOne(carl);
                break;
            case 1:
                one.blueOne(carl);
                break;
            case 0:
                zero.blueOne(carl);
                break;
        }

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters;
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        Vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
   private void initTfod() {
        //VuforiaLocalizer.Parameters parameters;
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, Vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/

    }
}


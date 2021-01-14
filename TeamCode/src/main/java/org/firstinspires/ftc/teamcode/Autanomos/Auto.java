package org.firstinspires.ftc.teamcode.Autanomos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

//import static org.firstinspires.ftc.teamcode.EasyOpenCV.SkystoneDeterminationPipeline.RingPosition.FOUR;
//import static org.firstinspires.ftc.teamcode.EasyOpenCV.SkystoneDeterminationPipeline.RingPosition.NONE;
//import static org.firstinspires.ftc.teamcode.EasyOpenCV.SkystoneDeterminationPipeline.RingPosition.ONE;

//@Autonomous
public class Auto extends LinearOpMode{
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;
  //  EasyOpenCV.SkystoneDeterminationPipeline.RingPosition yeh;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //pipeline = new EasyOpenCV.SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        encoders bob = new encoders(leftBackMotor, rightBackMotor, leftFrontMotor,rightFrontMotor);

        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                sleep(2000);
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        waitForStart();

            sleep(2000);
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();

            sleep(100);//calibrate the thingy

        //50 in per 10000

           switch(pipeline.position){
                case FOUR:
                    four.BlueOne(bob);
                    break;
                case ONE:
                    one.blueOne(bob);
                    break;
                case NONE:
                    zero.blueOne(bob);
                    break;
            }


    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {

        /*
         * An enum to define the skystone position
         */
       public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(180,100);

        static final int REGION_WIDTH = 100;
        static final int REGION_HEIGHT = 100;

        final int FOUR_RING_THRESHOLD = 135;
        final int ONE_RING_THRESHOLD = 129;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
       Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile SkystoneDeterminationPipeline.RingPosition position;//= FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            //position = FOUR; // Record our analysis
            if(avg1 >= FOUR_RING_THRESHOLD){
                position =  SkystoneDeterminationPipeline.RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = SkystoneDeterminationPipeline.RingPosition.ONE;

            }else{
                position = SkystoneDeterminationPipeline.RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }

    }
   }


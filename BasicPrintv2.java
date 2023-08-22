package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.*;

import org.firstinspires.ftc.teamcode.PID;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.MatOfPoint2f;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name="BasicPrintv2", group="First")
public class BasicPrintv2 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftBack = null;
    private DcMotorEx rightBack = null;

    private DcMotorEx leftFront = null;
    private  DcMotorEx rightFront = null;
    static OpenCvCamera phoneCam;
    private int MAX_TPS = 1800;

    public void runOpMode(){
        telemetry.setAutoClear(false);

        //Telemetry Initialization
        Telemetry.Item leftEncode = telemetry.addData("Left Encoder:", 0);
        Telemetry.Item rightEncode = telemetry.addData("Right Encoder:", 0);
        Telemetry.Item leftEncodef = telemetry.addData("Left Encoder (Front):", 0);
        Telemetry.Item rightEncodef = telemetry.addData("Right Encoder (Front):", 0);

        Telemetry.Item timeVal = telemetry.addData("Elapsed Time", runtime);

        Telemetry.Item lPowerText = telemetry.addData("lPower", 0.0);
        Telemetry.Item rPowerText = telemetry.addData("rPower", 0.0);

        Telemetry.Item SPDirection = telemetry.addData("SPDirection", 0.0);
        Telemetry.Item SPDirectionMag = telemetry.addData("SPDirectionMag", 0.0);

        Telemetry.Item lTargetText = telemetry.addData("lTarget", 0.0);
        Telemetry.Item rTargetText = telemetry.addData("rTarget", 0.0);
        Telemetry.Item lyText = telemetry.addData("ly", 0.0);

        //Motor Initialization
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBackWheel");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBackWheel");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFrontWheel");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFrontWheel");

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);

        //PID Initialization
        PID left_PID = new PID(leftBack.getVelocity(), 0.0, runtime);
        PID right_PID = new PID(rightBack.getVelocity(), 0.0, runtime);
        PID right_PID_f = new PID(rightFront.getVelocity(), 0.0, runtime);
        PID left_PID_f = new PID(leftFront.getVelocity(), 0.0, runtime);
        TurnPID left_turn_PID = new TurnPID(SamplePipeline.directionMag, SamplePipeline.direction, "left");
        TurnPID right_turn_PID = new TurnPID(SamplePipeline.directionMag, SamplePipeline.direction, "right");


        //Camera Initialization
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(
                OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.setPipeline(new BasicPrintv2.SamplePipeline());
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the camera to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Also, we specify the rotation that the camera is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()){

            // Get Vars for Velocity
            double lVelb = leftBack.getVelocity();
            double rVelb = rightBack.getVelocity();
            double lVelf = leftFront.getVelocity();
            double rVelf = rightFront.getVelocity();

            // Get Inputs
            double ly = -gamepad1.left_stick_y;
            double lx = gamepad1.left_stick_x;

            // Set Targets for wheels
            double steering = lx;
            if (ly < 0) {
                steering *= -1; // Reverse steering when moving backwards
            }

            double lTarget, rTarget;
            lTarget = ly + steering;
            rTarget = ly - steering;

            double maxSpeed = Math.max(Math.abs(lTarget), Math.abs(rTarget));
            if (maxSpeed > 1.0) {
                lTarget /= maxSpeed;
                rTarget /= maxSpeed;
            }


            switch (SamplePipeline.direction) {
                case "straight":
                    lTarget = 0.4;
                    rTarget = 0.4;
                    break;
                case "left": case "right":
                    lTarget = left_turn_PID.updatePID(SamplePipeline.directionMag, SamplePipeline.direction);
                    rTarget = right_turn_PID.updatePID(SamplePipeline.directionMag, SamplePipeline.direction);
                    break;
                case "stop": default:
                    break;
            }

            lTarget *= MAX_TPS;
            rTarget *= MAX_TPS;
            // Calculate and Set power levels
            double lPower = left_PID.updatePID(lVelb, lTarget, runtime, leftBack.getPower());
            double rPower = right_PID.updatePID(rVelb, rTarget, runtime, rightBack.getPower());
            double lPowerf = left_PID.updatePID(lVelf, lTarget, runtime, leftFront.getPower());
            double rPowerf = right_PID.updatePID(rVelf, rTarget, runtime, rightFront.getPower());

            leftBack.setPower(lPower);
            rightBack.setPower(rPower);
            leftFront.setPower(lPowerf);
            rightFront.setPower(rPowerf);

            // Set Telemetry Vals
            lyText.setValue(ly);
            leftEncode.setValue(lVelb);
            rightEncode.setValue(rVelb);
            leftEncodef.setValue(lVelf);
            rightEncodef.setValue(rVelf);
            timeVal.setValue(getRuntime());
            lPowerText.setValue(lPower);
            rPowerText.setValue(rPower);
            SPDirection.setValue(SamplePipeline.direction);
            SPDirectionMag.setValue(SamplePipeline.directionMag);
            lTargetText.setValue(lTarget);
            rTargetText.setValue(rTarget);

            telemetry.update();
        }
    }

    private static class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused = false;
        Mat hsv;
        Mat mask;
        Mat maskedImg;
        Mat heirarchy;

        public static String direction = "stop";
        public static double directionMag = 0;

        private final double threshold = 5;

        @Override
        public void init(Mat firstFrame)
        {
            hsv = new Mat();
            mask = new Mat();
            maskedImg = new Mat();
            heirarchy = new Mat();
        }

        @Override
        public Mat processFrame(Mat input)
        {
            hsv.setTo(new Scalar(0, 0, 0));
            mask.setTo(new Scalar(0, 0, 0));
            maskedImg.setTo(new Scalar(0, 0, 0));
            heirarchy.setTo(new Scalar(0, 0, 0));

            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Scalar lowerBlue = new Scalar(25, 100, 100);
            Scalar upperBlue = new Scalar(50, 255, 255);

            Core.inRange(hsv, lowerBlue, upperBlue, mask);

            Core.bitwise_and(input, input, maskedImg, mask);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, heirarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            double area = 0;
            double circleCenterx = 0;
            for (MatOfPoint contour : contours) {
                area = Imgproc.contourArea(contour);
                double perimeter = Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true);
                if (perimeter == 0) {
                    continue;
                }
                double circularity = 4 * Math.PI * area / (perimeter * perimeter);

                // Adjust the threshold for circularity to filter the circular contour
                if (0.1 < circularity && circularity < 1.5 && area > 100) {
                    Moments moments = Imgproc.moments(contour);
                    double center_x = moments.get_m10() / moments.get_m00();
                    double center_y = moments.get_m01() / moments.get_m00();

                    Point circleCenter = new Point(center_x, center_y);
                    circleCenterx = center_x;
                    // Draw a circle to highlight the centerpoint
                    Imgproc.circle(input, circleCenter, 5, new Scalar(0, 0, 255), -1);
                }
            }

            double linex = input.cols()/2f;
            if (circleCenterx != 0){
                if(Math.abs(linex - circleCenterx) <= threshold){
                    if (area > 2000){
                        direction = "stop";
                        directionMag = 0;
                    }
                    else{
                        direction = "straight";
                        directionMag = 2000-area;
                    }
                } else if (circleCenterx > linex) {
                    direction = "right";
                    directionMag = Math.abs(linex - circleCenterx);
                } else if (circleCenterx < linex) {
                    direction = "left";
                    directionMag = Math.abs(linex - circleCenterx);
                }
            }
            if (area == 0){
                direction = "stop";
                directionMag = 0;
            }

            Imgproc.line(input,
                    new Point(input.cols()/2f, 0),
                    new Point(input.cols()/2f, input.rows()),
                    new Scalar(0, 0, 255),
                    1
            );

            Imgproc.putText(input,
                    direction,
                    new Point(25, 25),
                    Imgproc.FONT_HERSHEY_COMPLEX,
                    1,
                    new Scalar(255, 255, 255)
            );

            Imgproc.putText(input,
                    String.valueOf(directionMag),
                    new Point(25, 50),
                    Imgproc.FONT_HERSHEY_COMPLEX,
                    1,
                    new Scalar(255, 255, 255)
            );

            return input;
        }

        @Override
        public void onViewportTapped(){
            viewportPaused = !viewportPaused;
            if (viewportPaused){
                phoneCam.pauseViewport();
            } else{
                phoneCam.resumeViewport();
            }
        }

    }
}

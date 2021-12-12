package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.assembly.BobTheDuckBot;
import org.opencv.core.Mat;

import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;

import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name = "TestDetection", group = "League")
public class TestTeamElementWebcam_NEW extends LinearOpMode
{
    OpenCvCamera webCam;
    TeamElementDeterminationPipeline pipeline;
    BobTheDuckBot frenzyBot = new BobTheDuckBot();
    static boolean firstTime = true;

    @Override
    public void runOpMode()
    {

        frenzyBot.initRobot(hardwareMap);

        int cameraMonitorViewId = frenzyBot.getRobotHardware().getHwMap().appContext.getResources().getIdentifier("cameraMonitorViewId", "id", frenzyBot.getRobotHardware().getHwMap().appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(frenzyBot.getRobotHardware().getHwMap().get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new TeamElementDeterminationPipeline(telemetry);
        webCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
 //       webCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webCam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("currentLocation", pipeline.getFrameLocation());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(5);
        }
    }

    public static class TeamElementDeterminationPipeline extends OpenCvPipeline
    {

        static Telemetry telemetry;

        public TeamElementDeterminationPipeline(Telemetry tele) {
            telemetry = tele;
        }

        String location;
/*
        @Override
        public  Mat processFrame(Mat input)
        {


                double[] pixel;
                Size size = input.size();
                double height = size.height;
                double width = size.width;
                double redValue = 0;
                double greenValue = 0;
                double blueValue = 0;

                int leftCount = 0;
                int middleCount = 0;
                for (int i = 0; i < width /4; i++) {
                    for (int j = 0; j < height; j++) {
                        pixel = input.get(i, j);
                        if (pixel != null && pixel.length > 0) {
                            redValue = pixel[0];
                            greenValue = pixel[1];
                            blueValue= pixel[2];
                            if ((redValue > 65 && redValue< 80)
                                    && (greenValue > 60 && greenValue < 255)
                                    && (blueValue > 60 && blueValue < 255)) {
                                leftCount++;

                            }

                        } else {
                            //telemetry.addData("Pixel Null at",  i );
                            //telemetry.addData(" ",  j );
                            //telemetry.update();
                        }
                    }
                }
            for (int i = (int)(3* width/4); i < width; i++) {
                for (int j = 0; j < height; j++) {
                    pixel = input.get(i, j);
                    telemetry.addData("if second for loop i",  i );
                    telemetry.addData("second loop j",  j );
                    telemetry.update();
                    if (pixel != null && pixel.length > 0) {
                        redValue = pixel[0];
                        greenValue = pixel[1];
                        blueValue= pixel[2];
                        if ((redValue > 65 && redValue< 80)
                                && (greenValue > 60 && greenValue < 255)
                                && (blueValue > 60 && blueValue < 255)) {
                            middleCount++;

                        }

                    } else {
                        //telemetry.addData("Pixel Null at",  i );
                        //telemetry.addData(" ",  j );
                        //telemetry.update();
                    }
                }
            }
                if (leftCount > 1000) {
                    location = "left";
                } else if (middleCount > 1000) {
                    location = "middle";
                } else {
                    location = "right";
                }
                telemetry.addData("leftCount", leftCount);
                telemetry.addData("middleCount", middleCount);

                telemetry.addData("width", width);
                telemetry.addData("height", height);
                telemetry.update();


                return input;

        }

*/

@Override
public Mat processFrame(Mat input) {
    // "Mat" stands for matrix, which is basically the image that the detector will process
    // the input matrix is the image coming from the camera
    // the function will return a matrix to be drawn on your phone's screen

    // The detector detects team element The camera fits area for two positions.
    // If it finds team element in either of these locations, it can identify whether left or middle.
    // If not team element is on right

    // Make a working copy of the input matrix in HSV
    Mat mat = new Mat();
    Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

    // if something is wrong, we assume there's no team element
    if (mat.empty()) {
        location = "RIGHT";
        return input;
    }

    // We create a HSV range for yellow to detect regular stones
    // NOTE: In OpenCV's implementation,
    // Hue values are half the real value
    //Scalar lowHSV = new Scalar(20, 100, 100); // lower bound HSV for yellow
    //Scalar highHSV = new Scalar(30, 255, 255); // higher bound HSV for yellow

    Scalar lowHSV = new Scalar(65, 60, 60); // lower bound HSV for green
    Scalar highHSV = new Scalar(80, 255, 255); // higher bound HSV for green

    Mat thresh = new Mat();

    // We'll get a black and white image. The white regions represent the regular stones.
    // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
    Core.inRange(mat, lowHSV, highHSV, thresh);

    // Use Canny Edge Detection to find edges
    // you might have to tune the thresholds for hysteresis
    Mat edges = new Mat();
    Imgproc.Canny(thresh, edges, 100, 300);

    // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
    // Oftentimes the edges are disconnected. findContours connects these edges.
    // We then find the bounding rectangles of those contours
    List<MatOfPoint> contours = new ArrayList<>();
    Mat hierarchy = new Mat();
    Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

    MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
    Rect[] boundRect = new Rect[contours.size()];
    for (int i = 0; i < contours.size(); i++) {
        contoursPoly[i] = new MatOfPoint2f();
        Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
        boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
    }

    Size size = input.size();
    double height = size.height;
    double width = size.width;
    // Iterate and check whether the bounding boxes
    // cover left and/or right side of the image
    double left_x = 0.25 * width;
    double right_x = 0.75 * width;
    boolean left = false; // true if team element found on the left side
    boolean right = false; // true if found on the right side, indicates middle location
    for (int i = 0; i != boundRect.length; i++) {
        if (boundRect[i].x < left_x)
            left = true;
        if (boundRect[i].x + boundRect[i].width > right_x)
            right = true;

        // draw red bounding rectangles on mat
        // the mat has been converted to HSV so we need to use HSV as well
        Imgproc.rectangle(mat, boundRect[i], new Scalar(0.5, 76.9, 89.8));

    }

    // if there is no yellow regions on a side
    // that side should be blank
    if (left) location = "LEFT";
    else if (right) location = "MIDDLE";
        // if both are false, then there's no teamelement in front.
    else location ="RIGHT";

    return mat; // return the mat with rectangles drawn
}
/*

@Override
public Mat processFrame(Mat input) {
    // "Mat" stands for matrix, which is basically the image that the detector will process
    // the input matrix is the image coming from the camera
    // the function will return a matrix to be drawn on your phone's screen

    // The detector detects team element The camera fits area for two positions.
    // If it finds team element in either of these locations, it can identify whether left or middle.
    // If not team element is on right

    // Make a working copy of the input matrix in HSV
    Mat mat = new Mat();
    Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

    // if something is wrong, we assume there's no team element
    if (mat.empty()) {
        location = "RIGHT";
        return input;
    }

    // We create a HSV range for yellow to detect regular stones
    // NOTE: In OpenCV's implementation,
    // Hue values are half the real value
    //Scalar lowHSV = new Scalar(20, 100, 100); // lower bound HSV for yellow
    //Scalar highHSV = new Scalar(30, 255, 255); // higher bound HSV for yellow

    Scalar lowHSV = new Scalar(65, 60, 60); // lower bound HSV for green
    Scalar highHSV = new Scalar(80, 255, 255); // higher bound HSV for green

    Mat thresh = new Mat();

    // We'll get a black and white image. The white regions represent the regular stones.
    // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
    Core.inRange(mat, lowHSV, highHSV, thresh);

    // Use Canny Edge Detection to find edges
    // you might have to tune the thresholds for hysteresis
    Mat edges = new Mat();
    Imgproc.Canny(thresh, edges, 100, 300);

    // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
    // Oftentimes the edges are disconnected. findContours connects these edges.
    // We then find the bounding rectangles of those contours
    List<MatOfPoint> contours = new ArrayList<>();
    Mat hierarchy = new Mat();
    Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

    MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
    Rect[] boundRect = new Rect[contours.size()];
    for (int i = 0; i < contours.size(); i++) {
        contoursPoly[i] = new MatOfPoint2f();
        Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
        boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
    }

    Size size = input.size();
    double height = size.height;
    double width = size.width;
    // Iterate and check whether the bounding boxes
    // cover left and/or right side of the image
    double left_x = 0.25 * width;
    double right_x = 0.75 * width;
    boolean left = false; // true if team element found on the left side
    boolean right = false; // true if found on the right side, indicates middle location
    for (int i = 0; i != boundRect.length; i++) {
        if (boundRect[i].x < left_x)
            left = true;
        if (boundRect[i].x + boundRect[i].width > right_x)
            right = true;

        // draw red bounding rectangles on mat
        // the mat has been converted to HSV so we need to use HSV as well
        Imgproc.rectangle(mat, boundRect[i], new Scalar(0.5, 76.9, 89.8));

    }

    // if there is no yellow regions on a side
    // that side should be blank
    if (left) location = "LEFT";
    else if (right) location = "MIDDLE";
        // if both are false, then there's no teamelement in front.
    else location ="RIGHT";

    return mat; // return the mat with rectangles drawn
}
 */
        public String getFrameLocation()
        {

            return location;
        }
    }
}
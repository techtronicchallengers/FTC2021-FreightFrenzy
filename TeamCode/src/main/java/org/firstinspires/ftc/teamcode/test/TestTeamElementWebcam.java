package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Disabled

@Autonomous(name = "TestTeamElementWebcam", group = "League")
public class TestTeamElementWebcam extends LinearOpMode
{
    OpenCvCamera webCam;
    TeamElementDeterminationPipeline pipeline;
    BobTheDuckBot frenzyBot = new BobTheDuckBot();
    static boolean firstTime = true;

    @Override
    public void runOpMode()
    {

        frenzyBot.initRobot(hardwareMap);

        int cameraMonitorViewId = frenzyBot.getRobotHardware().getHwMap().appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
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

        @Override
        public  Mat processFrame(Mat input)
        {
            if(firstTime) {

                int startRow = 0;
                int endRow = 0;
                int startColumn = 0;
                int endColumn = 0;

                double[] pixel;
                Size size = input.size();
                double height = size.height;
                double width = size.width;

                double greenValue = 0;
                int leftCount = 0;
                int middleCount = 0;
                double maxGreen = 0;
                for (int i = 0; i < width / 2; i++) {
                    for (int j = 0; j < height; j++) {
                        pixel = input.get(i, j);
                        // telemetry.addData(" j=",  j );
                        //telemetry.addData(" pixel=",  pixel[1] );
                        //telemetry.update();
                        if (pixel != null && pixel.length > 0) {
                            greenValue = pixel[1];
                            if (greenValue > maxGreen) {
                                maxGreen = greenValue;
                            }

                            if (greenValue >= 180) {
                                leftCount++;
                            }

                        } else {
                            //telemetry.addData("Pixel Null at",  i );
                            //telemetry.addData(" ",  j );
                            //telemetry.update();
                        }
                    }
                }
                for (int i = (int) width / 2; i < width; i++) {
                    for (int j = 0; j < height; j++) {
                        pixel = input.get(i, j);
                        if (pixel != null && pixel.length > 0) {
                            greenValue = pixel[1];
                            if (greenValue >= 180) {
                                middleCount++;
                            }
                        } else {
                            //telemetry.addData("Pixel Null at",  i );
                            //telemetry.addData(" ",  j );
                            //telemetry.update();
                        }
                    }
                }


                if (leftCount > 500) {
                    location = "left";
                } else if (middleCount > 500) {
                    location = "middle";
                } else {
                    location = "right";
                }
                telemetry.addData("leftCount", leftCount);
                telemetry.addData("middleCount", middleCount);
                telemetry.addData("greenValue", greenValue);
                telemetry.addData("maxGreen", maxGreen);
                telemetry.addData("width", width);
                telemetry.addData("height", height);
                //telemetry.addData("rightCount",  rightCount);
                telemetry.update();

                firstTime = false;
            }

                return input;

        }


        public String getFrameLocation()
        {

            return location;
        }
    }
}
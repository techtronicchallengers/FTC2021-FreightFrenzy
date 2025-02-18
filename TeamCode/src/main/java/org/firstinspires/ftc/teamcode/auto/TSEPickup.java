package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.assembly.ChassisAssembly;
import org.firstinspires.ftc.teamcode.assembly.SensorNavigation;
import org.firstinspires.ftc.teamcode.assembly.VisualCortex;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

import java.util.Locale;

import org.firstinspires.ftc.teamcode.assembly.BobTheDuckBot;

@Autonomous(name = "TSEPickup")

@Disabled

public class TSEPickup extends LinearOpMode
{
    //Encoder Constants
    final double COUNTS_PER_MOTOR_REV    = 537.6;
    final double DRIVE_GEAR_REDUCTION = 1;
    final double WHEEL_DIAMETER_INCHES = 4.0;
    final double WHEEL_DIAMETER_INCHES_BIG = 5.0;
    final double COUNTS_PER_INCH_4 = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    final double COUNTS_PER_INCH_5 = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES_BIG * Math.PI);
    final double COUNTS_PER_DEGREE = 5.4;

    OpenCvCamera webCam;
    SenseTest.TeamElementDeterminationPipeline pipeline;

    BobTheDuckBot frenzyBot = new BobTheDuckBot();
    ChassisAssembly chassis = null;

    ElapsedTime runtime = new ElapsedTime();

    Orientation angles;
    Acceleration gravity;

    double flipUp = 0.4;
    double flipDown = 0.2;
    double rampUp = 0.45;

    String tsePos = "unknown";

    @Override public void runOpMode()
    {
        frenzyBot.initRobot(hardwareMap);
        //nav = frenzyBot.getNavigation();
        chassis = frenzyBot.getChassisAssembly();

        frenzyBot.initRobot(hardwareMap);
/*
        int cameraMonitorViewId = frenzyBot.getRobotHardware().getHwMap().appContext.getResources().getIdentifier("cameraMonitorViewId", "id", frenzyBot.getRobotHardware().getHwMap().appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(frenzyBot.getRobotHardware().getHwMap().get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new SenseTest.TeamElementDeterminationPipeline(telemetry);
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

        telemetry.addData("currentLocation", pipeline.getFrameLocation());
        telemetry.update();
*/
        //Wait for Start
        telemetry.addData("Waiting for start", "");
        waitForStart();

        rampSpeedEncoderDrive(1.0, 25, 7);
        encoderDrive(1, 2, 7);
        frenzyBot.getRobotHardware().rampServo.setPosition(rampUp);
        frenzyBot.getRobotHardware().intakeBox.setPosition(flipUp);
        encoderTurn(1.0, 120, 7);
        rampSpeedEncoderDrive(1, -12, 7);
        frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frenzyBot.getRobotHardware().lift.setTargetPosition(-1425);

        frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frenzyBot.getRobotHardware().lift.setPower(-0.5);
        sleep(1500);
        frenzyBot.getRobotHardware().intakeBox.setPosition(flipDown);
        sleep(300);
        frenzyBot.getRobotHardware().intakeBox.setPosition(flipUp);

        sleep(400);
        frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(!frenzyBot.getRobotHardware().liftLimit.isPressed()){
            frenzyBot.getRobotHardware().lift.setPower(0.5);
        }
        frenzyBot.getRobotHardware().lift.setPower(0);

        rampSpeedEncoderDrive(1, 40, 7);
        encoderDrive(1, 2, 7);

        encoderTurn(1, 75, 7);

        encoderDrive(1, 5, 7);

        sleep(3000);

        rampSpeedEncoderDrive(1, -20, 7);

    }

    public void turnToAngle(double speed, double desiredAngle, int numLoops)
    {
        double currentAngle = angles.firstAngle;
        double angleToTurn;

        for(int i = 0; i < numLoops; i++)
        {
            readAngle();
            currentAngle = angles.firstAngle;
            angleToTurn = desiredAngle - currentAngle;

            if(Math.abs(angleToTurn) > 180)
            {
                double newAngleToTurn = 360 - Math.abs(angleToTurn);
                if(angleToTurn > 0)
                {
                    newAngleToTurn = -Math.abs(newAngleToTurn);
                    angleToTurn = newAngleToTurn;
                }
                else
                {
                    newAngleToTurn = Math.abs(newAngleToTurn);
                    angleToTurn = newAngleToTurn;
                }
            }

            encoderTurn(speed, angleToTurn, 5);
        }

        telemetry.addData("current angle", currentAngle);
        telemetry.addData("desired angle", desiredAngle);
        telemetry.update();
    }


    void readAngle()
    {
    //    angles   = frenzyBot.getRobotHardware().imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
/*
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = frenzyBot.getRobotHardware().imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = frenzyBot.getRobotHardware().imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return frenzyBot.getRobotHardware().imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return frenzyBot.getRobotHardware().imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


     */
    public void rampSpeedEncoderDrive(double speed, double inches, double timeoutS)
    {
        double setSpeed = speed;
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            frenzyBot.getChassisAssembly().changeToEncoderMode();

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = frenzyBot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH_4);
            newBackRightTarget = frenzyBot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH_4);
            newFrontLeftTarget = frenzyBot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH_5);
            newFrontRightTarget = frenzyBot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH_5);


            frenzyBot.getChassisAssembly().setBackLeftWheelTargetPosition(newBackLeftTarget);
            frenzyBot.getChassisAssembly().setBackRightWheelTargetPosition(newBackRightTarget);
            frenzyBot.getChassisAssembly().setFrontLeftWheelTargetPosition(newFrontLeftTarget);
            frenzyBot.getChassisAssembly().setFrontRightWeelTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            frenzyBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            speed = 0.1;
            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frenzyBot.getChassisAssembly().isBackLeftWheelBusy() && frenzyBot.getChassisAssembly().isBackRightWheelBusy() &&
                            frenzyBot.getChassisAssembly().isFrontLeftWheelBusy() && frenzyBot.getChassisAssembly().isFrontRightWheelBusy())) {

                while((Math.abs(frenzyBot.getChassisAssembly().getFrontLeftWheelCurrentPosition()) < Math.abs(newFrontLeftTarget/4)
                        && Math.abs(speed) < Math.abs(setSpeed)))
                {
                    frenzyBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));
                    speed = speed + 0.005;
                }
                while((Math.abs(speed) < Math.abs(setSpeed))
                        && Math.abs(frenzyBot.getChassisAssembly().getFrontLeftWheelCurrentPosition()) < Math.abs(newFrontLeftTarget/2))
                {
                    frenzyBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));
                    speed = speed + 0.005;
                }
                while(Math.abs(frenzyBot.getChassisAssembly().getFrontLeftWheelCurrentPosition()) < Math.abs(newFrontLeftTarget/2))
                {
                    frenzyBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));
                }
                while((Math.abs(frenzyBot.getChassisAssembly().getFrontLeftWheelCurrentPosition()) > Math.abs(newFrontLeftTarget/2)) && speed > 0.1)
                {
                    frenzyBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));
                    speed = speed -  0.01;
                }

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d : %7d :%7d",
                        newBackLeftTarget,  newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d : %7d : %7d",
                        frenzyBot.getChassisAssembly().getBackLeftWheelCurrentPosition(),
                        frenzyBot.getChassisAssembly().getBackRightWheelCurrentPosition(),
                        frenzyBot.getChassisAssembly().getFrontLeftWheelCurrentPosition(),
                        frenzyBot.getChassisAssembly().getFrontRightWheelCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frenzyBot.getChassisAssembly().stopMoving();

            // Turn off RUN_TO_POSITION
            frenzyBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }//end of rampSpeedEncoderDrive

    public void encoderDrive(double speed, double inches, double timeoutS)
    {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            frenzyBot.getChassisAssembly().changeToEncoderMode();

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = frenzyBot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH_4);
            newBackRightTarget = frenzyBot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH_4);
            newFrontLeftTarget = frenzyBot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH_5);
            newFrontRightTarget = frenzyBot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH_5);



            frenzyBot.getChassisAssembly().setBackLeftWheelTargetPosition(newBackLeftTarget);
            frenzyBot.getChassisAssembly().setBackRightWheelTargetPosition(newBackRightTarget);
            frenzyBot.getChassisAssembly().setFrontLeftWheelTargetPosition(newFrontLeftTarget);
            frenzyBot.getChassisAssembly().setFrontRightWeelTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            frenzyBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            frenzyBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
            frenzyBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
            frenzyBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
            frenzyBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frenzyBot.getChassisAssembly().isBackLeftWheelBusy() && frenzyBot.getChassisAssembly().isBackRightWheelBusy() &&
                            frenzyBot.getChassisAssembly().isFrontLeftWheelBusy() && frenzyBot.getChassisAssembly().isFrontRightWheelBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d : %7d :%7d",
                        newBackLeftTarget,  newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d : %7d : %7d",
                        frenzyBot.getChassisAssembly().getBackLeftWheelCurrentPosition(),
                        frenzyBot.getChassisAssembly().getBackRightWheelCurrentPosition(),
                        frenzyBot.getChassisAssembly().getFrontLeftWheelCurrentPosition(),
                        frenzyBot.getChassisAssembly().getFrontRightWheelCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frenzyBot.getChassisAssembly().stopMoving();

            // Turn off RUN_TO_POSITION
            frenzyBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }//end of encoderDrive

    public void encoderTurn(double speed, double degrees, double timeoutS) {
        double setSpeed = speed;
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            newBackLeftTarget = frenzyBot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int) (-degrees * COUNTS_PER_DEGREE);
            newBackRightTarget = frenzyBot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
            newFrontLeftTarget = frenzyBot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int) (-degrees * COUNTS_PER_DEGREE);
            newFrontRightTarget = frenzyBot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);

            frenzyBot.getChassisAssembly().setBackLeftWheelTargetPosition(newBackLeftTarget);
            frenzyBot.getChassisAssembly().setBackRightWheelTargetPosition(newBackRightTarget);
            frenzyBot.getChassisAssembly().setFrontLeftWheelTargetPosition(newFrontLeftTarget);
            frenzyBot.getChassisAssembly().setFrontRightWeelTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            frenzyBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // reset the timeout time and start motion.
            runtime.reset();

            speed = 0.1;
            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frenzyBot.getChassisAssembly().isBackLeftWheelBusy() && frenzyBot.getChassisAssembly().isBackRightWheelBusy() &&
                            frenzyBot.getChassisAssembly().isFrontLeftWheelBusy() && frenzyBot.getChassisAssembly().isFrontRightWheelBusy())) {

                while((Math.abs(frenzyBot.getChassisAssembly().getFrontLeftWheelCurrentPosition()) < Math.abs(newFrontLeftTarget/4)
                        && Math.abs(speed) < Math.abs(setSpeed)))
                {
                    frenzyBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));
                    speed = speed + 0.0015;
                }
                while((Math.abs(speed) < Math.abs(setSpeed))
                        && Math.abs(frenzyBot.getChassisAssembly().getFrontLeftWheelCurrentPosition()) < Math.abs(newFrontLeftTarget/2))
                {
                    frenzyBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));
                    speed = speed + 0.005;
                }
                while(Math.abs(frenzyBot.getChassisAssembly().getFrontLeftWheelCurrentPosition()) < Math.abs(newFrontLeftTarget/2))
                {
                    frenzyBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));
                }
                while((Math.abs(frenzyBot.getChassisAssembly().getFrontLeftWheelCurrentPosition()) > Math.abs(newFrontLeftTarget/2)) && speed > 0.1)
                {
                    frenzyBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
                    frenzyBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));
                    speed = speed -  0.05;
                }

                // Display it for the driver.
                telemetry.addData("Turn1", "Running to %7d :%7d : %7d :%7d",
                        newBackLeftTarget, newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Turn2", "Running at %7d :%7d : %7d : %7d",
                        frenzyBot.getChassisAssembly().getBackLeftWheelCurrentPosition(),
                        frenzyBot.getChassisAssembly().getBackRightWheelCurrentPosition(),
                        frenzyBot.getChassisAssembly().getFrontLeftWheelCurrentPosition(),
                        frenzyBot.getChassisAssembly().getFrontRightWheelCurrentPosition());
                //telemetry.update();
            }

            // Stop all motion;
            frenzyBot.getChassisAssembly().stopMoving();

            // Turn off RUN_TO_POSITION
            frenzyBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }//end of encoderTurn

    public static class TeamElementDeterminationPipeline extends OpenCvPipeline
    {

        static Telemetry telemetry;

        public TeamElementDeterminationPipeline(Telemetry tele) {
            telemetry = tele;
        }

        String location;


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
        public String getFrameLocation()
        {

            return location;
        }
    }

}
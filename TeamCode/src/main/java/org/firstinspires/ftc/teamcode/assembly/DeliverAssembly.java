package org.firstinspires.ftc.teamcode.assembly;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

public class DeliverAssembly
{
    public RobotHardware robotHardware;

    public DeliverAssembly(RobotHardware hardware)
    {
        robotHardware = hardware;
    }

    public void moveLift(int target, double speed) {

        double setSpeed = speed;

        while((Math.abs(robotHardware.lift.getCurrentPosition()) < Math.abs(target/4)
                && Math.abs(speed) < Math.abs(setSpeed)))
        {
            robotHardware.lift.setPower(Math.abs(speed));
            speed += 0.005;
        }
        while((Math.abs(speed) < Math.abs(setSpeed))
                && Math.abs(robotHardware.lift.getCurrentPosition()) < Math.abs(target/2))
        {
            robotHardware.lift.setPower(Math.abs(speed));
            speed += 0.005;
        }
        while(Math.abs(robotHardware.lift.getCurrentPosition()) < Math.abs(target/2))
        {
            robotHardware.lift.setPower(Math.abs(speed));
        }
        while((Math.abs(robotHardware.lift.getCurrentPosition()) > Math.abs(target/2)) && speed > setSpeed)
        {
            robotHardware.lift.setPower(Math.abs(speed));
            speed -= 0.01;
        }

        robotHardware.lift.setPower(0);

    }
}

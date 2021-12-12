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
    //final public double shootingSpeed = 1.0;
    public RobotHardware robotHardware;

    public DeliverAssembly(RobotHardware hardware)
    {
        robotHardware = hardware;
    }

    public void lift(int level) {
        //robotHardware.lift;
        //DcMotorSimple.Direction.FORWARD
    }
    public void rampUp() {
        robotHardware.rampServo.setPosition(BotConstants.RAMP_UP_POSITION);
    }
    public void rampDown() {
        robotHardware.rampServo.setPosition(BotConstants.RAMP_DOWN_POSITION);
    }
    public void dropElement() {
        //lift to level
        robotHardware.intakeBox.setPosition(BotConstants.DROP_POSITION);
        try {
            Thread.sleep(BotConstants.DROP_ELEMENT_SLEEP);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        robotHardware.intakeBox.setPosition(BotConstants.UP_POSITION);
        lift(0);
    }

}

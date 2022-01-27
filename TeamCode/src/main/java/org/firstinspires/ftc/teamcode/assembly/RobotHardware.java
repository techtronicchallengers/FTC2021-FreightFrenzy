package org.firstinspires.ftc.teamcode.assembly;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


public class RobotHardware
{
    //Chassis
    public DcMotorEx frontLeftWheel = null;
    public DcMotorEx frontRightWheel = null;
    public DcMotorEx backLeftWheel = null;
    public DcMotorEx backRightWheel = null;

    public DcMotorEx intaker = null;
    public DcMotorEx lift = null;

    public DcMotorEx carouselMotor = null;

    public Servo intakeBox = null;
    public Servo rampServo = null;
    public Servo tseServo = null;

    public TouchSensor liftLimit = null;

    //Webcam
    public WebcamName webcam = null;

    public BNO055IMU imu = null;
    public BNO055IMU imu2 = null;

    //Distance Sensors
    //Rev2mDistanceSensor frontRightSensor = null;
    //Rev2mDistanceSensor backRightSensor = null;
    //Rev2mDistanceSensor frontLeftSensor = null;
    //Rev2mDistanceSensor backLeftSensor = null;

    //Distance Sensors
    //public ModernRoboticsI2cRangeSensor rightFrontSensor = null;
    //public ModernRoboticsI2cRangeSensor rightBackSensor = null;
    //public ModernRoboticsI2cRangeSensor backSensor = null;

    //Rev2mDistanceSensor backLaser = null;
    //Rev2mDistanceSensor frontLaser = null;

    //Touch Sensors
   // public TouchSensor liftTouchSensor = null;
   // public TouchSensor topTouch = null;
    //public TouchSensor armReturn = null;
    //public TouchSensor grabTouch = null;

    //Adding the Hardware Map
    private HardwareMap hwMap  = null;
    //public CRServo wheelServo = null;
    public  RobotHardware(HardwareMap ahwMap)
    {
        hwMap = ahwMap;
        webcam = hwMap.get(WebcamName.class, "webcam");
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu2 = hwMap.get(BNO055IMU.class, "imu2");
       // wheelServo = hwMap.get(CRServo.class, "wheelServo");

        //Wheel motors
        frontLeftWheel = hwMap.get(DcMotorEx.class, "frontLeft");
        frontRightWheel = hwMap.get(DcMotorEx.class, "frontRight");
        backLeftWheel = hwMap.get(DcMotorEx.class, "backLeft");
        backRightWheel = hwMap.get(DcMotorEx.class, "backRight");

        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backRightWheel.setDirection(DcMotor.Direction.REVERSE);
        frontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        backLeftWheel.setDirection(DcMotor.Direction.FORWARD);

        frontLeftWheel.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        backLeftWheel.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        frontRightWheel.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        backRightWheel.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        intaker = hwMap.get(DcMotorEx.class, "intaker");
        intaker.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.FLOAT));

        lift = hwMap.get(DcMotorEx.class, "lift");
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        carouselMotor = hwMap.get(DcMotorEx.class, "carouselMotor");
        carouselMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeBox = hwMap.get(Servo.class, "intakeBox");
        rampServo = hwMap.get(Servo.class, "rampServo");
        tseServo = hwMap.get(Servo.class, "tseServo");


        liftLimit = hwMap.get(TouchSensor.class, "liftLimit");



    }

    public HardwareMap getHwMap() {
        return hwMap;
    }
}
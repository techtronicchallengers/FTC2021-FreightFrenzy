package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.assembly.BobTheDuckBot;

import java.util.Locale;

@TeleOp(name = "TeleOpMode", group = "Qualifier")
public class TeleOpMode extends LinearOpMode
{
    Orientation angles;
    Acceleration gravity;

    private double   WHEEL_SPEED = 1.0;
    private static double INTAKE_SPEED = 1.0;

    //Encoder Constants
    final double COUNTS_PER_MOTOR_REV    = 537.6;
    final double DRIVE_GEAR_REDUCTION = 1;
    final double WHEEL_DIAMETER_INCHES = 4.0;
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    final double COUNTS_PER_DEGREE = 9;
    final double distanceBetweenSensors = 11.5;
    final double lpfConstant = 0.7;
    double kpt = -0.018;
    final double kit = 0;
    double kdt = 0;

    boolean liftRaised = false;

    double shooterAngle = 0.85;
    double angleToStraight = 0;

    int pushedRings = 0;

    boolean isIntaking = false;
    boolean intakePressed = false;
    boolean notPowerAng = true;

    //ramp servo variable
    double rampPos = 0.25;

    //tilter servo variable
    double tiltPos = 0.25;
    //servo positions
    double flipUp = 0.78;
    double flipDown = 0.15;
    double rampUp = 0.45;
    double carouselSpeed = 0;
    double tsePos = 0;
    int servoSelector = 0;

    double liftPosition = 0;

    double slowFactor = 0.3;

    boolean isSpinning = false;

    //Creating a Rover robot object
    BobTheDuckBot frenzyBot = new BobTheDuckBot();

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        frenzyBot.initRobot(hardwareMap);


        //Gyro Setup/Initialization
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "imuCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        frenzyBot.getRobotHardware().imu.initialize(parameters);

        // Set up our telemetry dashboard
        //composeTelemetry();

        Servo currentServo = frenzyBot.getRobotHardware().tseServo;

        frenzyBot.getRobotHardware().frontRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        frenzyBot.getRobotHardware().backRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
            frenzyBot.getRobotHardware().rampServo.setPosition(rampUp);
            frenzyBot.getRobotHardware().intakeBox.setPosition(flipUp);


        }
        while (opModeIsActive()) {

            frenzyBot.getRobotHardware().rampServo.setPosition(rampUp);

            liftPosition = frenzyBot.getRobotHardware().lift.getCurrentPosition();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            double normalizer = Math.max(Math.abs(x)+Math.abs(y)+Math.abs(turn), 1);

            telemetry.addData("Lift position: ", liftPosition);
            telemetry.addData("Current Servo: ", currentServo);
            telemetry.addData("Set position: ", tsePos);
            telemetry.addData("Current Position: ", currentServo.getPosition());
            telemetry.update();

            //Movement

            while(liftRaised){
                if(Math.abs(gamepad1.right_stick_y) > 0.25){

                    frenzyBot.getChassisAssembly().moveForward(-slowFactor*gamepad1.right_stick_y/Math.abs(gamepad1.right_stick_y));

                }

                else {
                    frenzyBot.getRobotHardware().frontRightWheel.setPower(Math.pow((y - x - turn) / normalizer, 1));
                    frenzyBot.getRobotHardware().backRightWheel.setPower(Math.pow((y + x - turn) / normalizer, 1));
                    frenzyBot.getRobotHardware().frontLeftWheel.setPower(Math.pow((y + x + turn) / normalizer, 1));
                    frenzyBot.getRobotHardware().backLeftWheel.setPower(Math.pow(((y - x + turn) / normalizer), 1));
                }

                if(gamepad1.y){
                    ElapsedTime timer = new ElapsedTime();

                    while(timer.seconds() < 0.2 && frenzyBot.getRobotHardware().intakeBox.getPosition() != flipDown){
                        frenzyBot.getRobotHardware().intakeBox.setPosition(flipUp - (timer.seconds() * (flipUp - flipDown) / 0.2));

                    }
                    sleep(700);
                    frenzyBot.getRobotHardware().intakeBox.setPosition(flipUp);

                    frenzyBot.getIntakeAssembly().stopIntake();

                    sleep(400);
                    frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    while(!frenzyBot.getRobotHardware().liftLimit.isPressed()){
                        frenzyBot.getRobotHardware().lift.setPower(0.5);
                    }
                    frenzyBot.getRobotHardware().lift.setPower(0);
                    liftRaised = false;
                }

            }

            if(Math.abs(gamepad1.right_stick_y) > 0.25){

                frenzyBot.getChassisAssembly().moveForward(-slowFactor*gamepad1.right_stick_y/Math.abs(gamepad1.right_stick_y));

            }

            else {
                frenzyBot.getRobotHardware().frontRightWheel.setPower(Math.pow((y - x - turn) / normalizer, 1));
                frenzyBot.getRobotHardware().backRightWheel.setPower(Math.pow((y + x - turn) / normalizer, 1));
                frenzyBot.getRobotHardware().frontLeftWheel.setPower(Math.pow((y + x + turn) / normalizer, 1));
                frenzyBot.getRobotHardware().backLeftWheel.setPower(Math.pow(((y - x + turn) / normalizer), 1));
            }
            //intake
            if (gamepad1.left_trigger > 0.1) {
                frenzyBot.getRobotHardware().intaker.setDirection(DcMotorSimple.Direction.REVERSE);
                sleep(100);
                frenzyBot.getIntakeAssembly().intake(1.0);
            } else if (gamepad1.right_trigger > 0.1) {
                frenzyBot.getRobotHardware().intaker.setDirection(DcMotorSimple.Direction.FORWARD);
                sleep(100);
                frenzyBot.getIntakeAssembly().intake(1.0);
            } else {
                frenzyBot.getIntakeAssembly().stopIntake();
            }

            //flipper
            if (gamepad1.y) {
                frenzyBot.getRobotHardware().intakeBox.setPosition(flipDown);
                sleep(700);
                frenzyBot.getRobotHardware().intakeBox.setPosition(flipUp);
            }

            //tse
            if(gamepad2.dpad_up){
                tsePos += 0.01;
                sleep(300);
            }

            if(gamepad2.dpad_down){
                tsePos -= 0.01;
                sleep(300);
            }

            if(gamepad2.dpad_right){
                currentServo.setPosition(tsePos);
                sleep(300);
            }

            if(servoSelector % 3 == 0){
                currentServo = frenzyBot.getRobotHardware().tseServo;
            }

            if(servoSelector % 3 == 1){
                currentServo = frenzyBot.getRobotHardware().rampServo;
            }

            if(servoSelector % 3 == 2){
                currentServo = frenzyBot.getRobotHardware().intakeBox;
            }

            if(gamepad2.a){
                servoSelector += 1;
                sleep(300);
            }

            if(gamepad2.x){
                servoSelector -= 1;
                sleep(300);
            }

            if(gamepad1.dpad_left){
                turnPID(0,5);
                sleep(500);
                frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frenzyBot.getRobotHardware().lift.setTargetPosition(-1390);

                frenzyBot.getRobotHardware().intaker.setDirection(DcMotorSimple.Direction.REVERSE);
                sleep(100);
                frenzyBot.getIntakeAssembly().intake(0.4);
            }

            //lift
            if (gamepad1.left_bumper) {
                frenzyBot.getRobotHardware().frontRightWheel.setPower(Math.pow((y - x + 0.5) / normalizer, 1));
                frenzyBot.getRobotHardware().backRightWheel.setPower(Math.pow((y + x + 0.5) / normalizer, 1));
                frenzyBot.getRobotHardware().frontLeftWheel.setPower(Math.pow((y + x - 0.5) / normalizer, 1));
                frenzyBot.getRobotHardware().backLeftWheel.setPower(Math.pow(((y - x - 0.5) / normalizer), 1));
            }

            else if (gamepad1.right_bumper){
                frenzyBot.getRobotHardware().frontRightWheel.setPower(Math.pow((y - x - 0.5) / normalizer, 1));
                frenzyBot.getRobotHardware().backRightWheel.setPower(Math.pow((y + x - 0.5) / normalizer, 1));
                frenzyBot.getRobotHardware().frontLeftWheel.setPower(Math.pow((y + x + 0.5) / normalizer, 1));
                frenzyBot.getRobotHardware().backLeftWheel.setPower(Math.pow(((y - x + 0.5) / normalizer), 1));
            }

            else if (gamepad1.dpad_down) {
                frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frenzyBot.getRobotHardware().lift.setTargetPosition(-790);

                frenzyBot.getRobotHardware().intaker.setDirection(DcMotorSimple.Direction.REVERSE);
                sleep(100);
                frenzyBot.getIntakeAssembly().intake(0.4);

                frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frenzyBot.getRobotHardware().lift.setPower(-0.5);
                sleep(1500);
                frenzyBot.getRobotHardware().intakeBox.setPosition(flipDown);
                sleep(700);
                frenzyBot.getRobotHardware().intakeBox.setPosition(flipUp);

                frenzyBot.getIntakeAssembly().stopIntake();

                sleep(400);
                frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                while(!frenzyBot.getRobotHardware().liftLimit.isPressed()){
                    frenzyBot.getRobotHardware().lift.setPower(0.5);
                }
                frenzyBot.getRobotHardware().lift.setPower(0);
            }

            else if (gamepad1.dpad_up) {
                frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frenzyBot.getRobotHardware().lift.setTargetPosition(-1390);

                frenzyBot.getRobotHardware().intaker.setDirection(DcMotorSimple.Direction.REVERSE);
                sleep(100);
                frenzyBot.getIntakeAssembly().intake(0.4);

                frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frenzyBot.getRobotHardware().lift.setPower(-0.5);
                sleep(1500);

                ElapsedTime timer = new ElapsedTime();

                while(timer.seconds() < 0.2 && frenzyBot.getRobotHardware().intakeBox.getPosition() != flipDown){
                    frenzyBot.getRobotHardware().intakeBox.setPosition(flipUp - (timer.seconds() * (flipUp - flipDown) / 0.2));

                }
                sleep(700);
                frenzyBot.getRobotHardware().intakeBox.setPosition(flipUp);

                frenzyBot.getIntakeAssembly().stopIntake();

                sleep(400);
                frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                while(!frenzyBot.getRobotHardware().liftLimit.isPressed()){
                    frenzyBot.getRobotHardware().lift.setPower(0.5);
                }
                frenzyBot.getRobotHardware().lift.setPower(0);
            }

            else if (gamepad1.dpad_right) {
                frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frenzyBot.getRobotHardware().lift.setTargetPosition(-1085);

                frenzyBot.getRobotHardware().intaker.setDirection(DcMotorSimple.Direction.REVERSE);
                sleep(100);
                frenzyBot.getIntakeAssembly().intake(0.4);

                frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frenzyBot.getRobotHardware().lift.setPower(-0.5);
                sleep(1500);
                frenzyBot.getRobotHardware().intakeBox.setPosition(flipDown);
                sleep(700);
                frenzyBot.getRobotHardware().intakeBox.setPosition(flipUp);

                frenzyBot.getIntakeAssembly().stopIntake();

                sleep(400);
                frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                while(!frenzyBot.getRobotHardware().liftLimit.isPressed()){
                    frenzyBot.getRobotHardware().lift.setPower(0.5);
                }
                frenzyBot.getRobotHardware().lift.setPower(0);
            }

            else{
                frenzyBot.getRobotHardware().lift.setPower(0);
            }


            if(gamepad1.b){
                isSpinning = true;
                sleep(300);
            }

            if(gamepad1.x){
                isSpinning = true;
                sleep(300);
            }

            while(isSpinning){
                while(frenzyBot.getRobotHardware().carouselMotor.getCurrentPosition() < 1120*(15/4)){
                    frenzyBot.getRobotHardware().carouselMotor.setPower(0.75);
                    telemetry.addData("Status:", "turning");
                    telemetry.addData("Position:", frenzyBot.getRobotHardware().carouselMotor.getCurrentPosition());
                    telemetry.addData("Power:", frenzyBot.getRobotHardware().carouselMotor.getPower());
                    telemetry.update();
                }
                telemetry.addData("Status:","done");
                telemetry.update();
                frenzyBot.getRobotHardware().carouselMotor.setPower(0);
                sleep(300);
                if(gamepad1.b){
                    isSpinning = false;
                    sleep(100);
                }
            }

        }

    }


    /*
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
    }
*/
/*

    /**
     *ENCODER DRIVE METHOD
     * @param speed (at which the robot should move)
     * @param inches (positive is forward, negative is backwards)
     * @param timeoutS (the robot will stop moving if it after this many seconds)


    public void encoderDrive(double speed, double inches, double timeoutS)
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
            newBackLeftTarget = frenzyBot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newBackRightTarget = frenzyBot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFrontLeftTarget = frenzyBot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFrontRightTarget = frenzyBot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);


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

                // Display it for the driver
            }

            // Stop all motion;
            frenzyBot.getChassisAssembly().stopMoving();

            // Turn off RUN_TO_POSITION
            frenzyBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }//end of encoderDrive

    public void encoderTurn(double speed, double degrees, double timeoutS) {
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


            }

            // Stop all motion;
            frenzyBot.getChassisAssembly().stopMoving();

            // Turn off RUN_TO_POSITION
            frenzyBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }//end of encoderTurn
*/
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /*void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.


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

     */

    /*void readAngle()
    {
        angles   = frenzyBot.getRobotHardware().imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
    */

    public void turnPID(double degrees, double margin) {

        frenzyBot.getChassisAssembly().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frenzyBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double Integral = 0;
        double lastError = 0;
        double Derivative;
        double Power;

        double reference = degrees;
        double errorMargin = margin;

        double estimate = 0;
        double lastEstimate = 0;

        double State = frenzyBot.getRobotHardware().imu.getAngularOrientation().firstAngle;

        double Error = reference - State;

        double time = 0;

        ElapsedTime timer = new ElapsedTime();

        while (Math.abs(Error) > errorMargin) {

            State = frenzyBot.getRobotHardware().imu.getAngularOrientation().firstAngle;
            Error = reference - State;

            double delta = Error - lastError;

            estimate = (1 - lpfConstant) * delta + lpfConstant * lastEstimate;

            Derivative = estimate / timer.seconds();
            Integral += (Error * timer.seconds());
            Power = (kpt * Error) + (kit * Integral) + (kdt * Derivative);
            frenzyBot.getChassisAssembly().turn(Power);

            lastEstimate = estimate;

            lastError = Error;

            RobotLog.d(time + "," + reference + "," + State + "," + Power + "," + Derivative + "," + frenzyBot.getRobotHardware().imu.getAngularOrientation().firstAngle + "," + frenzyBot.getRobotHardware().imu2.getAngularOrientation().firstAngle);

            time += timer.seconds();

            timer.reset();

            if (gamepad1.x) {
                frenzyBot.getChassisAssembly().stopMoving();
                break;
            }


        }

        frenzyBot.getChassisAssembly().stopMoving();

    }


}
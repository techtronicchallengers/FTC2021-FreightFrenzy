package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    final double kit = -0.001;
    double kdt = -0.005;

    boolean liftRaised = false;

    //servo positions
    double flipUp = 0.78;
    double flipDown = 0.15;
    double rampUp = 0.45;
    double tsePos = 0;
    double gateOpen = 0.4;
    double gateClosed = 0.74;

    //lift positions
    double liftPosition = 0;
    int topPos = -1390;
    int midPos = -1000;
    int lowPos = -760;

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
      //  frenzyBot.getRobotHardware().imu.initialize(parameters);

        // Set up our telemetry dashboard
        //composeTelemetry();

        frenzyBot.getRobotHardware().frontRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        frenzyBot.getRobotHardware().backRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
            frenzyBot.getRobotHardware().rampServo.setPosition(rampUp);
            frenzyBot.getRobotHardware().intakeBox.setPosition(flipUp);
            sleep(700);
            frenzyBot.getRobotHardware().gate.setPosition(gateClosed);
        }

        while (opModeIsActive()) {

            telemetry.addData("Lift position: ", liftPosition);
            telemetry.addData("Set position: ", tsePos);
            telemetry.addData("Current Position: ", frenzyBot.getRobotHardware().gate.getPosition());
            telemetry.update();

            frenzyBot.getRobotHardware().rampServo.setPosition(rampUp);

            liftPosition = frenzyBot.getRobotHardware().lift.getCurrentPosition();

            //movement
            frenzyBot.getChassisAssembly().allowMovement(gamepad1);

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

            //tse
            if(gamepad2.left_stick_button){
                frenzyBot.getRobotHardware().lift.setPower(0.5 * gamepad2.left_stick_y);
            }

            if (gamepad1.dpad_down) {
                frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frenzyBot.getRobotHardware().lift.setTargetPosition(lowPos);

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
                    frenzyBot.getChassisAssembly().allowMovement(gamepad1);
                    frenzyBot.getRobotHardware().lift.setPower(0.5);
                }
                frenzyBot.getRobotHardware().lift.setPower(0);
            }

            else if (gamepad1.dpad_up) {
                frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frenzyBot.getRobotHardware().intaker.setDirection(DcMotorSimple.Direction.REVERSE);
                frenzyBot.getIntakeAssembly().intake(0.4);

                while(frenzyBot.getRobotHardware().lift.getCurrentPosition() > topPos){
                    frenzyBot.getRobotHardware().lift.setPower(-0.5);
                    frenzyBot.getChassisAssembly().allowMovement(gamepad1);
                }

                frenzyBot.getRobotHardware().lift.setPower(-0.1);

                ElapsedTime timer = new ElapsedTime();

                while(timer.seconds() < 0.2 && frenzyBot.getRobotHardware().intakeBox.getPosition() != flipDown){
                    frenzyBot.getChassisAssembly().allowMovement(gamepad1);
                    frenzyBot.getRobotHardware().intakeBox.setPosition(flipUp - (timer.seconds() * (flipUp - flipDown) / 0.2));
                }

                while(timer.seconds() < 1){
                    frenzyBot.getChassisAssembly().allowMovement(gamepad1);
                }

                frenzyBot.getRobotHardware().intakeBox.setPosition(flipUp);

                frenzyBot.getIntakeAssembly().stopIntake();

                sleep(400);

                frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                while(!frenzyBot.getRobotHardware().liftLimit.isPressed()){
                    frenzyBot.getChassisAssembly().allowMovement(gamepad1);
                    frenzyBot.getRobotHardware().lift.setPower(0.5);
                }
                frenzyBot.getRobotHardware().lift.setPower(0);

                liftRaised = true;
            }

            else if (gamepad1.dpad_right) {
                frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frenzyBot.getRobotHardware().lift.setTargetPosition(midPos);

                frenzyBot.getRobotHardware().intaker.setDirection(DcMotorSimple.Direction.REVERSE);
                sleep(100);
                frenzyBot.getIntakeAssembly().intake(0.4);

                frenzyBot.getRobotHardware().lift.setPower(-0.5);
                sleep(1500);
                frenzyBot.getRobotHardware().intakeBox.setPosition(flipDown);
                sleep(700);
                frenzyBot.getRobotHardware().intakeBox.setPosition(flipUp);

                frenzyBot.getIntakeAssembly().stopIntake();

                sleep(400);
                frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                while(!frenzyBot.getRobotHardware().liftLimit.isPressed()){
                    frenzyBot.getChassisAssembly().allowMovement(gamepad1);
                    frenzyBot.getRobotHardware().lift.setPower(0.5);
                }
                frenzyBot.getRobotHardware().lift.setPower(0);
            }

            if((gamepad1.b) && frenzyBot.getRobotHardware().carouselMotor.getPower() < -0.95){
                frenzyBot.getRobotHardware().carouselMotor.setPower(0);
                sleep(200);
            }

            else if(gamepad1.b){
                frenzyBot.getRobotHardware().carouselMotor.setPower(-1);
                sleep(200);
            }

            if(gamepad1.x && frenzyBot.getRobotHardware().carouselMotor.getPower() > 0.95){
                frenzyBot.getRobotHardware().carouselMotor.setPower(0);
                sleep(200);
            }

            else if(gamepad1.x){
                frenzyBot.getRobotHardware().carouselMotor.setPower(1);
                sleep(200);
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

    /*public void turnPID(double degrees, double margin) {

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

            time += timer.seconds();

            timer.reset();

            telemetry.addData("State:", State);
            telemetry.addData("Power:", Power);
            telemetry.update();

        }

        frenzyBot.getChassisAssembly().stopMoving();

    }*/

}
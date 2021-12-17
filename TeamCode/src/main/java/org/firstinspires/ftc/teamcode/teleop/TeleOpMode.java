package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    final double LIFT_WHEEL_DIAMETER = 1.37795;
    final double LIFT_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (LIFT_WHEEL_DIAMETER * Math.PI);
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    final double COUNTS_PER_DEGREE = 9;
    final double distanceBetweenSensors = 11.5;

    double shooterAngle = 0.85;
    double angleToStraight = 0;

    int pushedRings = 0;

    boolean isIntaking = false;
    boolean intakePressed = false;
    boolean notPowerAng = true;


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
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "imuCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        frenzyBot.getRobotHardware().imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();
        telemetry.update();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        while (opModeIsActive())
        {
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;

            //Movement
            /*
            if (drive < 0) {
                frenzyBot.getChassisAssembly().moveForward(Math.abs(drive));
            }
            else if (drive > 0){
                frenzyBot.getChassisAssembly().moveBackwards(Math.abs(drive));
            }
            //turn right
            else if (turn > 0) {
                frenzyBot.getChassisAssembly().turnRight(Math.abs(turn));
            }
            //turn left
            else if (turn < 0) {
                frenzyBot.getChassisAssembly().turnLeft(Math.abs(turn));
            }
            else{
                frenzyBot.getChassisAssembly().moveForward(0);

            }


             */
            if(gamepad1.x == true){

                frenzyBot.getIntakeAssembly().intake();
            }
            else{
                frenzyBot.getIntakeAssembly().stopIntake();
            }
            frenzyBot.getChassisAssembly().forwardTurn(-gamepad1.left_stick_y, gamepad1.left_stick_x);


        }

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

    /**
     *ENCODER DRIVE METHOD
     * @param speed (at which the robot should move)
     * @param inches (positive is forward, negative is backwards)
     * @param timeoutS (the robot will stop moving if it after this many seconds)
     */
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

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d : %7d :%7d",
                        newBackLeftTarget, newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d : %7d",
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

    public void encoderLiftDrive(double speed, double inches, double timeoutS)
{
    double setSpeed = speed;
    int newLiftTarget;

    // Ensure that the opmode is still active
    if (opModeIsActive()) {

        frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Determine new target position, and pass to motor controller
        newLiftTarget = frenzyBot.getRobotHardware().lift.getCurrentPosition() + (int)(inches * LIFT_COUNTS_PER_INCH);

        frenzyBot.getRobotHardware().lift.setTargetPosition(newLiftTarget);

        // Turn On RUN_TO_POSITION
        frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // reset the timeout time and start motion.
        runtime.reset();

        speed = 0.1;
        // keep looping while we are still active, and there is time left, and both motors are running.
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (frenzyBot.getRobotHardware().lift.isBusy())) {

            while((Math.abs(frenzyBot.getRobotHardware().lift.getCurrentPosition()) < Math.abs(newLiftTarget)
                    && Math.abs(speed) < Math.abs(setSpeed)))
            {
                frenzyBot.getRobotHardware().lift.setPower(Math.abs(speed));
                speed = speed + 0.005;
            }

            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d",
                    newLiftTarget);
            telemetry.addData("Path2",  "Running at %7d",
                    frenzyBot.getChassisAssembly().getBackLeftWheelCurrentPosition(), frenzyBot.getRobotHardware().lift.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        frenzyBot.getRobotHardware().lift.setPower(0);

        // Turn off RUN_TO_POSITION
        frenzyBot.getRobotHardware().lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}//end of liftEncoderDrive

    public void lift(int level) {
        if(level == 1) {
            encoderLiftDrive(0.1,2, 5);

        }
        else if(level == 2) {
            encoderLiftDrive(0.1,4, 5);
        }
        else if(level == 3) {
            encoderLiftDrive(0.1,6,5);
        }
    }
    public void resetLift(){
    }


    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

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

    void readAngle()
    {
        angles   = frenzyBot.getRobotHardware().imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
}
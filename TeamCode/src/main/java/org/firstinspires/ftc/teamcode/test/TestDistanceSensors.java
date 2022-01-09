package org.firstinspires.ftc.teamcode.test;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.assembly.BobTheDuckBot;

@Disabled
@TeleOp(name = "DistanceSensorsTest", group = "Test")
public class TestDistanceSensors extends LinearOpMode
{
    //Creating a Rover robot object
    BobTheDuckBot bobTheDuckBot = new BobTheDuckBot();

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        bobTheDuckBot.initRobot(hardwareMap);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while (opModeIsActive())
        {
            /*
            telemetry.addData("Front Right", bobTheDuckBot.getNavigation().frontRightDistance());
            telemetry.addData("Back Right", bobTheDuckBot.getNavigation().backRightDistance());
            telemetry.addData("Front Left", bobTheDuckBot.getNavigation().frontLeftDistance());
            telemetry.addData("Back Left", bobTheDuckBot.getNavigation().backLeftDistance());

            telemetry.addData("Front", bobTheDuckBot.getNavigation().frontDistance());
            telemetry.addData("Back", bobTheDuckBot.getNavigation().backDistance());

            telemetry.addData("Right Angle", bobTheDuckBot.getNavigation().rightAngle());
            telemetry.addData("Left Angle", bobTheDuckBot.getNavigation().leftAngle());
            telemetry.update();
             */
        }
    }
}
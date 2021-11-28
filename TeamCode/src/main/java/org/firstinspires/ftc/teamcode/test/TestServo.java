package org.firstinspires.ftc.teamcode.test;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.assembly.BobTheDuckBot;


@TeleOp(name = "ServoTest", group = "Test")
public class TestServo extends LinearOpMode
{
    //test program for carousel servo
    private ElapsedTime runtime = new ElapsedTime();

    public CRServo wheelServo = null;
    @Override
    public void runOpMode() {
        wheelServo = hardwareMap.get(CRServo.class, "wheelServo");

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while (opModeIsActive())
        {
            if(gamepad1.y)
            {
                wheelServo.setPower(0.75);
            }
            else{
                wheelServo.setPower(0);
            }

        }
    }
}
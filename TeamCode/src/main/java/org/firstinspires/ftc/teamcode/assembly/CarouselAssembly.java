package org.firstinspires.ftc.teamcode.assembly;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.assembly.BobTheDuckBot;

public class CarouselAssembly
{
    public RobotHardware robotHardware;


    public CarouselAssembly(RobotHardware hardware)
    {
        robotHardware = hardware;
    }
    public void rotateRedCarousel() {
        robotHardware.carouselServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void rotateBlueCarousel() {
        robotHardware.carouselServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }


}

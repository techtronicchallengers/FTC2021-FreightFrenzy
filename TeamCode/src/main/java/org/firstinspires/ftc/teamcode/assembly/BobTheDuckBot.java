package org.firstinspires.ftc.teamcode.assembly;

import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Created by Athira on 10/14/2018.
 */

public class BobTheDuckBot
{

    private static RobotHardware robotHardware = null;
    private ChassisAssembly chassisAssembly = null;
    private IntakeAssembly intakeAssembly = null;
    private DeliverAssembly shooterAssembly = null;
    private CarouselAssembly wobbleAssembly = null;
    private SensorNavigation navigation = null;
    private VisualCortex vuforia_tf = null;

    public void initRobot (HardwareMap hwMap)
    {
        robotHardware = new RobotHardware(hwMap);
        buildChassisAssembly();
        buildIntakeAssembly();
        buildShooterAssembly();
        buildWobbleAssembly();
        buildNavigation();
    }
    public void buildChassisAssembly ()
    {
        this.chassisAssembly = new ChassisAssembly(robotHardware);
    }
    public ChassisAssembly getChassisAssembly()
    {
        return chassisAssembly;
    }
    public void buildIntakeAssembly ()
    {
        this.intakeAssembly = new IntakeAssembly(robotHardware);
    }
    public IntakeAssembly getIntakeAssembly()
    {
        return intakeAssembly;
    }
    public void buildShooterAssembly ()
    {
        this.shooterAssembly = new DeliverAssembly(robotHardware);
    }
    public DeliverAssembly getShooterAssembly()
    {
        return shooterAssembly;
    }
    public void buildWobbleAssembly ()
    {
        this.wobbleAssembly = new CarouselAssembly(robotHardware);
    }
    public CarouselAssembly getWobbleAssembly()
    {
        return wobbleAssembly;
    }
    public void buildNavigation()
    {
        this.navigation = new SensorNavigation(robotHardware);
    }
    public void initializeVuforiaAndTensorFlow()
    {
        this.vuforia_tf = new VisualCortex(robotHardware);
        vuforia_tf.initVuforia();
        vuforia_tf.initTfod();

    }
    public void loadVuforiaTrackables()
    {
       vuforia_tf.loadTrackables();
    }
    public SensorNavigation getNavigation()
    {
        return navigation;
    }
    public VisualCortex getVisualCortex()
    {
        return vuforia_tf;
    }
    public RobotHardware getRobotHardware()
    {
        return robotHardware;
    }
}

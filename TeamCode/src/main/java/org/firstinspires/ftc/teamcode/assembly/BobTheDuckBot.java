package org.firstinspires.ftc.teamcode.assembly;

import com.qualcomm.robotcore.hardware.HardwareMap;


public class BobTheDuckBot
{

    private static RobotHardware robotHardware = null;
    private ChassisAssembly chassisAssembly = null;
    private IntakeAssembly intakeAssembly = null;
    private DeliverAssembly placerAssembly = null;
    private CarouselAssembly carouselAssembly = null;
    private SensorNavigation navigation = null;
    private VisualCortex vuforia_tf = null;

    public void initRobot (HardwareMap hwMap)
    {
        robotHardware = new RobotHardware(hwMap);
        buildChassisAssembly();
        buildIntakeAssembly();
        buildPlacerAssembly();
        buildCarouselAssembly();
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
    public void buildPlacerAssembly ()
    {
        this.placerAssembly = new DeliverAssembly(robotHardware);
    }
    public DeliverAssembly getPlacerAssembly()
    {
        return placerAssembly;
    }
    public void buildCarouselAssembly ()
    {
        this.carouselAssembly = new CarouselAssembly(robotHardware);
    }
    public CarouselAssembly getCarouselAssembly()
    {
        return carouselAssembly;
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

package team.gif.robot;


public abstract class RobotMap {
    // Controllers
    public static final int DRIVER_CONTROLLER_ID = 0;
    public static final int AUX_CONTROLLER_ID = 1;
    public static final int TEST_CONTROLLER_ID = 2;



    public static final int PIGEON_ID = 9;
    public static final int CIM_MOTOR_ID = 10;
    public static final int NEO_MOTOR_ID = 45; // SparkMAX

    // DIO (Digital Input/Output) channels on the RoboRIO
    public static final int LIMIT_SWITCH_ID = 9;

    // Pneumatics - placeholders, set to match how your PCM and solenoid are wired
    public static final int PCM_ID = 0; // CAN ID of the PCM (0 is the factory default)
    public static final int SOLENOID_FORWARD_CHANNEL = 0;
    public static final int SOLENOID_REVERSE_CHANNEL = 1;
}

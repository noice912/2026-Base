package team.gif.robot;


public abstract class RobotMap {
    // Controllers
    public static final int DRIVER_CONTROLLER_ID = 0;
    public static final int AUX_CONTROLLER_ID = 1;
    public static final int TEST_CONTROLLER_ID = 2;



    public static final int PIGEON_ID = 9;
    public static final int CIM_MOTOR_ID = 10;
    public static final int NEO_MOTOR_ID = 45; // SparkMAX

    // PWM ports on the RoboRIO
    public static final int NEO_PWM_PORT = 0; // SparkMAX signal cable - placeholder, set to the PWM port it's plugged into

    // DIO (Digital Input/Output) channels on the RoboRIO
    public static final int LIMIT_SWITCH_ID = 9;
}

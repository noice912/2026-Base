package team.gif.robot;

public final class Constants {
    public static final double DEBOUNCE_DEFAULT = 0.020;

    public static final double CIM_MOTOR_PERCENT = 0.3;

    public static final double CIM_LIMIT_SWITCH_SCALE = 0.5;

    public static final double NEO_VOLTAGE = 3.0;

    public static final int NEO_CURRENT_LIMIT = 50;

    public static final double NEO_POSITION_SETPOINT = 10.0;
    public static final double NEO_POSITION_KP = 0.1;
    public static final double NEO_POSITION_KI = 0.0;
    public static final double NEO_POSITION_KD = 0.0;
    public static final double NEO_POSITION_MAX_OUTPUT = 0.3;

    public static final double NEO_VELOCITY_RPM = 1000.0;
    public static final double NEO_VELOCITY_KV = 12.0 / 5676.0;
    public static final double NEO_VELOCITY_KP = 0.001;
    public static final double NEO_VELOCITY_KI = 0.0;
    public static final double NEO_VELOCITY_KD = 0.0;

    public static final class Joystick {
        public static final double DEADBAND = 0.1;
    }
}
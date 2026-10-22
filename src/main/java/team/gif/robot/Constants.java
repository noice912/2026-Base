// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double DEBOUNCE_DEFAULT = 0.020;

    // Percent output for the CIM motor (-1.0 to 1.0). Start low when testing.
    public static final double CIM_MOTOR_PERCENT = 0.3;

    // Voltage for the NEO motor (-12 to 12). Start low when testing.
    public static final double NEO_VOLTAGE = 3.0;

    // Current limit for the NEO in amps (protects the motor, from the REV config docs)
    public static final int NEO_CURRENT_LIMIT = 50;
    public static final double SPARK_MOTOR_VOLTAGE = 2.5;

    // NEO position control (units are motor rotations)
    public static final double NEO_POSITION_SETPOINT = 10.0;
    public static final double NEO_POSITION_KP = 0.1;
    public static final double NEO_POSITION_KI = 0.0;
    public static final double NEO_POSITION_KD = 0.0;
    public static final double NEO_POSITION_MAX_OUTPUT = 0.3; // max percent output while moving to the setpoint

    // NEO velocity control (units are RPM, output is volts)
    public static final double NEO_VELOCITY_RPM = 1000.0;
    public static final double NEO_VELOCITY_KV = 12.0 / 5676.0; // volts per RPM (NEO free speed is ~5676 RPM at 12V)
    public static final double NEO_VELOCITY_KP = 0.001; // volts per RPM of error
    public static final double NEO_VELOCITY_KI = 0.0;
    public static final double NEO_VELOCITY_KD = 0.0;

    //These constants should be referenced via Robot.swerveDrive.getConstants();

    public static final class Joystick {
        public static final double DEADBAND = 0.1;

    //These constants are for neo-voltage percentage %

    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DrivebaseConstants {
    public static final double MAX_SPEED = Units.feetToMeters(17.5);
  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int FEEDER_MOTOR_ID = 6;
    public static final int INTAKE_LAUNCHER_MOTOR_ID = 5;

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final Current FEEDER_MOTOR_CURRENT_LIMIT = Amp.of(80);
    public static final Current LAUNCHER_MOTOR_CURRENT_LIMIT = Amp.of(80);

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double INTAKING_FEEDER_VOLTAGE = -12;
    public static final double INTAKING_INTAKE_VOLTAGE = 10;
    public static final double LAUNCHING_FEEDER_VOLTAGE = 12; // 9
    public static final double LAUNCHING_LAUNCHER_VOLTAGE = 6.05; // 10.6
    public static final double LAUNCHING_FARLAUNCHER_VOLTAGE = 12; // 10.6
    public static final double SPIN_UP_FEEDER_VOLTAGE = -7; // -6
    public static final double SPIN_UP_SECONDS = 0.3;
    // ===== NEW: WPILib PID Constants for Launcher Flywheel Control =====

    // PID Gains - START WITH THESE, THEN TUNE ON YOUR ROBOT
    // These values work with RPM (not RPS) for easier tuning

    // kP: Proportional gain - voltage per RPM of error
    // Start small since we're working in RPM (large numbers)
    public static final double LAUNCHER_KP = 0.0001;

    // kI: Integral gain - voltage per RPM*second of accumulated error
    // Usually not needed for flywheel control - start at 0
    public static final double LAUNCHER_KI = 0.0;

    // kD: Derivative gain - voltage per (RPM/second) rate of change
    // Helps reduce oscillation - start at 0, add if needed
    public static final double LAUNCHER_KD = 0.0;

    // Feedforward Gains
    // kS: Static gain - voltage to overcome friction (volts)
    // Minimum voltage needed to get the motor moving
    public static final double LAUNCHER_KS = 0.0;

    // kV: Velocity gain - voltage per rotation/second
    // This is the main feedforward term for velocity control
    // Calculate as: kV ≈ 12V / (max_rpm / 60)
    // For 5000 RPM max: kV ≈ 12 / (5000/60) ≈ 0.144
    public static final double LAUNCHER_KV = 0.1105;

    // I-Zone: Maximum integral accumulation (in RPM)
    // Prevents integral windup - set to max acceptable error
    public static final double LAUNCHER_IZONE = 100.0;

    // Target velocities in RPM
    // Tune these values based on testing to find optimal shooting speeds
    public static final double LAUNCHER_TARGET_RPM = 3400.0; // Normal shot
    public static final double LAUNCHER_FAR_TARGET_RPM = 4000.0; // Far shot

    // Velocity tolerance in RPM
    // How close to target velocity before considering "at speed"
    public static final double LAUNCHER_VELOCITY_TOLERANCE = 50.0;
  }

  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double DEADBAND = 0.1;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

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
    public static final LinearVelocity MAX_SPEED = FeetPerSecond.of(17.5);
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
    public static final Voltage INTAKING_FEEDER_VOLTAGE = Volts.of(-12);
    public static final Voltage INTAKING_INTAKE_VOLTAGE = Volts.of(10);
    public static final Voltage LAUNCHING_FEEDER_VOLTAGE = Volts.of(12);
    public static final Voltage LAUNCHING_LAUNCHER_VOLTAGE = Volts.of(10.6);
    public static final Voltage SPIN_UP_FEEDER_VOLTAGE = Volts.of(-7);
    public static final Time SPIN_UP_TIME = Seconds.of(0.3);

    // kP: Proportional gain - voltage per RPM of error
    public static final double LAUNCHER_KP = 0.0001;

    // kI: Integral gain - voltage per RPM*second of accumulated error
    public static final double LAUNCHER_KI = 0.0;

    // kD: Derivative gain - voltage per (RPM/second) rate of change
    public static final double LAUNCHER_KD = 0.0;

    // Feedforward Gains
    // kS: Static gain - voltage to overcome friction (volts)
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
    public static final AngularVelocity LAUNCHER_TARGET_SPEED = RPM.of(3400); // Normal shot
    public static final AngularVelocity LAUNCHER_FAR_TARGET_SPEED = RPM.of(4000); // Far shot

    // Velocity tolerance in RPM
    // How close to target velocity before considering "at speed"
    public static final AngularVelocity LAUNCHER_VELOCITY_TOLERANCE = RPM.of(50);
  }

  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double DEADBAND = 0.1;
  }
}

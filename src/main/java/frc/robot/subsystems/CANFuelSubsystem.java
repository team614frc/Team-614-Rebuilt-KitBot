// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.FuelConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkFlex feederRoller = new SparkFlex(FEEDER_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex intakeLauncherRoller =
      new SparkFlex(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);

  // WPILib PID Controller for flywheel velocity control
  private final PIDController launcherPID;

  // SimpleMotorFeedforward for velocity feedforward
  private final SimpleMotorFeedforward launcherFeedforward;

  // Target velocity in RPM
  private double targetLauncherVelocity = 0.0;

  // Control mode flag
  private boolean useVelocityControl = false;

  /** Creates a new CANFuelSubsystem. */
  public CANFuelSubsystem() {
    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking Feeder Roller Value", INTAKING_FEEDER_VOLTAGE.in(Volts));
    SmartDashboard.putNumber("Intaking Intake Roller Value", INTAKING_INTAKE_VOLTAGE.in(Volts));
    SmartDashboard.putNumber("Launching Feeder Roller Value", LAUNCHING_FEEDER_VOLTAGE.in(Volts));
    SmartDashboard.putNumber(
        "Launching Launcher Roller Value", LAUNCHING_LAUNCHER_VOLTAGE.in(Volts));
    SmartDashboard.putNumber("Spin-up Feeder Roller Value", SPIN_UP_FEEDER_VOLTAGE.in(Volts));

    // PIDF tuning values - start with these and tune on the robot
    SmartDashboard.putNumber("Launcher kP", LAUNCHER_KP);
    SmartDashboard.putNumber("Launcher kI", LAUNCHER_KI);
    SmartDashboard.putNumber("Launcher kD", LAUNCHER_KD);
    SmartDashboard.putNumber("Launcher kS", LAUNCHER_KS);
    SmartDashboard.putNumber("Launcher kV", LAUNCHER_KV);
    SmartDashboard.putNumber("Target Launcher RPM", LAUNCHER_TARGET_SPEED.in(RPM));
    SmartDashboard.putNumber("Target Launcher Far RPM", LAUNCHER_FAR_TARGET_SPEED.in(RPM));

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    feederRoller.configure(
        Configs.FeederLauncherConfig.FEEDER_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    intakeLauncherRoller.configure(
        Configs.FeederLauncherConfig.LAUNCHER_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Initialize WPILib PID controller for launcher velocity control
    // PID works on error (target - current), outputs a motor command
    launcherPID = new PIDController(LAUNCHER_KP, LAUNCHER_KI, LAUNCHER_KD);

    // Set integral zone to prevent windup
    launcherPID.setIntegratorRange(-LAUNCHER_IZONE, LAUNCHER_IZONE);

    // Initialize feedforward for velocity control
    // kS = voltage to overcome static friction
    // kV = voltage per unit velocity (volts / (rotations/sec))
    launcherFeedforward = new SimpleMotorFeedforward(LAUNCHER_KS, LAUNCHER_KV);
  }

  /** Updates PIDF values from SmartDashboard. Call this to tune gains live. */
  public void updatePIDFFromDashboard() {
    double kP = SmartDashboard.getNumber("Launcher kP", LAUNCHER_KP);
    double kI = SmartDashboard.getNumber("Launcher kI", LAUNCHER_KI);
    double kD = SmartDashboard.getNumber("Launcher kD", LAUNCHER_KD);

    launcherPID.setPID(kP, kI, kD);
  }

  // A method to set the rollers to values for intaking
  public void intake() {
    useVelocityControl = false;
    feederRoller.setVoltage(
        SmartDashboard.getNumber(
            "Intaking Feeder Roller Value", INTAKING_FEEDER_VOLTAGE.in(Volts)));
    intakeLauncherRoller.setVoltage(
        SmartDashboard.getNumber(
            "Intaking Intake Roller Value", INTAKING_INTAKE_VOLTAGE.in(Volts)));
    targetLauncherVelocity = 0.0;
    launcherPID.reset();
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
    useVelocityControl = false;
    feederRoller.setVoltage(
        -1
            * SmartDashboard.getNumber(
                "Intaking Feeder Roller Value", INTAKING_FEEDER_VOLTAGE.in(Volts)));
    intakeLauncherRoller.setVoltage(
        -1
            * SmartDashboard.getNumber(
                "Intaking Launcher Roller Value", INTAKING_INTAKE_VOLTAGE.in(Volts)));
    targetLauncherVelocity = 0.0;
    launcherPID.reset();
  }

  // A method to set the rollers to values for launching using PIDF velocity control
  public void launch() {
    useVelocityControl = true;
    feederRoller.setVoltage(
        SmartDashboard.getNumber(
            "Launching Feeder Roller Value", LAUNCHING_FEEDER_VOLTAGE.in(Volts)));

    // Set target velocity for PID control
    targetLauncherVelocity =
        SmartDashboard.getNumber("Target Launcher RPM", LAUNCHER_TARGET_SPEED.in(RPM));
  }

  public void launchFar() {
    useVelocityControl = true;
    feederRoller.setVoltage(
        SmartDashboard.getNumber(
            "Launching Feeder Roller Value", LAUNCHING_FEEDER_VOLTAGE.in(Volts)));

    // Set target velocity for far shot
    targetLauncherVelocity =
        SmartDashboard.getNumber("Target Launcher Far RPM", LAUNCHER_FAR_TARGET_SPEED.in(RPM));
  }

  // A method to stop the rollers
  public void stop() {
    useVelocityControl = false;
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
    targetLauncherVelocity = 0.0;
    launcherPID.reset();
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher (using PIDF control)
  public void spinUp() {
    useVelocityControl = true;
    feederRoller.setVoltage(
        SmartDashboard.getNumber("Spin-up Feeder Roller Value", SPIN_UP_FEEDER_VOLTAGE.in(Volts)));

    // Set target velocity for PID control
    targetLauncherVelocity =
        SmartDashboard.getNumber("Target Launcher RPM", LAUNCHER_TARGET_SPEED.in(RPM));
  }

  public void spinUpFar() {
    useVelocityControl = true;
    feederRoller.setVoltage(
        SmartDashboard.getNumber("Spin-up Feeder Roller Value", SPIN_UP_FEEDER_VOLTAGE.in(Volts)));

    // Set target velocity for far shot
    targetLauncherVelocity =
        SmartDashboard.getNumber("Target Launcher Far RPM", LAUNCHER_FAR_TARGET_SPEED.in(RPM));
  }

  /** Returns true if the launcher is at the target velocity (within tolerance) */
  public boolean isLauncherAtSpeed() {
    if (targetLauncherVelocity == 0.0 || !useVelocityControl) {
      return true; // Not trying to spin up
    }
    double currentVelocity = getLauncherVelocity();
    double error = Math.abs(targetLauncherVelocity - currentVelocity);
    return error < LAUNCHER_VELOCITY_TOLERANCE.in(RPM);
  }

  /** Gets the current launcher velocity in RPM */
  public double getLauncherVelocity() {
    // SPARK Flex encoder returns velocity in RPM by default
    return intakeLauncherRoller.getEncoder().getVelocity();
  }

  /** Gets the target launcher velocity in RPM */
  public double getTargetLauncherVelocity() {
    return targetLauncherVelocity;
  }

  /** Gets the launcher velocity error in RPM */
  public double getLauncherVelocityError() {
    return targetLauncherVelocity - getLauncherVelocity();
  }

  // A command factory to turn the spinUp method into a command that requires this
  // subsystem
  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  public Command spinUpFarCommand() {
    return this.run(() -> spinUpFar());
  }

  // A command factory to turn the launch method into a command that requires this
  // subsystem
  public Command launchCommand() {
    return this.run(() -> launch());
  }

  public Command launchFarCommand() {
    return this.run(() -> launchFar());
  }

  public Command stopCommand() {
    return this.run(() -> stop());
  }

  public Command intakeCommand() {
    return this.run(() -> intake());
  }

  /** Command that waits for the launcher to reach target speed before finishing */
  public Command waitForLauncherSpeed() {
    return this.run(() -> {}).until(this::isLauncherAtSpeed).withName("WaitForLauncherSpeed");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // If using velocity control, calculate and apply PID + feedforward
    if (useVelocityControl && targetLauncherVelocity > 0) {
      // Get current velocity in RPM
      double currentVelocityRPM = getLauncherVelocity();

      // Convert RPM to rotations per second for feedforward
      double currentVelocityRPS = currentVelocityRPM / 60.0;
      double targetVelocityRPS = targetLauncherVelocity / 60.0;

      // Calculate PID output (this returns a voltage correction)
      double pidOutput = launcherPID.calculate(currentVelocityRPM, targetLauncherVelocity);

      // Calculate feedforward voltage
      double feedforwardOutput = launcherFeedforward.calculate(targetVelocityRPS);

      // Combine PID and feedforward
      double totalVoltage = pidOutput + feedforwardOutput;

      // Apply voltage to motor
      intakeLauncherRoller.setVoltage(totalVoltage);
    }

    // Log telemetry to SmartDashboard for debugging and tuning
    SmartDashboard.putNumber("Launcher Velocity (RPM)", getLauncherVelocity());
    SmartDashboard.putNumber("Launcher Target Velocity (RPM)", getTargetLauncherVelocity());
    SmartDashboard.putNumber("Launcher Velocity Error (RPM)", getLauncherVelocityError());
    SmartDashboard.putBoolean("Launcher At Speed", isLauncherAtSpeed());
    SmartDashboard.putNumber("Launcher Output Current", intakeLauncherRoller.getOutputCurrent());
    SmartDashboard.putNumber("Launcher Applied Output", intakeLauncherRoller.getAppliedOutput());
    SmartDashboard.putBoolean("Using Velocity Control", useVelocityControl);
  }
}

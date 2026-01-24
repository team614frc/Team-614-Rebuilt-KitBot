// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.FuelConstants.*;
import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.subsystems.VisionSubsystem.MAX_LINEAR_SPEED_MPS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final CANFuelSubsystem ballSubsystem = new CANFuelSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem(drivebase);

  // The driver's controller
  private final CommandXboxController driverXbox =
      new CommandXboxController(DRIVER_CONTROLLER_PORT);

  // The operator's controller
  private final CommandXboxController codriverXbox =
      new CommandXboxController(OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity.
   */
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> -driverXbox.getLeftY(),
              () -> -driverXbox.getLeftX())
          .withControllerRotationAxis(() -> -driverXbox.getRightX())
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.95)
          .allianceRelativeControl(true);

  SwerveInputStream driveAngularVelocityKeyboard =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> -driverXbox.getLeftY(),
              () -> -driverXbox.getLeftX())
          .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.95)
          .allianceRelativeControl(true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    NamedCommands.registerCommand(
        "Launch",
        ballSubsystem
            .spinUpCommand()
            .withTimeout(SPIN_UP_SECONDS)
            .andThen(ballSubsystem.launchCommand())
            .withTimeout(5.85)
            .finallyDo(() -> ballSubsystem.stop()));
    NamedCommands.registerCommand(
        "4Launch",
        ballSubsystem
            .spinUpCommand()
            .withTimeout(SPIN_UP_SECONDS)
            .andThen(ballSubsystem.launchCommand())
            .withTimeout(4)
            .finallyDo(() -> ballSubsystem.stop()));
    NamedCommands.registerCommand("Stop", ballSubsystem.stopCommand());
    NamedCommands.registerCommand("Intake", ballSubsystem.intakeCommand());

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/ {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    // While the left bumper on driver controller is held, intake Fuel
    driverXbox
        .leftBumper()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop()));
    // While the right bumper on the driver controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    driverXbox
        .rightBumper()
        .whileTrue(
            ballSubsystem
                .spinUpCommand()
                .withTimeout(SPIN_UP_SECONDS)
                .andThen(ballSubsystem.launchCommand())
                .finallyDo(() -> ballSubsystem.stop()));

    driverXbox
        .rightTrigger()
        .whileTrue(
            ballSubsystem
                .spinUpFarCommand()
                .withTimeout(SPIN_UP_SECONDS)
                .andThen(ballSubsystem.launchFarCommand())
                .finallyDo(() -> ballSubsystem.stop()));

    driverXbox
        .a()
        .whileTrue(
            vision.rotateToAllianceTagWhileDriving(
                () -> driverXbox.getLeftY() * MAX_LINEAR_SPEED_MPS,
                () -> driverXbox.getLeftX() * MAX_LINEAR_SPEED_MPS));

    // // While the A button is held on the driver controller, eject fuel back out
    // // the intake
    // driverXbox
    //     .a()
    //     .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.eject(), () ->
    // ballSubsystem.stop()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}

package frc.robot;

import static edu.wpi.first.units.Units.Amp;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public final class Configs {

  public static final class FeederLauncherConfig {
    public static final SparkFlexConfig FEEDER_CONFIG = new SparkFlexConfig();
    public static final SparkFlexConfig LAUNCHER_CONFIG = new SparkFlexConfig();

    static {
      FEEDER_CONFIG
          .idleMode(IdleMode.kBrake)
          .inverted(true)
          .smartCurrentLimit((int) Constants.FuelConstants.FEEDER_MOTOR_CURRENT_LIMIT.in(Amp));
    }

    static {
      LAUNCHER_CONFIG
          .idleMode(IdleMode.kBrake)
          .inverted(true)
          .smartCurrentLimit((int) Constants.FuelConstants.LAUNCHER_MOTOR_CURRENT_LIMIT.in(Amp));
    }
  }
}

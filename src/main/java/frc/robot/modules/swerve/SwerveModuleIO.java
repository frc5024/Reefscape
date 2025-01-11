package frc.robot.modules.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * 
 */
public interface SwerveModuleIO {
  @AutoLog
  public static class SwerveModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public boolean turnConnected = false;
    public boolean turnEncoderConnected = false;
    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** 
   * Updates the set of loggable inputs. 
   */
  public default void updateInputs(SwerveModuleIOInputs inputs) {}

  /** 
   * Run the drive motor at the specified open loop value.
   */
  public default void setDriveOpenLoop(double output) {}

  /** 
   * Run the turn motor at the specified open loop value. 
   */
  public default void setTurnOpenLoop(double output) {}

  /** 
   * Run the drive motor at the specified velocity. 
   */
  public default void setDriveVelocity(double velocityRadPerSec) {}

  /**
   * Run the turn motor to the specified rotation. 
   */
  public default void setTurnPosition(Rotation2d rotation) {}
}

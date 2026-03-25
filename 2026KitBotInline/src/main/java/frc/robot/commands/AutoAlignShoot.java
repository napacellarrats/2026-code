package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.ALIGNMENT_P;
import static frc.robot.Constants.FuelConstants.DEFAULT_SHOOTER_RANGE_METERS;
import static frc.robot.Constants.FuelConstants.DEFAULT_SHOOTER_RPS;
import static frc.robot.Constants.FuelConstants.SHOOTER_MAX_ALIGN_TURN;
import static frc.robot.Constants.FuelConstants.SHOOTER_TX_TOLERANCE_DEGREES;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

public class AutoAlignShoot extends Command {
  private static final String LIMELIGHT_NAME = "limelight";

  private final CANDriveSubsystem driveSubsystem;
  private final CANFuelSubsystem fuelSubsystem;
  private final PIDController txController = new PIDController(ALIGNMENT_P, 0, 0);

  public AutoAlignShoot(CANDriveSubsystem driveSubsystem, CANFuelSubsystem fuelSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.fuelSubsystem = fuelSubsystem;

    txController.setSetpoint(0.0);
    txController.setTolerance(SHOOTER_TX_TOLERANCE_DEGREES);

    addRequirements(driveSubsystem, fuelSubsystem);
  }

  @Override
  public void initialize() {
    txController.reset();
  }

  @Override
  public void execute() {
    boolean hasTarget = LimelightHelpers.getTV(LIMELIGHT_NAME);
    double requestedRps = DEFAULT_SHOOTER_RPS;
    double tx = 0.0;

    if (hasTarget) {
      tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
      double rangeMeters = getBestRangeMeters();
      requestedRps = fuelSubsystem.getInterpolatedShooterRps(rangeMeters);

      double turnCommand = MathUtil.clamp(
          txController.calculate(tx),
          -SHOOTER_MAX_ALIGN_TURN,
          SHOOTER_MAX_ALIGN_TURN);
      driveSubsystem.turnInPlace(turnCommand);

      SmartDashboard.putNumber("Auto Shoot Range Meters", rangeMeters);
    } else {
      driveSubsystem.stopDrive();
    }

    fuelSubsystem.spinUpToRps(requestedRps);

    boolean aligned = hasTarget && Math.abs(tx) <= SHOOTER_TX_TOLERANCE_DEGREES;
    if (aligned && fuelSubsystem.isShooterAtSetpoint()) {
      fuelSubsystem.launch();
    }

    SmartDashboard.putBoolean("Auto Shoot Has Target", hasTarget);
    SmartDashboard.putNumber("Auto Shoot Target RPS", requestedRps);
    SmartDashboard.putNumber("Auto Shoot TX", tx);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stopDrive();
    fuelSubsystem.stop(true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private double getBestRangeMeters() {
    LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(LIMELIGHT_NAME);

    double bestRangeMeters = Double.POSITIVE_INFINITY;
    for (LimelightHelpers.RawFiducial fiducial : fiducials) {
      double candidateRange = fiducial.distToRobot > 0.0 ? fiducial.distToRobot : fiducial.distToCamera;
      if (candidateRange > 0.0) {
        bestRangeMeters = Math.min(bestRangeMeters, candidateRange);
      }
    }

    if (Double.isFinite(bestRangeMeters)) {
      return bestRangeMeters;
    }

    return DEFAULT_SHOOTER_RANGE_METERS;
  }
}

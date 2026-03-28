package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.ALIGNMENT_D;
import static frc.robot.Constants.DriveConstants.ALIGNMENT_P;
import static frc.robot.Constants.FuelConstants.DEFAULT_SHOOTER_RANGE_METERS;
import static frc.robot.Constants.FuelConstants.DEFAULT_SHOOTER_RPS;
import static frc.robot.Constants.FuelConstants.SHOOTER_MAX_ALIGN_TURN;
import static frc.robot.Constants.FuelConstants.SHOOTER_READY_HOLD_SECONDS;
import static frc.robot.Constants.FuelConstants.SHOOTER_TX_TOLERANCE_DEGREES;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

public class AutoAlignShoot extends Command {
  private static final String LIMELIGHT_NAME = "limelight";

  private final CANDriveSubsystem driveSubsystem;
  private final CANFuelSubsystem fuelSubsystem;
  private final PIDController txController = new PIDController(ALIGNMENT_P, 0, ALIGNMENT_D);
  private final Timer readyToFeedTimer = new Timer();

  private boolean wasFeeding = false;

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
    readyToFeedTimer.stop();
    readyToFeedTimer.reset();
    wasFeeding = false;
  }

  @Override
  public void execute() {
    boolean hasTarget = LimelightHelpers.getTV(LIMELIGHT_NAME);
    double requestedRps = DEFAULT_SHOOTER_RPS;
    double tx = 0.0;
    double rangeMeters = DEFAULT_SHOOTER_RANGE_METERS;

    if (hasTarget) {
      tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
      rangeMeters = getBestRangeMeters();
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

    //fuelSubsystem.spinUpToRps(requestedRps);

    boolean aligned = hasTarget && Math.abs(tx) <= SHOOTER_TX_TOLERANCE_DEGREES;
    boolean readyToFeed = aligned && fuelSubsystem.isShooterAtSetpoint();
    updateReadyToFeedTimer(readyToFeed);
    boolean feedAllowed = false;

    if (feedAllowed) {
      fuelSubsystem.launch();
      if (!wasFeeding) {
        System.out.printf(
            "AutoShoot feed start range=%.2f targetRPS=%.1f actualRPS=%.1f tx=%.2f",
            rangeMeters,
            requestedRps,
            fuelSubsystem.getShooterVelocityRps(),
            tx);
        System.out.println();
      }
    }
    wasFeeding = feedAllowed;

    SmartDashboard.putBoolean("Auto Shoot Has Target", hasTarget);
    SmartDashboard.putNumber("Auto Shoot Target RPS", requestedRps);
    SmartDashboard.putNumber("Auto Shoot Actual RPS", fuelSubsystem.getShooterVelocityRps());
    SmartDashboard.putNumber("Auto Shoot TX", tx);
    SmartDashboard.putBoolean("Auto Shoot Ready To Feed", readyToFeed);
    SmartDashboard.putBoolean("Auto Shoot Feed Allowed", feedAllowed);
    SmartDashboard.putNumber("Auto Shoot Ready Hold Seconds", readyToFeedTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stopDrive();
    fuelSubsystem.stop(true);
    System.out.println("AutoShoot end interrupted=" + interrupted);
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

  private void updateReadyToFeedTimer(boolean readyToFeed) {
    if (readyToFeed) {
      if (!readyToFeedTimer.isRunning()) {
        readyToFeedTimer.restart();
      }
      return;
    }

    readyToFeedTimer.stop();
    readyToFeedTimer.reset();
  }
}

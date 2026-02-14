package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;

public class AlignToHub extends Command {
    private PIDController xController, yController;
    private Timer dontSeeTagTimer, stopTimer;
    private double tagID = -1;
    private CANDriveSubsystem driveSubsystem;

    public AlignToHub(CANDriveSubsystem driveSubsystem) {
        xController = new PIDController(DriveConstants.X_ALIGNMENT_P, 0, 0);
        yController = new PIDController(DriveConstants.Y_ALIGNMENT_P, 0, 0);
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();

        xController.setSetpoint(DriveConstants.X_SETPOINT_ALIGNMENT);
        xController.setTolerance(DriveConstants.X_TOLERANCE_ALIGNMENT);

        yController.setSetpoint(DriveConstants.Y_SETPOINT_ALIGNMENT);
        yController.setTolerance(DriveConstants.Y_TOLERANCE_ALIGNMENT);

        if (LimelightHelpers.getTV("")) {
            tagID = LimelightHelpers.getFiducialID("");
        }
    }

    public void execute() {
        if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
            this.dontSeeTagTimer.reset();

            double[] positions = LimelightHelpers.getBotPose_TargetSpace("");
            double xSpeed = xController.calculate(positions[0]);
            double ySpeed = -yController.calculate(positions[2]);

            if (yController.getError() < DriveConstants.Y_TOLERANCE_ALIGNMENT) {
                driveSubsystem.driveArcade(() -> ySpeed, () -> xSpeed);
            }

            if (!yController.atSetpoint() || !xController.atSetpoint()) {
                stopTimer.reset();
            }
        } else {
            driveSubsystem.driveArcade(() -> 0.0, () -> 0.0);
        }
    }

    public boolean isFinished() {
        return this.dontSeeTagTimer.hasElapsed(DriveConstants.DONT_SEE_TAG_WAIT_TIME)
                || stopTimer.hasElapsed(DriveConstants.POSE_VALIDATION_TIME);
    }
}

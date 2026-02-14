package frc;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CANDriveSubsystem;

public class LimelightAlign {
    public final Command limelightShootAlign(double steering_adjust, double distance_adjust) {
        double KpAim = -0.1;
        double KpDistance = -0.1;
        double min_aim_command = 0.05;

        double tx = LimelightHelpers.getTX("");
        double ty = LimelightHelpers.getTY("");
        boolean hasTarget = LimelightHelpers.getTV("");

        steering_adjust = 0.0;

        if (tx > 1.0) {
            steering_adjust = KpAim * -tx - min_aim_command;
        } else if (tx < -1.0) {
            steering_adjust = KpAim * -tx + min_aim_command;
        }

        distance_adjust = KpDistance * -ty;

        return new SequentialCommandGroup(

        );
    }
}

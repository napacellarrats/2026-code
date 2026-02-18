package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANDriveSubsystem;

public class LimelightAlign {
    public final Double[] limelightShootAlign() {
        double KpAim = -0.1;
        double KpDistance = -0.1;
        double min_aim_command = 0.05;

        double steering_adjust = 0.0;
        double distance_adjust = 0.0;

        double tx = LimelightHelpers.getTX("");
        double ty = LimelightHelpers.getTY("");
        boolean hasTarget = LimelightHelpers.getTV("");

        steering_adjust = 0.0;

        System.out.println( tx);
        System.out.print("kpaim* tx: ");
        System.out.println(KpAim * tx);

        if (tx > 1.0 && (KpAim * tx) +min_aim_command < 0) {
            steering_adjust = -0.5;
        } else if (tx < -1.0 && (KpAim * tx) -min_aim_command > 0) {
            steering_adjust = 0.5;
        }

        distance_adjust = KpDistance * -ty;

        return new Double[] { distance_adjust, steering_adjust };
    }
}
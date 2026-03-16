package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.variables;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class SpeedControl extends Command {
    private final double normalSpeed = 0.7;
    private final double slowSpeed = 0.3;
    public SpeedControl() {
    }

    public Command runEnd(Runnable run, Runnable end) {
        return new FunctionalCommand(() -> {}, run, interrupted -> end.run(), () -> false, null);
    }



    public void scaleSpeed(boolean slow) {
        if (slow) {
            variables.scaling = this.slowSpeed;
        } else {
            variables.scaling = this.normalSpeed;
        }
    }
}

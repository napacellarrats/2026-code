// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.FuelConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;

public final class Autos {
  public static final Command auto(CANFuelSubsystem ballSubsystem) {
    return new SequentialCommandGroup(ballSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS)
            .andThen(ballSubsystem.launchCommand().withTimeout(5))
            .finallyDo(() -> ballSubsystem.stop(true)));
  }
}

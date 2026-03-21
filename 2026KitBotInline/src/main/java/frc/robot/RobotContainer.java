// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static frc.robot.Constants.OperatorConstants.*;

import java.net.CookieManager;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;

import static frc.robot.Constants.FuelConstants.*;

import frc.robot.LimelightAlign.*;
import frc.robot.commands.AlignToHub;
import frc.robot.commands.Autos;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RuntimeType;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
    private final CANFuelSubsystem ballSubsystem = new CANFuelSubsystem();

    // Limelight
    // private final LimelightAlign limelightAlign = new LimelightAlign();
    // private final Double yCorrection = 0.0;
    // private final Double xCorrection = 0.0;

    // The driver's controller
    public final Joystick driverController = new Joystick(
            DRIVER_CONTROLLER_PORT);

    // The operator's controller
    private final Joystick operatorController = new Joystick(
            OPERATOR_CONTROLLER_PORT);

    // The autonomous chooser
    private final SendableChooser<Command> autoChooser;

    // In order to align using limelight

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureBindings();

        // Set the options to show up in the Dashboard for selecting auto modes. If you
        // add additional auto modes you can add additional lines here with
        autoChooser = AutoBuilder.buildAutoChooser();
        // autoChooser.addOption
        // autoChooser.setDefaultOption("Drive and shoot from middle",
        // Autos.driveShootMiddle(driveSubsystem, ballSubsystem));
        // autoChooser.addOption("Drive and shoot from left",
        // Autos.driveShootLeft(driveSubsystem, ballSubsystem));
        autoChooser.addOption("Drive and shoot from right",
                Autos.auto(ballSubsystem));
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
     * constructor with an arbitrary predicate, or via the named factories in
     * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
     * for {@link CommandXboxController Xbox}/0-
     * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {

        // While the left bumper on operator controller is held, intake Fuel
        new JoystickButton(driverController, 2)
                .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop(false)));

        new JoystickButton(operatorController, 2)
                .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop(false)));

        // While the right bumper on the operator controller is held, spin up for 1
        // second, then launch fuel. When the button is released, stop.

        new JoystickButton(driverController, 1)
                .whileTrue(ballSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS)
                        .andThen(ballSubsystem.launchCommand())
                        .finallyDo(() -> ballSubsystem.stop(true)));
        // While the A button is held on the operator controller, eject fuel back out
        // the intake
        new JoystickButton(driverController, 3)
                .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.eject(), () -> ballSubsystem.stop(false)));

        // Auto-alignment for shooting
        new JoystickButton(driverController, 4)
                .onTrue(new AlignToHub(driveSubsystem).withTimeout(3));

        // Attempt to unjam
        new JoystickButton(driverController, 5)
                .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.unjam(), () -> ballSubsystem.stop(false)));

        // Slow mode
        // new JoystickButton(driverController, 11)
        // .whileTrue(Commands.runEnd(() -> variables.scaling = 0.3, () ->
        // variables.scaling = 0.5));

        // // Fast mode
        // new JoystickButton(driverController, 9)
        // .whileTrue(Commands.runEnd(() -> variables.scaling = 0.7, () ->
        // variables.scaling = 0.5));

        // Tests
        new JoystickButton(driverController, 12)
                .whileTrue(Commands.runOnce(SignalLogger::start));

        new JoystickButton(driverController, 11)
                .whileTrue(Commands.runOnce(SignalLogger::stop));

        new JoystickButton(driverController, 10)
                .whileTrue(driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

        new JoystickButton(driverController, 9)
                .whileTrue(driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        new JoystickButton(driverController, 8)
                .whileTrue(driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));

        new JoystickButton(driverController, 7)
                .whileTrue(driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Set the default command for the drive subsystem to the command provided by
        // factory with the values provided by the joystick axes on the driver
        // controller. The Y axis of the controller is inverted so that pushing the
        // stick away from you (a negative value) drives the robot forwards (a positive
        // value). The X-axis is also inverted so a positive value (stick to the right)
        // results in clockwise rotation (front of the robot turning right). Both axes
        // are also scaled down so the rotation is more easily controllable.

        driveSubsystem.setDefaultCommand(
                driveSubsystem.run(() -> driveSubsystem.arcadeDrive(-Math.pow(2, -driverController.getY()),
                        ((-Math.pow(2, driverController.getX())) - (Math.pow(2, driverController.getZ()))))));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * () -> -driverController.getY() * DRIVE_SCALING,
     * () -> - driverController.getZ() * ROTATION_SCALING
     * 
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Command to run in auto
        return autoChooser.getSelected();
    }
}

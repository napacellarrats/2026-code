// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterLookupPoint;
import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private final CANBus kCanBus = new CANBus("rio");

  private final TalonFX feederMotor = new TalonFX(FEEDER_MOTOR_ID, kCanBus);
  private final TalonFX launcherIntakeMotor = new TalonFX(INTAKE_LAUNCHER_MOTOR_ID, kCanBus);

  private final DutyCycleOut feederout = new DutyCycleOut(0);
  private final VelocityVoltage launcherVelocityRequest = new VelocityVoltage(0);
  private final InterpolatingDoubleTreeMap shooterRpsByRangeMeters = new InterpolatingDoubleTreeMap();

  private double shooterTargetRps = 0.0;

  public CANFuelSubsystem() {
    var feederConfiguration = new TalonFXConfiguration();
    var launcherIntakeConfiguration = new TalonFXConfiguration();

    feederConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    feederConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.25;
    launcherIntakeConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    launcherIntakeConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;
    launcherIntakeConfiguration.Slot0.kP = SHOOTER_KP;
    launcherIntakeConfiguration.Slot0.kS = SHOOTER_KS;
    launcherIntakeConfiguration.Slot0.kV = SHOOTER_KV;

    feederMotor.getConfigurator().apply(feederConfiguration);
    launcherIntakeMotor.getConfigurator().apply(launcherIntakeConfiguration);

    for (ShooterLookupPoint point : SHOOTER_LOOKUP_POINTS) {
      shooterRpsByRangeMeters.put(point.rangeMeters(), point.shooterRps());
    }
  }

  public void launch() {
    feederout.Output = LAUNCHING_FEEDER_VALUE;
    feederMotor.setControl(feederout);
    runShooterAtTarget();
  }

  public void spinDown() {
    shooterTargetRps = 0.0;
    feederMotor.stopMotor();
    launcherIntakeMotor.stopMotor();
  }

  public void intake() {
    feederout.Output = INTAKING_FEEDER_VALUE;
    feederMotor.setControl(feederout);
    launcherIntakeMotor.setControl(new DutyCycleOut(INTAKING_INTAKE_VALUE));
  }

  public void eject() {
    feederout.Output = -INTAKING_FEEDER_VALUE;
    feederMotor.setControl(feederout);
    launcherIntakeMotor.setControl(new DutyCycleOut(-INTAKING_INTAKE_VALUE));
  }

  public void stop(boolean shoot) {
    spinDown();
  }

  public void unjam() {
    feederout.Output = SPIN_UP_FEEDER_VALUE;
    feederMotor.setControl(feederout);
    launcherIntakeMotor.setControl(new DutyCycleOut(-LAUNCHING_LAUNCHER_VALUE));
  }

  public void spinUp() {
    spinUpToRps(DEFAULT_SHOOTER_RPS);
  }

  public void spinUpToRps(double targetRps) {
    shooterTargetRps = Math.max(0.0, targetRps);
    feederout.Output = SPIN_UP_FEEDER_VALUE;
    feederMotor.setControl(feederout);
    runShooterAtTarget();
  }

  public void holdShooterAtRps(double targetRps) {
    shooterTargetRps = Math.max(0.0, targetRps);
    feederMotor.stopMotor();
    runShooterAtTarget();
  }

  public void holdFeeder() {
    feederout.Output = SPIN_UP_FEEDER_VALUE;
    feederMotor.setControl(feederout);
  }

  public double getInterpolatedShooterRps(double rangeMeters) {
    return shooterRpsByRangeMeters.get(rangeMeters);
  }

  public double getShooterVelocityRps() {
    return Math.abs(launcherIntakeMotor.getVelocity().getValueAsDouble());
  }

  public double getShooterTargetRps() {
    return shooterTargetRps;
  }

  public boolean isShooterAtSetpoint() {
    return Math.abs(getShooterVelocityRps() - shooterTargetRps) <= SHOOTER_RPS_TOLERANCE;
  }

  private void runShooterAtTarget() {
    launcherIntakeMotor.setControl(
        launcherVelocityRequest.withVelocity(SHOOTER_DIRECTION_SIGN * shooterTargetRps));
  }

  public Command ejectCommand() {
    return this.run(() -> eject());
  }

  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  public Command launchCommand() {
    return this.run(() -> launch());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Target RPS", shooterTargetRps);
    SmartDashboard.putNumber("Shooter Actual RPS", getShooterVelocityRps());
    SmartDashboard.putBoolean("Shooter At Setpoint", isShooterAtSetpoint());
  }
}
/*
 * public class CANFuel1Subsystem extends SubsystemBase {
 * private final SparkMax feederRoller;
 * private final SparkMax intakeLauncherRoller;
 * 
 * // Creates a new CANBallSubsystem.
 * public CANFuel1Subsystem() {
 * // create brushed motors for each of the motors on the launcher mechanism
 * intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID,
 * MotorType.kBrushed);
 * feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushed);
 * 
 * // put default values for various fuel operations onto the dashboard
 * // all methods in this subsystem pull their values from the dashbaord to
 * allow
 * // you to tune the values easily, and then replace the values in
 * Constants.java
 * // with your new values. For more information, see the Software Guide.
 * SmartDashboard.putNumber("Intaking feeder roller value",
 * INTAKING_FEEDER_VOLTAGE);
 * SmartDashboard.putNumber("Intaking intake roller value",
 * INTAKING_INTAKE_VOLTAGE);
 * SmartDashboard.putNumber("Launching feeder roller value",
 * LAUNCHING_FEEDER_VOLTAGE);
 * SmartDashboard.putNumber("Launching launcher roller value",
 * LAUNCHING_LAUNCHER_VOLTAGE);
 * SmartDashboard.putNumber("Spin-up feeder roller value",
 * SPIN_UP_FEEDER_VOLTAGE);
 * 
 * // create the configuration for the feeder roller, set a current limit and
 * apply
 * // the config to the controller
 * SparkMaxConfig feederConfig = new SparkMaxConfig();
 * feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
 * feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters,
 * PersistMode.kPersistParameters);
 * 
 * // create the configuration for the launcher roller, set a current limit, set
 * // the motor to inverted so that positive values are used for both intaking
 * and
 * // launching, and apply the config to the controller
 * SparkMaxConfig launcherConfig = new SparkMaxConfig();
 * launcherConfig.inverted(true);
 * launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
 * intakeLauncherRoller.configure(launcherConfig,
 * ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 * }
 * 
 * // A method to set the rollers to values for intaking
 * public void intake() {
 * feederRoller.setVoltage(SmartDashboard.
 * getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
 * intakeLauncherRoller
 * .setVoltage(SmartDashboard.getNumber("Intaking intake roller value",
 * INTAKING_INTAKE_VOLTAGE));
 * }
 * 
 * // A method to set the rollers to values for ejecting fuel out the intake.
 * Uses
 * // the same values as intaking, but in the opposite direction.
 * public void eject() {
 * feederRoller
 * .setVoltage(-1 * SmartDashboard.getNumber("Intaking feeder roller value",
 * INTAKING_FEEDER_VOLTAGE));
 * intakeLauncherRoller
 * .setVoltage(-1 * SmartDashboard.getNumber("Intaking launcher roller value",
 * INTAKING_INTAKE_VOLTAGE));
 * }
 * 
 * // A method to set the rollers to values for launching.
 * public void launch() {
 * feederRoller.setVoltage(SmartDashboard.
 * getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
 * intakeLauncherRoller
 * .setVoltage(SmartDashboard.getNumber("Launching launcher roller value",
 * LAUNCHING_LAUNCHER_VOLTAGE));
 * }
 * 
 * // A method to stop the rollers
 * public void stop() {
 * feederRoller.set(0);
 * intakeLauncherRoller.set(0);
 * }
 * 
 * // A method to spin up the launcher roller while spinning the feeder roller
 * to
 * // push Fuel away from the launcher
 * public void spinUp() {
 * feederRoller
 * .setVoltage(SmartDashboard.getNumber("Spin-up feeder roller value",
 * SPIN_UP_FEEDER_VOLTAGE));
 * intakeLauncherRoller
 * .setVoltage(SmartDashboard.getNumber("Launching launcher roller value",
 * LAUNCHING_LAUNCHER_VOLTAGE));
 * }
 * 
 * // A command factory to turn the spinUp method into a command that requires
 * this
 * // subsystem
 * public Command spinUpCommand() {
 * return this.run(() -> spinUp());
 * }
 * 
 * // A command factory to turn the launch method into a command that requires
 * this
 * // subsystem
 * public Command launchCommand() {
 * return this.run(() -> launch());
 * }
 * 
 * @Override
 * public void periodic() {
 * // This method will be called once per scheduler run
 * }
 * }
 */

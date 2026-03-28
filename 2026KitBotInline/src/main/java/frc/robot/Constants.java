// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public record ShooterLookupPoint(double rangeMeters, double shooterRps) {}

  public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 3;
    public static final int LEFT_FOLLOWER_ID = 4;
    public static final int RIGHT_LEADER_ID = 1;
    public static final int RIGHT_FOLLOWER_ID = 2;

    public static final double DRIVE_GEAR_RATIO = 8.450;

    // IMU ID
    public static final int PIGEON2_ID = 7;

    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;

    // Alignment values
    public static final double ALIGNMENT_P = 0.05;
    public static final double ALIGNMENT_S = 0.0;
    public static final double ALIGNMENT_V = 0.0;
    public static final double ALIGNMENT_A = 0.0;
    public static final double ALIGNMENT_D = 0.01;
    public static final double X_SETPOINT_ALIGNMENT = 0.0;
    public static final double Y_SETPOINT_ALIGNMENT = 4.0;
    public static final double X_TOLERANCE_ALIGNMENT = 0.5;
    public static final double Y_TOLERANCE_ALIGNMENT = 0.5;
    public static final double DONT_SEE_TAG_WAIT_TIME = 1.0;
    public static final double POSE_VALIDATION_TIME = 3.0;


  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int FEEDER_MOTOR_ID = 6;
    public static final int INTAKE_LAUNCHER_MOTOR_ID = 5;
    // 01/27/2026 6 is PR, 5 is PL
    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double INTAKING_FEEDER_VALUE = 0.7;
    public static final double INTAKING_INTAKE_VALUE = -0.5;
    public static final double LAUNCHING_FEEDER_VALUE = -0.6;
    public static final double LAUNCHING_LAUNCHER_VALUE = -0.65;
    public static final double SPIN_UP_FEEDER_VALUE = 0.7;
    public static final double SPIN_UP_SECONDS = 0.5;

    public static final double SHOOTER_KP = 0.18;
    public static final double SHOOTER_KS = 0.0;
    public static final double SHOOTER_KV = 0.113;
    public static final double SHOOTER_DIRECTION_SIGN = -1.0;
    public static final double DEFAULT_SHOOTER_RPS = 68.0;
    public static final double DEFAULT_SHOOTER_RANGE_METERS = 2.5;
    public static final double SHOOTER_RPS_TOLERANCE = 3.0;
    public static final double SHOOTER_TX_TOLERANCE_DEGREES = 1.0;
    public static final double SHOOTER_MAX_ALIGN_TURN = 0.5;
    public static final double SHOOTER_READY_HOLD_SECONDS = 0.15;
    public static final ShooterLookupPoint[] SHOOTER_LOOKUP_POINTS = {
        new ShooterLookupPoint(1.5, 62.0),
        new ShooterLookupPoint(2.0, 65.0),
        new ShooterLookupPoint(2.5, 68.0),
        new ShooterLookupPoint(3.0, 71.0),
        new ShooterLookupPoint(3.5, 74.0)
    };
  }

  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // Input shaping values for teleop drive. A higher cubic weight makes the
    // controls softer near center while still reaching full output at full stick.
    public static final double DRIVE_INPUT_DEADBAND = 0.08;
    public static final double ROTATION_INPUT_DEADBAND = 0.08;
    public static final double DRIVE_INPUT_CUBIC_WEIGHT = 0.75;
    public static final double ROTATION_INPUT_CUBIC_WEIGHT = 0.6;

    // This value is multiplied by the joystick value when driving the robot to
    // help avoid driving and turning too fast and being difficult to control
    public static final double DRIVE_SCALING = .7;
    public static final double SLOW_SCALING = .3;
    public static final double Z_ROTATION_SCALING = .7;
  }
}

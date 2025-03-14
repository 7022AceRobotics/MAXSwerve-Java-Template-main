// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds. 4.5 is max
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAngularSpeed = 2*Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24);
    // Distance between front and back wheels on robot

    // Distance between centers of right and left wheels for new robot
    //public static final double kWheelBase = Units.inchesToMeters(21.4);
    // Distance between centers of right and left wheels for new robot
    //public static final double kWheelBase = Units.inchesToMeters(21.4);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    // public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    // public static final double kFrontRightChassisAngularOffset = 0;
    // public static final double kBackLeftChassisAngularOffset = Math.PI;
    // public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final double kFrontLeftChassisAngularOffset = -Math.PI;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = -Math.PI;
    public static final double kBackRightChassisAngularOffset = 0;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 2;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 7;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    // Note that if kMaxSpeedMetersPerSecond will only be used if the speed of the robot during auto is greater than kMaxSpeedMetersPerSecond
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2 * Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;

    public static final double kIXController = 0;
    public static final double kIYController = 0;

    public static final double kPThetaController = 1;
    public static final double kIThetaController = 0;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
  public static final class ElevatorConstants {
    public static final int ElevatorMotorPort = 12;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kIz = 0;
    public static final double kFF = 0;
    public static final double kMaxOutput = 0;
    public static final double kMinOutput = 0;
    public static final double maxVel = 0;
    public static final double maxAccel = 0;
    public static final double allowedErr = 0;
    //Elevator Positions has to be positive
    public static final double Stage1Posistion = 0;
    public static final double Stage2Posistion = 0;
    public static final double Stage3Posistion = 0;
    public static final double Stage4Posistion = 0;
  }
  public static final class AlgeaConstants {
    public static final int AlgeaPivotMotorPort = 11;
    public static final int AlgeaPullMotorPort = 10;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kIz = 0;
    public static final double kFF = 0;
    public static final double kMaxOutput = 0;
    public static final double kMinOutput = 0;
    public static final double maxVel = 0;
    public static final double maxAccel = 0;
    public static final double allowedErr = 0;
    //Elevator Positions has to be positive
    public static final double OutPosistion = 0;
    public static final double InPosistion = 0;
  }

  public static final class MicrosoftCameraConstants{
    public static final AprilTagFieldLayout map = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    public static final Transform3d camToRobot = new Transform3d(new Translation3d(0, 0, Units.inchesToMeters(40)), new Rotation3d(0,0,Math.PI));
    
  }
}

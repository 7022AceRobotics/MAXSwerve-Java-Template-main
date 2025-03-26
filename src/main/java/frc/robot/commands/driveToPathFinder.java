// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class driveToPathFinder extends Command {
  /** Creates a new driveToPathFinder. */
  private DriveSubsystem m_drive_subsystem;

  private Pose2d pose_initial;
  private Pose2d pose_middle;
  private Pose2d pose_final;

  private int id;

  private AprilTagFieldLayout map = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);


  public driveToPathFinder() {
    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d position_of_apriltag = map.getTagPose(id).get().toPose2d();
    pose_initial = m_drive_subsystem.m_swerve_drive_pose_estimator.getEstimatedPosition();

    pose_middle = position_of_apriltag.rotateBy(position_of_apriltag.getRotation().times(-1))
    .transformBy(new Transform2d(DriveConstants.kWheelBase/2 + 0.1, -0.6, new Rotation2d(0)))
    .rotateBy(position_of_apriltag.getRotation())
    .rotateBy(new Rotation2d(Math.PI))
    .times(-1);

    pose_final = position_of_apriltag.rotateBy(position_of_apriltag.getRotation().times(-1))
    .transformBy(new Transform2d(DriveConstants.kWheelBase/2 + Units.inchesToMeters(7), 0.3, new Rotation2d(0)))
    .rotateBy(position_of_apriltag.getRotation())
    .rotateBy(new Rotation2d(Math.PI))
    .times(-1);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

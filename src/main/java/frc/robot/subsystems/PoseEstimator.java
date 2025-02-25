// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimator extends SubsystemBase {
  /** Creates a new PoseEstimator. */
  final private SwerveDrivePoseEstimator m_swerve_drive_pose_estimator;
  final private SwerveDriveKinematics m_swerve_drive_kinematics;
  final private Rotation2d m_robot_drive;
  final private SwerveModulePosition [] m_swerve_module_positions;
  private final Pose2d m_pose;

  public PoseEstimator(SwerveDriveKinematics m_swerve_drive_kinematics, Rotation2d m_robot_drive, SwerveModulePosition[] m_swerve_module_positions, Pose2d m_pose) {
    this.m_swerve_drive_kinematics = m_swerve_drive_kinematics;
    this.m_robot_drive = m_robot_drive;
    this.m_swerve_module_positions = m_swerve_module_positions;
    this.m_pose = m_pose;
    this.m_swerve_drive_pose_estimator = new SwerveDrivePoseEstimator(m_swerve_drive_kinematics, m_robot_drive, m_swerve_module_positions, m_pose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

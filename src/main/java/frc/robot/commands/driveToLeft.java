// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.nio.file.FileSystem;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class driveToLeft extends Command {
  /** Creates a new driveToLeft. */
  private DriveSubsystem m_drive_subsystem;
  private LimelightHelpers m_limelight_subsystem;
  private SwerveControllerCommand swerveControllerCommand;
  private AprilTagFieldLayout map = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
  
  
  private double id_dub;
  
  //AprilTagFieldLayout map = new AprilTagFieldLayout((Path) AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark));
  //private Path file_path = FileSystem.getDeployDirectory().toPath().resolve(path_to_map);

  
  public driveToLeft(DriveSubsystem m_drive_subsystem, LimelightHelpers m_limelight_subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_limelight_subsystem = m_limelight_subsystem;
    this.m_drive_subsystem = m_drive_subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    id_dub = LimelightHelpers.getFiducialID("limelight");
    int id = (int) id_dub;
    Pose2d position_of_apriltag = map.getTagPose(id).get().toPose2d();
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);
    //config.setReversed(true);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        m_drive_subsystem.m_swerve_drive_pose_estimator.getEstimatedPosition(),
        List.of(new Translation2d(0, 0)),
        position_of_apriltag.rotateBy(position_of_apriltag.getRotation().times(-1)).transformBy(new Transform2d(0, -1, new Rotation2d())).rotateBy(position_of_apriltag.getRotation()),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_drive_subsystem::getPose,
        DriveConstants.kDriveKinematics,

        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_drive_subsystem::setModuleStates,
        m_drive_subsystem);

        m_drive_subsystem.resetOdometry(exampleTrajectory.getInitialPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveControllerCommand.andThen(() -> m_drive_subsystem.drive(0, 0, 0, false));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

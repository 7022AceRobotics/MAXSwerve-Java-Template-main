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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.LimelightHelpers;

import frc.robot.Constants.DriveConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class driveToLeft extends InstantCommand {
  /** Creates a new driveToLeft. */
  private DriveSubsystem m_drive_subsystem;
  private LimelightHelpers m_limelight_subsystem;
  private SwerveControllerCommand swerveControllerCommand;
  private AprilTagFieldLayout map = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
  
  
  private double id_dub;
  private Pose2d pose_final;
  private Pose2d pose_initial;
  
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
    if (id_dub != -1){
    int id = (int) id_dub;
    Pose2d position_of_apriltag = map.getTagPose(id).get().toPose2d();
    pose_initial = m_drive_subsystem.m_swerve_drive_pose_estimator.getEstimatedPosition();
    pose_final = position_of_apriltag.rotateBy(position_of_apriltag.getRotation().times(-1)).transformBy(new Transform2d(DriveConstants.kWheelBase/2, -0.08, new Rotation2d(0))).rotateBy(position_of_apriltag.getRotation()).rotateBy(new Rotation2d(3.14159)).times(-1);
    SmartDashboard.putNumber("P", position_of_apriltag.getRotation().getDegrees());
    if(id != 7 && id != 10 && id != 18 && id != 21){
      pose_final = new Pose2d(pose_final.getX(), pose_final.getY(), pose_final.getRotation().times(-1));
      SmartDashboard.putNumber("F", pose_final.getRotation().getDegrees());
    }

    //m_drive_subsystem.reseOdometry(pose_initial);

    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);
    //config.setReversed(true);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        pose_initial,
        List.of(),
        pose_final,
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, AutoConstants.kIThetaController, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_drive_subsystem::getPose,
        DriveConstants.kDriveKinematics,

        new PIDController(AutoConstants.kPXController, 0, AutoConstants.kIXController),
        new PIDController(AutoConstants.kPYController, 0, AutoConstants.kIYController),
        thetaController,
        m_drive_subsystem::setModuleStates,
        m_drive_subsystem);

    swerveControllerCommand.andThen(() -> m_drive_subsystem.drive(0, 0, 0, false, DriverStation.getAlliance())).schedule();

    // SmartDashboard.putNumber("X1: ", pose_initial.getX());
    // SmartDashboard.putNumber("Y1: ", pose_initial.getY());

    // SmartDashboard.putNumber("X2: ", pose_final.getX());
    // SmartDashboard.putNumber("Y2: ", pose_final.getY());

    // SmartDashboard.putNumber("R: ", pose_initial.getRotation().getDegrees());
    // SmartDashboard.putNumber("R2: ", pose_final.getRotation().getDegrees());
    
    m_drive_subsystem.resetOdometry(exampleTrajectory.getInitialPose());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (id_dub != -1){
    swerveControllerCommand.end(interrupted);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

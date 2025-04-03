// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.nio.file.FileSystem;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

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
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVisionHelper;
import frc.robot.util.LimelightHelpers;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.Constants.DriveConstants;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestPathFinder extends Command {
  /** Creates a new driveToLeft. */
  private DriveSubsystem m_drive_subsystem;
  private LimelightHelpers m_limelight_subsystem;
  private Command swerveControllerCommand;
  private AprilTagFieldLayout map = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
  
  
  private double id_dub;
  private Pose2d pose_final;
  private Pose2d pose_initial;

  private PathConstraints obstacles;
  private List<Waypoint> waypoints;
  private PathPlannerPath path;
  private int id;
  
  //AprilTagFieldLayout map = new AprilTagFieldLayout((Path) AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark));
  //private Path file_path = FileSystem.getDeployDirectory().toPath().resolve(path_to_map);

  
  public TestPathFinder(DriveSubsystem m_drive_subsystem, LimelightHelpers m_photon_vision_subsystem, int id) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_limelight_subsystem = m_photon_vision_subsystem;
    this.m_drive_subsystem = m_drive_subsystem;
    this.id = id;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d position_of_apriltag = map.getTagPose(id).get().toPose2d();
    pose_initial = m_drive_subsystem.m_swerve_drive_pose_estimator.getEstimatedPosition();

    pose_final = position_of_apriltag.rotateBy(position_of_apriltag.getRotation().times(-1))
    .transformBy(new Transform2d(DriveConstants.kWheelBase/2, -0.05, new Rotation2d(0)))
    .rotateBy(position_of_apriltag.getRotation())
    .rotateBy(new Rotation2d(Math.PI))
    .times(-1);
    
    if(id != 7 && id != 10 && id != 18 && id != 21){
      pose_final = new Pose2d(pose_final.getX(), pose_final.getY(), pose_final.getRotation().times(-1));
    }

    path = m_drive_subsystem.getPathTo(pose_initial, pose_final);

    }
  }


  @Override
  public void test_pathfinder_function(pose_initial, pose_final){
    path = m_drive_subsystem.getPathTo(pose_initial, pose_final)

    obstacles = new PathConstraints(
      DriveConstants.kMaxSpeedMetersPerSecond, 
      DriveConstants.kMaxAccelerationMetersPerSecondSquared, 
      DriveConstants.kMaxAngularSpeed, 
      DriveConstants.kMaxAngularSpeedRadiansPerSecondSquared
    );

    AutoBuilder.pathfindThenFollowPath(
        path,
        constraints
    );
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

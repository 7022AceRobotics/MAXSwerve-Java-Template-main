// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVisionHelper;
import frc.robot.util.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class driveToCoralStation extends Command {
  /** Creates a new driveToCoralStation. */
  private DriveSubsystem m_drive_subsystem;
  private PhotonVisionHelper m_photon_vision_subsystem;
  private SwerveControllerCommand swerveControllerCommand;
  private AprilTagFieldLayout map = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
  
  private Pose2d pose_final;
  private Pose2d pose_initial;
  public driveToCoralStation(DriveSubsystem m_drive_subsystem, PhotonVisionHelper m_photon_vision_subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive_subsystem = m_drive_subsystem;
    this.m_photon_vision_subsystem = m_photon_vision_subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_photon_vision_subsystem.getID() != -1){
    Pose2d position_of_apriltag = map.getTagPose(m_photon_vision_subsystem.getID()).get().toPose2d();
    pose_initial = m_drive_subsystem.m_swerve_drive_pose_estimator.getEstimatedPosition();
    pose_initial = new Pose2d(pose_initial.getX(), pose_initial.getY(), new Rotation2d(pose_initial.getRotation().getRadians() + Math.PI));
    pose_final = position_of_apriltag.rotateBy(position_of_apriltag.getRotation().times(-1)).transformBy(new Transform2d(DriveConstants.kWheelBase/2, -0.08, new Rotation2d(0))).rotateBy(position_of_apriltag.getRotation());
    //pose_final = new Pose2d(pose_final.getX(), pose_final.getY(), pose_final.getRotation().times(-1));
    //m_drive_subsystem.reseOdometry(pose_initial);

    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);
    //config.setReversed(true);

    // List.of(new Translation2d(
    //       Math.min(pose_initial.getX(), pose_final.getX()) + Math.abs(pose_initial.getX() - pose_final.getX())/3, 
    //       Math.min(pose_initial.getY(), pose_final.getY()) + Math.abs(pose_initial.getY() - pose_final.getY())/3),
    //       new Translation2d(
    //       (pose_initial.getX() + pose_final.getX())/2, 
    //       (pose_initial.getY() + pose_final.getY())/2))


    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        pose_initial,
        List.of(),
        pose_final,
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, AutoConstants.kIThetaController, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    m_drive_subsystem.resetOdometry(exampleTrajectory.getInitialPose());

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

    SmartDashboard.putNumber("X1: ", pose_initial.getX());
    SmartDashboard.putNumber("Y1: ", pose_initial.getY());

    SmartDashboard.putNumber("X2: ", pose_final.getX());
    SmartDashboard.putNumber("Y2: ", pose_final.getY());

    SmartDashboard.putNumber("R: ", pose_initial.getRotation().getDegrees());
    SmartDashboard.putNumber("R2: ", pose_final.getRotation().getDegrees());
    //m_drive_subsystem.resetOdometry(exampleTrajectory.getInitialPose());
    }
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

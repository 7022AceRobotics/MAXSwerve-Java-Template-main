// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightHelpersC;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

 

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class limelight extends Command {
  /** Creates a new limelight. */
  private final LimelightHelpersC m_limelight_subsystem;
  private final DriveSubsystem m_drive_subsystem;
  private DoubleLogEntry m_DoubleLog;
  private DataLog log;
  private Pose2d pose;
  //private SwerveDrivePoseEstimator m_swerve_drive_pose_estimator;
  private Timer timer;

  private PoseEstimate botpose;
  private double time;

  public limelight(LimelightHelpersC limelight_subsystem, DriveSubsystem drive_subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_limelight_subsystem = limelight_subsystem;
    this.m_drive_subsystem = drive_subsystem;
    //this.m_swerve_drive_pose_estimator = m_drive_subsystem.m_swerve_drive_pose_estimator;
    

    addRequirements(limelight_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_swerve_drive_pose_estimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, m_drive_subsystem.getHeading(), m_drive_subsystem.getSwerveModulePositions(), new Pose2d());
    time = timer.getFPGATimestamp();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //LimelightHelpers.SetRobotOrientation("limelight", m_drive_subsystem.m_swerve_drive_pose_estimator.getEstimatedPosition().getRotation().getDegrees() + 180, m_drive_subsystem.m_gyro.getRate(), 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight", m_drive_subsystem.m_gyro.getAngle(), m_drive_subsystem.m_gyro.getRate(), 0, 0, 0, 0);
    botpose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    log = DataLogManager.getLog();
    m_DoubleLog = new DoubleLogEntry(log, "/my/double");

    SmartDashboard.putNumber("rotation: ", this.m_drive_subsystem.m_swerve_drive_pose_estimator.getEstimatedPosition().getRotation().getDegrees());
    SmartDashboard.putNumber("rotation2: ", this.m_drive_subsystem.m_gyro.getAngle());

    if (botpose != null){
      if (botpose.tagCount > 0){
        SmartDashboard.putNumber("Pose1: ", botpose.pose.getX());
        SmartDashboard.putNumber("Pose2: ", botpose.pose.getY());
        SmartDashboard.putNumber("Pose3: ", botpose.pose.getRotation().getDegrees());
              // SmartDashboard.putNumber("Pose4: ", botpose[3]);
        // SmartDashboard.putNumber("Pose5: ", botpose[4]);
        // SmartDashboard.putNumber("Pose6: ", botpose[5]);
        // m_DoubleLog.append(botpose[2]);

        // pose = new Pose2d(botpose[0], botpose[1], m_drive_subsystem.getHeading());

        this.m_drive_subsystem.updateVisionSwerveDrive(botpose.pose, botpose.timestampSeconds);

        this.m_drive_subsystem.resetOdometry(m_drive_subsystem.getPoseEstimate());
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

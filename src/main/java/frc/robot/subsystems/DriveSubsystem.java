// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import java.nio.file.Path;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonPoseEstimator;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Configs;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MicrosoftCameraConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.changing_vars;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset,
      Configs.MAXSwerveModule.turningConfig,
      Configs.MAXSwerveModule.turningConfig);

  private MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset,
      Configs.MAXSwerveModule.turningConfig,
      Configs.MAXSwerveModule.turningConfig);

  private MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset,
      Configs.MAXSwerveModule.turningConfig,
      Configs.MAXSwerveModule.turningConfig);

  private MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset,
      Configs.MAXSwerveModule.turningConfig,
      Configs.MAXSwerveModule.turningConfig);

  // The gyro sensor
  public final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  //@Logged(name = "Odo")
  public SwerveDrivePoseEstimator m_swerve_drive_pose_estimator = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics, 
    Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)), 
    new SwerveModulePosition[] {
    m_frontLeft.getPosition(),
    m_frontRight.getPosition(),
    m_rearLeft.getPosition(),
    m_rearRight.getPosition()}, 
    new Pose2d());

  public PhotonPoseEstimator m_photon_pose_estimator = new PhotonPoseEstimator(
          MicrosoftCameraConstants.map, 
          PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, 
          MicrosoftCameraConstants.camToRobot);

  public Field2d m_field = new Field2d();

  double xSpeedDelivered;
  double ySpeedDelivered;
  double rotDelivered;


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    startHeading();
    zeroHeadingThread().start();
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5, 0.0, 0.0) // Rotation PID constants
            ),
            PathPlannerConstants.robot, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

    SmartDashboard.putNumber("Drive P", 0);
    SmartDashboard.putNumber("Drive I", 0);
    SmartDashboard.putNumber("Drive D", 0);
    SmartDashboard.putNumber("Turning P", 0);
    SmartDashboard.putNumber("Turning I", 0);
    SmartDashboard.putNumber("Turning D", 0);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_field.setRobotPose(getPose());
    m_field.setRobotPose(m_swerve_drive_pose_estimator.getEstimatedPosition());
    SmartDashboard.putData("Field", m_field);
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    updateSwerveDrive(Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)), getSwerveModulePositions());

    SmartDashboard.putNumber("rotation3",m_gyro.getAngle());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  @Logged(name="joe is a bad driver")
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, Optional<Alliance> ally) {
    // Convert the commanded speeds into the correct units for the drivetrain
    xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond * changing_vars.speed_multi_change;
    ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond * changing_vars.speed_multi_change;
    rotDelivered = rot * DriveConstants.kMaxAngularSpeed * changing_vars.speed_multi_change;

    // SlewRateLimiter filter = new SlewRateLimiter(2);
    // xSpeedDelivered = filter.calculate(xSpeedDelivered);
    // ySpeedDelivered = filter.calculate(ySpeedDelivered);
    //rotDelivered = filter.calculate(rotDelivered);
    // if (ally.isPresent()){
    // if (ally.get() == Alliance.Blue){
    //   m_gyro.setGyroAngleZ(m_gyro.getAngle(IMUAxis.kZ) - 180);
    // }
  //}
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      ChassisSpeeds.discretize(fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered), 0.02));
    //ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(wheelSpeeds, 0.02);

  SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

  //   if (ally.isPresent()){
  //   if (ally.get() == Alliance.Blue){
  //     m_gyro.setGyroAngleZ(m_gyro.getAngle(IMUAxis.kZ) + 180);
  //   }
  // }

    SmartDashboard.putNumber("Theo", swerveModuleStates[0].speedMetersPerSecond);
    SmartDashboard.putNumber("Act", m_frontLeft.getState().speedMetersPerSecond);

    SmartDashboard.putNumber("Theo2", swerveModuleStates[1].speedMetersPerSecond);
    SmartDashboard.putNumber("Act2", m_frontRight.getState().speedMetersPerSecond);
  }

  public void driveRobotRelative(ChassisSpeeds wheelSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(wheelSpeeds, 0.08);
    var swerveModuleState = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleState, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleState[0]);
    m_frontRight.setDesiredState(swerveModuleState[1]);
    m_rearLeft.setDesiredState(swerveModuleState[2]);
    m_rearRight.setDesiredState(swerveModuleState[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    //m_gyro.setGyroAngleZ(180);
  }

  public void startHeading() {
    m_gyro.reset();
    m_gyro.setGyroAngleZ(225);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  @Logged(name="Swerve_Rotation")
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ));
  }


  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public SwerveModulePosition[] getSwerveModulePositions(){
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()};
  }

  public void updateSwerveDrive(Rotation2d rotation, SwerveModulePosition[] swerveModulePositions){
    m_swerve_drive_pose_estimator.update(rotation, swerveModulePositions);
  }

  public void updateVisionSwerveDrive(Pose2d pose, double timestamp){
    m_swerve_drive_pose_estimator.addVisionMeasurement(pose, timestamp);;
  }
  public Pose2d getPoseEstimate(){
    return m_swerve_drive_pose_estimator.getEstimatedPosition();
  }

  @Logged(name="ChassisSpeeds")
  public ChassisSpeeds getChassisSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getSwerveModuleStateCus());
  }
  @Logged(name="voltageSwerve1")
  public double getVoltage(){
    return m_frontLeft.getDriveVoltage();
  }

  @Logged(name="swerve_positions")
  public SwerveModuleState[] getSwerveModuleStateCus(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = m_frontLeft.getState();
    states[1] = m_frontRight.getState();
    states[2] = m_rearLeft.getState();
    states[3] = m_rearRight.getState();
    return states;
  }

  @Logged(name="targetted_swerve")
  public ChassisSpeeds getTargetedSwerve(){
    return new ChassisSpeeds(xSpeedDelivered * DriveConstants.kMaxSpeedMetersPerSecond * changing_vars.speed_multi_change, ySpeedDelivered * DriveConstants.kMaxSpeedMetersPerSecond * changing_vars.speed_multi_change, rotDelivered * DriveConstants.kMaxSpeedMetersPerSecond * changing_vars.speed_multi_change);
  }

  public PathPlannerPath getPathTo(Pose2d pose_initial, Pose2d pose_final, Pose2d pose_middle){

    List<Waypoint> waypoints;
    PathPlannerPath path;

    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);
    //config.setReversed(true);

    // waypoints = PathPlannerPath.waypointsFromPoses(
    //   new Pose2d(pose_initial.getX(), pose_initial.getY(), new Rotation2d(0)),
    //   //new Pose2d(pose_middle.getX(), pose_middle.getY(), new Rotation2d(0)),
    //   new Pose2d(pose_final.getX(), pose_final.getY(), new Rotation2d(0))
    // );

    waypoints = PathPlannerPath.waypointsFromPoses(
      pose_initial,
      //new Pose2d(pose_middle.getX(), pose_middle.getY(), new Rotation2d(0)),
      pose_final
      );

    List<ConstraintsZone> constraint_zones = List.of(new ConstraintsZone(
      0.65, 
      1, 
      new PathConstraints(
        DriveConstants.kMaxSpeedMetersPerSecond*0.5, 
        DriveConstants.kMaxAccelerationMetersPerSecondSquared*0.5, 
        DriveConstants.kMaxAngularSpeed*0.5, 
        DriveConstants.kMaxAngularSpeedRadiansPerSecondSquared*0.5)));

    // waypoints = PathPlannerPath.waypointsFromPoses(
    //   pose_initial,
    //   pose_middle,
    //   pose_final
    // );
    var test10 = 1;
    PathConstraints constraints = new PathConstraints(
      DriveConstants.kMaxSpeedMetersPerSecond*test10,
      DriveConstants.kMaxAccelerationMetersPerSecondSquared*test10, 
      DriveConstants.kMaxAngularSpeed*test10, 
      DriveConstants.kMaxAngularSpeedRadiansPerSecondSquared*test10);

    // path = new PathPlannerPath(
    //   waypoints, 
    //   constraints, 
    //   new IdealStartingState(Math.pow(Math.pow(getChassisSpeeds().vxMetersPerSecond,2) + Math.pow(getChassisSpeeds().vyMetersPerSecond,2), 0.5), pose_initial.getRotation()), 
    //   new GoalEndState(0, pose_final.getRotation()));
    // path.preventFlipping = true;

    path = new PathPlannerPath(
      waypoints, 
      List.of(), 
      List.of(),
       constraint_zones, 
       List.of(), 
       constraints, 
       new IdealStartingState(Math.pow(Math.pow(getChassisSpeeds().vxMetersPerSecond,2) + Math.pow(getChassisSpeeds().vyMetersPerSecond,2), 0.5), pose_initial.getRotation()), 
       new GoalEndState(0, pose_final.getRotation()), 
       false);

      path.preventFlipping = true;


    // VERY LIKELY TO BE EXCESSIVE. DEPRECATE POTENTIALLY
    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, AutoConstants.kIThetaController, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    resetOdometry(pose_initial);

    return path;
  }

  public Command getPathFinderPath(Pose2d pose_initial, Pose2d pose_middle, Pose2d pose_final){
    PathConstraints path_constraints = new PathConstraints(
    DriveConstants.kMaxSpeedMetersPerSecond, 
    DriveConstants.kMaxAccelerationMetersPerSecondSquared, 
    DriveConstants.kMaxAngularSpeed, 
    DriveConstants.kMaxAngularSpeedRadiansPerSecondSquared);

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(pose_initial.getX(), pose_initial.getY(), new Rotation2d(0)),
      //new Pose2d(pose_middle.getX(), pose_middle.getY(), new Rotation2d(0)),
      new Pose2d(pose_final.getX(), pose_final.getY(), new Rotation2d(0))
    );

    List<ConstraintsZone> constraint_zones = List.of(new ConstraintsZone(
      0.65, 
      1, 
      new PathConstraints(
        DriveConstants.kMaxSpeedMetersPerSecond, 
        DriveConstants.kMaxAccelerationMetersPerSecondSquared, 
        DriveConstants.kMaxAngularSpeed, 
        DriveConstants.kMaxAngularSpeedRadiansPerSecondSquared)));

    PathPlannerPath path = new PathPlannerPath(
      waypoints, 
      path_constraints, 
      null, 
      new GoalEndState(0, pose_final.getRotation()));
    
    return AutoBuilder.pathfindThenFollowPath(path, path_constraints);


    
  }

  public void resetPID(){

    // USED FOR TESTING

    SparkMaxConfig new_driving_config = new SparkMaxConfig();
    SparkMaxConfig new_turning_config = new SparkMaxConfig();

    double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
    double turningFactor = 2 * Math.PI;
    double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;
    
    new_driving_config.idleMode(IdleMode.kBrake)
      .smartCurrentLimit(50);

      new_driving_config.encoder
      .positionConversionFactor(drivingFactor) // meters
      .velocityConversionFactor(drivingFactor / 60.0);

      new_driving_config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(SmartDashboard.getNumber("Drive P", 0))
      .i(SmartDashboard.getNumber("Drive I", 0))
      .d(SmartDashboard.getNumber("Drive D", 0))
      .velocityFF(drivingVelocityFeedForward)
      .outputRange(-1, 1);
    
      new_turning_config
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(20);

      new_turning_config.absoluteEncoder
      .inverted(true)
      .positionConversionFactor(turningFactor)
      .velocityConversionFactor(turningFactor / 60.0);

      new_turning_config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(SmartDashboard.getNumber("Turning P", 0))
      .i(SmartDashboard.getNumber("Turning I", 0))
      .d(SmartDashboard.getNumber("Turning D", 0))
      .velocityFF(drivingVelocityFeedForward)
      .outputRange(-1, 1)
      .positionWrappingEnabled(true)
      .positionWrappingInputRange(0, turningFactor);
    
    // Configs.MAXSwerveModule.drivingConfig.apply(
    //   new_driving_config
    // );
    // Configs.MAXSwerveModule.turningConfig.apply(
    //   new_turning_config
    // );

    m_frontLeft.changeConfig(new_turning_config, new_driving_config);
    m_frontRight.changeConfig(new_turning_config, new_driving_config);
    m_rearLeft.changeConfig(new_turning_config, new_driving_config);
    m_rearRight.changeConfig(new_turning_config, new_driving_config);

  }


public Thread zeroHeadingThread(){
  Thread zThread = new Thread(() -> {
    try {
      Thread.sleep(1000);
      zeroHeading();
      //startHeading();
      //m_odometry.resetPose(new Pose2d(7.157, 7.157, new Rotation2d(135)));
    }catch (Exception e) {
    }
  });
  return zThread;
}

}

package frc.robot.subsystems.Drive;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants.DriveMode;

public class Swerve extends SubsystemBase {
  public SwerveModule[] mSwerveMods;
  public GyroPigeon2 gyro;

  private final SwerveDrivePoseEstimator poseEstimator;

  private final VisionPhotonvision vision = new VisionPhotonvision();

  private final PIDController snapPIDController;
  private DriveMode driveMode = DriveMode.NORMAL;

  private Field2d debugField2d = new Field2d();

  // Tunable values
  private LoggedTunableNumber driveP = new LoggedTunableNumber("driveP", Constants.SwerveConstants.drivePID[0]);
  private LoggedTunableNumber driveI = new LoggedTunableNumber("driveI", Constants.SwerveConstants.drivePID[1]);
  private LoggedTunableNumber driveD = new LoggedTunableNumber("driveD", Constants.SwerveConstants.drivePID[2]);

  private LoggedTunableNumber driveS = new LoggedTunableNumber("driveS", Constants.SwerveConstants.driveSVA[0]);
  private LoggedTunableNumber driveV = new LoggedTunableNumber("driveV", Constants.SwerveConstants.driveSVA[1]);
  private LoggedTunableNumber driveA = new LoggedTunableNumber("driveA", Constants.SwerveConstants.driveSVA[2]);

  private LoggedTunableNumber snapP = new LoggedTunableNumber("SnapP", Constants.SwerveConstants.snapPID[0]);
  private LoggedTunableNumber snapI = new LoggedTunableNumber("SnapI", Constants.SwerveConstants.snapPID[1]);
  private LoggedTunableNumber snapD = new LoggedTunableNumber("SnapD", Constants.SwerveConstants.snapPID[2]);

  public Swerve() {
    /* Gyro setup */
    gyro = new GyroPigeon2(Constants.SwerveConstants.pigeonID);
    gyro.home();

    /* Swerve modules setup */
    mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
        new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
        new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
        new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
    };

    poseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.swerveKinematics, getYaw(), getPositions(),
        new Pose2d());

    snapPIDController = new PIDController(
        Constants.SwerveConstants.snapPID[0],
        Constants.SwerveConstants.snapPID[1],
        Constants.SwerveConstants.snapPID[2]);

    snapPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * The main function used for driving the robot
   * 
   * @param translation
   * @param rotation
   */
  public void drive(Translation2d translation, double rotation) {
    switch (driveMode) {
      case AMP:
        rotation = snapPIDController.calculate(getYaw().getRadians(), getRotationRelativeToAmp().getRadians());
        break;
      case LEFT:
        rotation = snapPIDController.calculate(getYaw().getRadians(), Math.PI / 2);
        break;
      case RIGHT:
        rotation = snapPIDController.calculate(getYaw().getRadians(), -Math.PI / 2);
        break;
      case FORWARD:
        rotation = snapPIDController.calculate(getYaw().getRadians(), 0);
        break;
      case BACKWARD:
        rotation = snapPIDController.calculate(getYaw().getRadians(), Math.PI);
        break;
      default:
        break;
    }

    SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
        ChassisSpeed.from);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber]);
    }
  }

  public void setDriveMode(DriveMode mode) {
    driveMode = mode;
  }

  public DriveMode getDriveMode() {
    return driveMode;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber]);
    }
  }

  /** Reset the module encoder values */
  public void resetModuleZeros() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  /**
   * Get the current state of the modules
   * 
   * @return state of the modules
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }

    return positions;
  }

  public SwerveModulePosition[] getRedPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getRedPosition();
    }

    return positions;
  }

  /**
   * get the orientation of the robot
   * 
   * @return the orientation of the robot
   */
  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(gyro.getYaw());
  }

  /**
   * Get the pitch from gyro
   * 
   * @return the pitch of the robot
   */
  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(gyro.getPitch());
  }

  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(gyro.getRoll());
  }

  public void zeroGyro() {
    gyro.homeYaw();
  }

  public void checkTunableValues() {
    if (!Constants.enableTunableValues)
      return;

    if (driveP.hasChanged() || driveI.hasChanged() || driveD.hasChanged()) {
      for (SwerveModule mod : mSwerveMods) {
        mod.driveController.setP(driveP.get());
        mod.driveController.setI(driveI.get());
        mod.driveController.setD(driveD.get());
      }
    }
    if (driveS.hasChanged() || driveV.hasChanged() || driveA.hasChanged()) {
      for (SwerveModule mod : mSwerveMods) {
        mod.feedforward = new SimpleMotorFeedforward(driveS.get(), driveV.get(), driveA.get());
      }
    }

    if (snapP.hasChanged() || snapI.hasChanged() || snapD.hasChanged()) {
      snapPIDController.setPID(snapP.get(), snapI.get(), snapD.get());
    }
  }

  public void updateVisionMeasurements() {
    var visionEst = vision.getEstimatedGlobalPose();
    visionEst.ifPresent(est -> {
      var estPose = est.estimatedPose.toPose2d();
      var estStdDevs = vision.getEstimationStdDevs(estPose);
      poseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
    });
  }

  private Optional<Pose3d> getAmpPose() {
    var alliance = DriverStation.getAlliance();
    Optional<Pose3d> ampPose = null;

    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Blue) {
        ampPose = Constants.VisionConstants.kTagLayout.getTagPose(7);
      } else {
        ampPose = Constants.VisionConstants.kTagLayout.getTagPose(4);
      }
    } else {
      ampPose = Constants.VisionConstants.kTagLayout.getTagPose(7);
    }

    return ampPose;
  }

  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public double getDistanceFromAmp() {
    return getEstimatedPose().getTranslation().getDistance(getAmpPose().get().getTranslation().toTranslation2d());
  }

  public Rotation2d getRotationRelativeToAmp() {
    return getEstimatedPose().getTranslation().minus(getAmpPose().get().getTranslation().toTranslation2d()).unaryMinus()
        .getAngle();
  }

  @Override
  public void periodic() {
    for (SwerveModule mod : mSwerveMods) {
      mod.logValues();
    }
    gyro.logValues();

    poseEstimator.update(getYaw(), getPositions());

    SmartDashboard.putNumber("Distance From Amp", getDistanceFromAmp());
    SmartDashboard.putNumber("Rotation to Amp", getRotationRelativeToAmp().getDegrees());

    debugField2d.setRobotPose(getEstimatedPose());
    if (vision.latestVision.isPresent()) {
      debugField2d.getObject("Vision Intrusive Thoughts").setPose(vision.latestVision.get().estimatedPose.toPose2d());
    }
  }
}

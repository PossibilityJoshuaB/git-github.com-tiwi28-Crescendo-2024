package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

  public static boolean enableTunableValues = true;

  public static class LightsConstants {
    public static int port = 1; // TODO: irl values TBD
    public static int length = 0; // TODO: irl values TBD

    public static enum LightsType {
      ENDGAME,
      CLIMB,
      SHOOTING,
      INTAKE,
      IDLE,
      DISABLED
    }
  }

  public static final class VisionConstants {
    public static String cameraName = "orangepi";
    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
        new Rotation3d(0, Units.degreesToRadians(-30), 0));
    public static final Vector<N3> stateStdDevs = VecBuilder.fill(0.2, 0.2, 0.4);
    public static final Vector<N3> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Vector<N3> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static final class Operators {
    public static final int driver = 0;
    public static final int operator = 1;
  }

  public static final class SwerveConstants {

    public static enum DriveMode {
      NORMAL,
      AMP,
      LEFT,
      RIGHT,
      FORWARD,
      BACKWARD
    }

    public static final double[] snapPID = { 0, 0, 0 }; // TODO: tune IRL

    /* Drive Controls */
    public static final double stickDeadband = 0.1;

    /* Gyro */
    public static final int pigeonID = 21;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(23.0);
    public static final double wheelBase = Units.inchesToMeters(23.0);
    public static final double wheelDiameter = Units.inchesToMeters(4.0); // TODO: measure
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = 150.0 / 7.0; // 150/7:1

    /* Kinematics */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second
    public static final double maxAngularVelocity = Math.PI * 2 * 2; // rads per second

    /* Neutral Modes */
    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    public static final String[] moduleNames = { "Front Left", "Front Right", "Back Left", "Back Right" }; // module #0,
    // #1, #2, #3

    public static final double[] driveSVA = new double[] { 0.05, 2.5, 0.5 };
    public static final double[] drivePID = new double[] { 0.001, 0.00005, 0.0005 };
    public static final double[] anglePID = new double[] { 0.02, 0.0, 0.005 };

    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 12;
      public static final int canCoderID = 0;
      public static final double angleOffset = 31.7;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, anglePID, drivePID, driveSVA);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 13;
      public static final int angleMotorID = 14;
      public static final int canCoderID = 1;
      public static final double angleOffset = 165;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, anglePID, drivePID, driveSVA);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 15;
      public static final int angleMotorID = 16;
      public static final int canCoderID = 3;
      public static final double angleOffset = 349.5;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, anglePID, drivePID, driveSVA);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 17;
      public static final int angleMotorID = 18;
      public static final int canCoderID = 2;
      public static final double angleOffset = 31.4;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, anglePID, drivePID, driveSVA);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final PIDConstants translationPID = new PIDConstants(0, 0, 0); // TODO: Tune after tunning modules
    public static final PIDConstants rotationPID = new PIDConstants(0, 0, 0); // TODO: Tune after tunning modules

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class ShooterConstants {

    public static final int id = 5;
    public static final double[] shooterPID = { 0, 0, 0 };
    public static final double[] shooterFeedforward = { 0, 0 };
    public static final int toleranceRPM = 10;
    public static final double maxVelocityPerSecond = 2000; // RPM/s
    public static final double maxAcceleration = 700; // RPM/s^2
  }
}

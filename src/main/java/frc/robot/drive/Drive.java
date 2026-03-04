package frc.robot.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Subsystem;
import frc.lib.sendables.SwerveDriveSendable;
import frc.lib.swerves.SwerveOutput;
import java.util.function.Supplier;

import frc.robot.localization.Vision;
import frc.robot.localization.VisionSim;

public class Drive extends Subsystem {

  private static Drive instance = null;

  private final SwerveOutput swerve;

  private SwerveDrivetrain.SwerveDriveState state;

  private final Field2d field;

  private boolean hasSetPerspective = false;

  private PIDController yawPID;

  /**
   * Vision-based localization system.
   */
  private final Vision vision;

  public static Drive getInstance() {
    if (instance == null) {
      instance = new Drive();
    }

    return instance;
  }

  private Drive() {
    swerve = DriveFactory.createSwerve();
    state = new SwerveDrivetrain.SwerveDriveState();
    field = new Field2d();

    yawPID = new PIDController(6, 0.0, 0.0);
    yawPID.enableContinuousInput(-0.5, 0.5);

    vision = new VisionSim("PhotonVisionSim");

    // TODO Static camera -- Needs an actual measurement from CAD
    // TODO Add support for cameras where the transform varies
    vision.addCamera("Static", new Transform3d(new Translation3d(0.1, 0, 0.5), new Rotation3d(0, Math.toRadians(-30), Math.toRadians(30))));
    vision.registerPoseUpdate(estimate -> addPoseEstimate(estimate.pose().toPose2d(), estimate.timestamp()));
  }

  @Override
  public void initializeTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

    tab.add("Field", field);
    tab.add("States", new SwerveDriveSendable(() -> state.ModuleStates, () -> this.getPose().getRotation()));
    tab.add("Targets", new SwerveDriveSendable(() -> state.ModuleTargets, () -> this.getPose().getRotation()));
  }

  @Override
  public void periodic() {
    state = swerve.getState();
    // NOTE Causes a feedback loop where simulated poses keep approaching the tags
    vision.update(state.Pose);
    field.setRobotPose(state.Pose);

    // NOTE This was taken from the generated project, unsure if it is needed
    // trySettingPerspective();
  }

  private void trySettingPerspective() {
    if (!hasSetPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance().ifPresent(
        allianceColor -> {
          swerve.setOperatorPerspectiveForward(
            allianceColor == DriverStation.Alliance.Red
              ? Rotation2d.k180deg
              : Rotation2d.kZero);
            hasSetPerspective = true;
        }
      );
    }
  }

  /**
   * Adds a pose estimate.
   *
   * @param pose2d The pose estimate to add.
   * @param timestamp The timestamp of the pose estimate.
   */
  public void addPoseEstimate(Pose2d pose2d, Time timestamp) {
    swerve.addVisionMeasurement(pose2d, timestamp.in(Seconds));
  }

  public ChassisSpeeds speedsFromController(CommandXboxController controller) {
    LinearVelocity maxVelocity = MetersPerSecond.of(2);
    AngularVelocity maxAngularVelocity = RotationsPerSecond.of(0.5);

    double x = MathUtil.applyDeadband(-controller.getLeftY(), 0.2);
    double y = MathUtil.applyDeadband(-controller.getLeftX(), 0.2);
    double omega = MathUtil.applyDeadband(-controller.getRightX(), 0.1);

    return new ChassisSpeeds(
      maxVelocity.times(x), maxVelocity.times(y), maxAngularVelocity.times(omega));
  }

  /**
   * Takes a ChassisSpeeds object as input, and a target chassis rotation, and changes the angular velocity of
   * the ChassisSpeeds object to turn the robot towards the target rotation
   * 
   * @param speeds ChassisSpeeds object to modify
   * @param rotation Target chassis rotation
   * @return Modified ChassisSpeeds object
   */
  public ChassisSpeeds turnTowards(ChassisSpeeds speeds, Rotation2d rotation) {
    AngularVelocity angularVelocity = RotationsPerSecond.of(yawPID.calculate(state.Pose.getRotation().getRotations(), rotation.getRotations()));

    SmartDashboard.putNumber("error", Math.abs(rotation.getRotations()-state.Pose.getRotation().getRotations()));

    return new ChassisSpeeds(
      speeds.vxMetersPerSecond,
      speeds.vyMetersPerSecond,
      angularVelocity.in(RadiansPerSecond));
  }

  public ChassisSpeeds turnTowardsController(ChassisSpeeds speeds, CommandXboxController controller) {
    double x = MathUtil.applyDeadband(-controller.getRightY(), 0.1);
    double y = MathUtil.applyDeadband(-controller.getRightX(), 0.1);
    Rotation2d targetRotation = state.Pose.getRotation();
    if (!(x == 0 && y == 0)) {
      targetRotation = new Rotation2d(x, y);
      SmartDashboard.putNumber("targetRotation", targetRotation.getRotations());
    }

    return turnTowards(speeds, targetRotation);
  }

  public ChassisSpeeds turnTowardsTranslation(ChassisSpeeds speeds, Translation2d translation) {
    Translation2d robotRelative = translation.minus(state.Pose.getTranslation());

    return turnTowards(speeds, robotRelative.getAngle());
  }

  public Pose2d getPose() {
    return state.Pose;
  }

  public SwerveDriveState getState() {
    return state;
  }

  public Command drive(Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
    SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();

    return Commands.run(() -> {
      ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

      swerve.setControl(request
        .withVelocityX(fieldSpeeds.vxMetersPerSecond)
        .withVelocityY(fieldSpeeds.vyMetersPerSecond)
        .withRotationalRate(fieldSpeeds.omegaRadiansPerSecond));
    }, this);
  }
}
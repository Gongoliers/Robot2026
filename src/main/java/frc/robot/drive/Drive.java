package frc.robot.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.PosePublisher;
import frc.lib.PoseUtils;
import frc.lib.Subsystem;
import frc.lib.sendables.SwerveDriveSendable;
import frc.lib.swerves.SwerveOutput;

import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

public class Drive extends Subsystem {

  private static Drive instance = null;

  private final SwerveOutput swerve;

  private SwerveDrivetrain.SwerveDriveState state;

  private final Field2d field;

  private boolean hasSetPerspective = false;

  private PIDController yawPID;

  private final DriverAssistance driver;

  private static final Distance DRIVER_TOLERANCE = Centimeters.of(5);

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

    // TODO DriverAssistance API wart
    driver = new DriverAssistance(
      MetersPerSecond.per(Meter).of(4),
      Meters.of(1),
      Meters.of(10)
    );
  }

  @Override
  public void initializeTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

    tab.add("Field", field);
    // tab.add("States", new SwerveDriveSendable(() -> state.ModuleStates, () -> this.getPose().getRotation()));
    // tab.add("Targets", new SwerveDriveSendable(() -> state.ModuleTargets, () -> this.getPose().getRotation()));
  }

  @Override
  public void periodic() {
    state = swerve.getState();
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

  public Command drive(Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
    SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();

    return run(() -> {
      ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

      swerve.setControl(request
        .withVelocityX(fieldSpeeds.vxMetersPerSecond)
        .withVelocityY(fieldSpeeds.vyMetersPerSecond)
        .withRotationalRate(fieldSpeeds.omegaRadiansPerSecond));
    });
  }

  public Command driveFacing(Supplier<ChassisSpeeds> fieldSpeedsSupplier, Supplier<Rotation2d> targetDirection) {
    // TODO Perform configuration
    SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle();

    return run(() -> {
      ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

      swerve.setControl(request
        .withVelocityX(fieldSpeeds.vxMetersPerSecond)
        .withVelocityY(fieldSpeeds.vyMetersPerSecond)
        .withTargetDirection(targetDirection.get())
        .withTargetRateFeedforward(fieldSpeeds.omegaRadiansPerSecond));
    });
  }

  public Command driveFollowing(Supplier<Pose2d> pose) {
    return driveFacing(() -> driver.createDriverAssistanceSpeeds(getPose(), pose.get()), () -> pose.get().getRotation());
  }

  public Command driveTo(Pose2d pose) {
    BooleanSupplier atPose = () -> PoseUtils.errorMagnitude(getPose(), pose).lte(DRIVER_TOLERANCE);
    return driveFollowing(() -> pose).until(atPose);
  }
}
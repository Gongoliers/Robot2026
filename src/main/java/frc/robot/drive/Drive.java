package frc.robot.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Subsystem;
import frc.lib.sendables.SwerveDriveSendable;
import frc.lib.swerves.SwerveOutput;
import java.util.function.Supplier;

public class Drive extends Subsystem {

  private static Drive instance = null;

  private final SwerveOutput swerve;

  private SwerveDrivetrain.SwerveDriveState state;

  private final Field2d field;

  private boolean hasSetPerspective = false;

  public static Drive getInstance() {
    if (instance == null) {
      instance = new Drive();
    }

    return instance;
  }

  private Drive() {
    this.swerve = DriveFactory.createSwerve();
    this.state = new SwerveDrivetrain.SwerveDriveState();
    this.field = new Field2d();
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

  public Pose2d getPose() {
    return state.Pose;
  }

  public SysIdRoutine createDriveRoutine() {
    return DriveFactory.createDriveRoutine(swerve, this);
  }

  public SysIdRoutine createSteerRoutine() {
    return DriveFactory.createSteerRoutine(swerve, this);
  }

  public SysIdRoutine createRotationRoutine() {
    return DriveFactory.createRotationRoutine(swerve, this);
  }

  public Command sysIdQuasistatic(SysIdRoutine routine, SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine routine, SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
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
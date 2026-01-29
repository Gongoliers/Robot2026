// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.Telemetry;
import frc.robot.azimuth.Azimuth;
import frc.robot.azimuth.AzimuthTester;
import frc.robot.drive.Drive;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.ShooterTester;

/** Robot container */
public class RobotContainer {

  /** Robot container singleton */
  private static RobotContainer instance = null;

  /** Driver controller */
  private final CommandXboxController driverController;

  /** Operator controller */
  private final CommandXboxController operatorController;

  /** Multithreader */
  private final Multithreader multithreader;

  /** Drive */
  private final Drive drive;

  /** Shooter */
  private final Shooter shooter;

  /** Shooter tester */
  private final ShooterTester shooterTester;

  /** Azimuth */
  private final Azimuth azimuth;

  /** Azimuth tester */
  private final AzimuthTester azimuthTester;

  /**
   * Gets robot container instance
   * 
   * @return robot container instance
   */
  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }

    return instance;
  }

  /** Initializes robot container */
  private RobotContainer() {
    driverController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    multithreader = Multithreader.getInstance();
    drive = Drive.getInstance();
    shooter = Shooter.getInstance();
    shooterTester = ShooterTester.getInstance();
    azimuth = Azimuth.getInstance();
    azimuthTester = AzimuthTester.getInstance();

    multithreader.start();

    Telemetry.initializeTabs(drive, shooter, azimuth);
    configureDefaultCommands();
    configureBindings();
  }

  private void configureDefaultCommands() {
    drive.setDefaultCommand(drive.drive(() -> drive.speedsFromController(driverController)));
  }

  private void configureBindings() {
    operatorController.a().onTrue(azimuthTester.runFullSysId());
    operatorController.b().onTrue(Commands.runOnce(() -> azimuth.resetPosition(Rotations.of(0.0))));

    operatorController.leftBumper().onTrue(Commands.runOnce(() -> azimuth.setSetpoint(Rotations.of(0.125))));
    operatorController.rightBumper().onTrue(Commands.runOnce(() -> azimuth.setSetpoint(Rotations.of(-0.125))));
    operatorController.povLeft().whileTrue(azimuth.runAtVoltage(() -> Volts.of(1)));
    operatorController.povRight().whileTrue(azimuth.runAtVoltage(() -> Volts.of(-1)));

    operatorController.leftTrigger().whileTrue(Commands.run(() -> {
      double x = -operatorController.getRightY();
      double y = -operatorController.getRightX();

      if (Math.hypot(x, y) > 0.75) {
        azimuth.setSetpoint(Rotations.of(new Rotation2d(x, y).getRotations()));
      }
    }));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

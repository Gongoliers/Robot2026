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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Telemetry;
import frc.robot.azimuth.Azimuth;
import frc.robot.azimuth.AzimuthTester;
import frc.robot.drive.Drive;
import frc.robot.hood.Hood;
import frc.robot.hood.HoodTester;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.ShooterTester;
import frc.robot.turret.Turret;

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

  /** Hood */
  private final Hood hood;

  /** Hood tester */
  private final HoodTester hoodTester;

  /** Turret */
  private final Turret turret;

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
    hood = Hood.getInstance();
    hoodTester = HoodTester.getInstance();
    turret = Turret.getInstance();

    multithreader.start();

    Telemetry.initializeTabs();
    configureDefaultCommands();
    configureBindings();
  }

  private void configureDefaultCommands() {
    drive.setDefaultCommand(drive.drive(() -> drive.speedsFromController(driverController)));
  }

  private void configureBindings() {
    operatorController.a().onTrue(shooterTester.runFullSysId());
    
    operatorController.b().onTrue(shooterTester.findVelocityVariance(RotationsPerSecond.of(40)));
    operatorController.rightBumper().whileTrue(shooterTester.runTests(RotationsPerSecond.of(40)));

    operatorController.leftBumper().whileTrue(Commands.run(() ->shooter.setSetpoint(RotationsPerSecond.of(31))).finallyDo(() -> shooter.setSetpoint(RotationsPerSecond.of(0))));

    operatorController.leftTrigger().whileTrue(hood.runAtVoltage(() -> Volts.of(-0.5)));
    operatorController.rightTrigger().whileTrue(hood.runAtVoltage(() -> Volts.of(0.5)));

    driverController.a().onTrue(hoodTester.runFullSysId());
    
    driverController.rightBumper().whileTrue(Commands.run(() -> hood.setSetpoint(Rotations.of(0.07))).finallyDo(() -> hood.setSetpoint(Rotations.of(0.04))));

    driverController.rightTrigger().onTrue(Commands.runOnce(() -> hood.setSetpoint(hood.getMinPosition())));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

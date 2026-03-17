// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.Telemetry;
import frc.robot.azimuth.Azimuth;
import frc.robot.azimuth.AzimuthSysID;
import frc.robot.drive.Drive;
import frc.robot.hood.Hood;
import frc.robot.hood.HoodSysID;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeRollerSysID;
import frc.robot.intake.IntakeState;
import frc.robot.kicker.Kicker;
import frc.robot.kicker.KickerState;
import frc.robot.kicker.KickerSysID;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.ShooterSysID;
import frc.robot.shooter.ShooterTester;
import frc.robot.spindexer.Spindexer;
import frc.robot.spindexer.SpindexerState;
import frc.robot.spindexer.SpindexerSysID;
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

  /** Hood */
  private final Hood hood;

  /** Turret */
  private final Turret turret;

  /** Intake */
  private final Intake intake;

  /** Spindexer */
  private final Spindexer spindexer;

  /** Kicker */
  private final Kicker kicker;

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
    hood = Hood.getInstance();
    turret = Turret.getInstance();
    intake = Intake.getInstance();
    spindexer = Spindexer.getInstance();
    kicker = Kicker.getInstance();

    multithreader.start();

    Telemetry.initializeTabs();
    configureDefaultCommands();
    configureBindings();
  }

  private void configureDefaultCommands() {
    drive.setDefaultCommand(drive.drive(() -> drive.speedsFromController(driverController)));
    turret.setDefaultCommand(turret.stow());
  }

  private void configureBindings() {
    operatorController.rightTrigger().whileTrue(azimuth.runAtVoltage(() -> Volts.of(-0.5)));
    operatorController.leftTrigger().whileTrue(azimuth.runAtVoltage(() -> Volts.of(0.5)));

    driverController.a().onTrue(intake.setState(IntakeState.OUT));
    driverController.b().whileTrue(Commands.parallel(
      turret.allowExternalControl(),
      kicker.setState(KickerState.TEST),
      spindexer.setState(SpindexerState.TEST),
      intake.setState(IntakeState.AGITATE)
    )).onFalse((Commands.parallel(
      kicker.setState(KickerState.STOP),
      spindexer.setState(SpindexerState.STOP),
      intake.setState(IntakeState.OUT)
    )));
    driverController.y().onTrue(intake.setState(IntakeState.STOW));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

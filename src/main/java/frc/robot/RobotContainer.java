// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.scripting.Action.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Telemetry;
import frc.robot.azimuth.Azimuth;
import frc.robot.drive.Drive;
import frc.robot.hood.Hood;
import frc.robot.scripting.Action;
import frc.robot.scripting.ObjectiveActionMachine;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakePivotState;
import frc.robot.intake.IntakeRollerState;
import frc.robot.intake.IntakeRollerSysID;
import frc.robot.kicker.Kicker;
import frc.robot.kicker.KickerState;
import frc.robot.kicker.KickerSysID;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.ShooterTester;
import frc.robot.spindexer.Spindexer;
import frc.robot.spindexer.SpindexerState;
import frc.robot.spindexer.SpindexerSysID;
import frc.robot.turret.Turret;
import frc.robot.turret.Turret;

import java.util.Arrays;
import java.util.List;
import java.util.function.Function;
import java.util.stream.IntStream;

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

  private final List<SendableChooser<Action>> choosers;

  private final SendableChooser<Boolean> isLeftSide;

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

    // TODO Create ActionSelector class
    choosers = IntStream.rangeClosed(1, 8).mapToObj(n -> String.format("Action %d", n)).map(name -> {
      SendableChooser<Action> chooser = new SendableChooser<>();
      chooser.setDefaultOption("", NONE);

      for (Action action : Arrays.stream(values()).filter(action -> action != NONE).toList()) {
        chooser.addOption(action.name(), action);
      }

      SmartDashboard.putData(name, chooser);
      return chooser;
    }).toList();
    SmartDashboard.putString("Action", "");

    isLeftSide = new SendableChooser<>();
    isLeftSide.setDefaultOption("Left", true);
    isLeftSide.addOption("Right", false);
    SmartDashboard.putData("Is Left Side?", isLeftSide);

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
    operatorController.a().onTrue(intake.goToState(IntakePivotState.STOW));
    operatorController.b().onTrue(intake.goToState(IntakePivotState.OUT));
    operatorController.x().onTrue(IntakeRollerSysID.runFullSysId());

    driverController.b().onTrue(kicker.goToState(KickerState.TEST));
    driverController.a().onTrue(kicker.goToState(KickerState.STOP));
  }

  private Command performAction(Action action) {
    Function<Command, Command> logActionAndThen = cmd -> Commands.sequence(
        Commands.runOnce(() -> SmartDashboard.putString("Action", action.name())),
        cmd,
        Commands.runOnce(() -> SmartDashboard.putString("Action", ""))
    );

    Command fakeIt = logActionAndThen.apply(Commands.waitSeconds(2.5));

    return switch (action) {
        case NONE, INTAKE_NEUTRAL, PASS, INTAKE_ZONE, CLIMB -> fakeIt;
        // TODO Implement `turret.score() -> Command`
        case SCORE -> logActionAndThen.apply(turret.faceHub().withTimeout(2.5).asProxy().withName("faceHub"));
    };
  }

  public Command getAutonomousCommand() {
    DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    Action[] actions = choosers.stream().map(SendableChooser::getSelected).toArray(Action[]::new);
    if (isLeftSide.getSelected()) {
      return ObjectiveActionMachine.createLeftSideCommand(alliance, actions, drive::driveTo, this::performAction);
    } else {
      return ObjectiveActionMachine.createRightSideCommand(alliance, actions, drive::driveTo, this::performAction);
    }
  }
}

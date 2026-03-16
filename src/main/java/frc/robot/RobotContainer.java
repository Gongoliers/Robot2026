// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.PosePublisher;
import frc.lib.Telemetry;
import frc.robot.azimuth.Azimuth;
import frc.robot.drive.Drive;
import frc.robot.hood.Hood;
import frc.robot.scripting.*;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeState;
import frc.robot.kicker.Kicker;
import frc.robot.kicker.KickerState;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.ShooterTester;
import frc.robot.spindexer.Spindexer;
import frc.robot.spindexer.SpindexerState;
import frc.robot.turret.Turret;

import java.util.Arrays;
import java.util.List;
import java.util.function.Function;

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

  private  final ScriptingChooser chooser;

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

    chooser = new ScriptingChooser(8);
    chooser.publishActions(SmartDashboard::putData);
    chooser.publishSide(SmartDashboard::putData);
    SmartDashboard.putString("Auto/CurrentAction", "");

    multithreader.start();

    PosePublisher.publish("Auto/AllNamedPoses", Arrays.stream(NamedPose.values()).map(NamedPose::blue).toArray(Pose2d[]::new));

    PosePublisher.publish("Auto/LeftSweep", NamedPose.NEUTRAL_LEFT_SWEEP.blue());
    PosePublisher.publish("Auto/RightSweep", NamedPose.NEUTRAL_RIGHT_SWEEP.blue());

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

  private Command performDrive(Pose2d pose) {
    return Commands.sequence(
        Commands.runOnce(() -> PosePublisher.publish("Auto/NextPose", pose)),
        drive.driveTo(pose)
    );
  }

  private Command performAction(Action action) {
    return switch (action) {
        case NONE, PASS, CLIMB -> Commands.waitSeconds(2.5);
        // TODO Implement `intake.intake() -> Command`
        case INTAKE_NEUTRAL, INTAKE_ZONE, INTAKE_SWEEP -> intake.setState(IntakeState.OUT).repeatedly().finallyDo(intake::stow);
        // TODO Implement `turret.score() -> Command`
        case SCORE -> turret.faceHub().withTimeout(2.5);
    };
  }

  private Command loggedPerformAction(Action action) {
    return Commands.sequence(
            Commands.runOnce(() -> SmartDashboard.putString("Auto/CurrentAction", action.name())),
            performAction(action),
            Commands.runOnce(() -> SmartDashboard.putString("Auto/CurrentAction", ""))
    );
  }

  private ObjectiveAutoBuilder.AutoComposers.AutoComposer compose(Action action) {
    return switch (action) {
      case NONE, PASS, CLIMB, SCORE -> ObjectiveAutoBuilder.AutoComposers.AFTER_DRIVING;
      case INTAKE_NEUTRAL, INTAKE_ZONE, INTAKE_SWEEP -> ObjectiveAutoBuilder.AutoComposers.WHILE_DRIVING;
    };
  }

  public Command getAutonomousCommand() {
    DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    Action[] actions = chooser.selectedActions().toArray(Action[]::new);
    Objective initial = chooser.selectedLeftSide() ? Objective.INITIAL_LEFT : Objective.INITIAL_RIGHT;

    initial.pose().ifPresent(pose -> {
      if (Robot.isSimulation()) {
        drive.resetPose(pose.forAlliance(alliance));
      }
    });

    List<Objective> objectives = Objective.walk(initial, actions);

    ObjectiveAutoBuilder auto = new ObjectiveAutoBuilder(
        this::performDrive,
        this::loggedPerformAction,
        this::compose
    );

    Function<DriverStation.Alliance, Command> sequence = auto.createSequence(objectives);
    return ObjectiveAutoBuilder.explain(objectives).andThen(sequence.apply(alliance));
  }
}

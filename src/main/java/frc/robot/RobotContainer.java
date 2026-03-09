// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Climber.Setpoints;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.SwerveDrive;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.ShootAndIndexCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer
{

  private final ShooterSubsystem      shooter   = new ShooterSubsystem();
  private final ClimbSubsystem        climb   = new ClimbSubsystem();
  private final IndexerSubsystem      indexer = new IndexerSubsystem();
  private final IntakeRollerSubsystem intake  = new IntakeRollerSubsystem();
  private final SwerveSubsystem       drivebase = new SwerveSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(0);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverController.getLeftY() * -1,

                                                                () -> driverController.getLeftX() * -1) // set to 0
                                                            .withControllerRotationAxis(() ->
                                                                                            driverController.getRightX() *
                                                                                            -1) // driverController::getRightX
                                                            .deadband(.1)
                                                            .scaleTranslation(.8)
                                                            .allianceRelativeControl(true);


  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveAngularVelocity);


  public RobotContainer()
  {
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    shooter.setDefaultCommand(shooter.setDutycycleCommand(0));
    climb.setDefaultCommand(climb.setDutycyleCommand(0));
    indexer.setDefaultCommand(indexer.setDutycycleCommand(0.0));
    intake.setDefaultCommand(intake.setDutyCycleCommand(0.0));

    new EventTrigger("StartIntake").onTrue(intake.setVelocityCommand(Intake.Setpoints.intakeRPM));
    new EventTrigger("StopIntake").onTrue(intake.setDutyCycleCommand(0));
    NamedCommands.registerCommand("ShootBalls",
                                  shooter.setVelocityCommand(Shooter.Setpoints.autonomousPeriodRPM)
                                         .withTimeout(Seconds.of(3)));
    configureBindings();
  }


  private void configureBindings()
  {

    driverController.a().whileTrue(climb.setAngleCommand(Setpoints.climbSetpoint));
    driverController.b().whileTrue(climb.setAngleCommand(Setpoints.releaseSetpoint));
    // Prevents Swerve Drive from moving by making an X
    driverController.x().whileTrue(drivebase.lock());

    // Intake and outtake controls.
    driverController.rightBumper().whileTrue(intake.setVelocityCommand(Intake.Setpoints.intakeRPM));
    driverController.leftBumper().whileTrue(intake.setVelocityCommand(Intake.Setpoints.outtakeRPM));

    // Shooting commands
    driverController.rightTrigger().whileTrue(new ShootAndIndexCommand(intake,
                                                                       indexer,
                                                                       shooter,
                                                                       Shooter.Setpoints.midRPM));
    driverController.start().whileTrue(new ShootAndIndexCommand(intake, indexer, shooter, Shooter.Setpoints.hubRPM));
    driverController.y().whileTrue(new ShootAndIndexCommand(intake, indexer, shooter, drivebase));

    driverController.start().and(driverController.back()).onTrue(drivebase.zeroGyroWithAllianceCommand());
    //auto aim
    driverController.povUp().onTrue(drivebase.resetOdometryCommand(SwerveDrive.Setpoints.robotPoseAtDepot));

    driverController.leftTrigger().whileTrue(new AutoAimCommand(drivebase, driveAngularVelocity));

  }


  public Command getAutonomousCommand()
  {
    return drivebase.getAutonomousCommand("Auto1");
  }
}
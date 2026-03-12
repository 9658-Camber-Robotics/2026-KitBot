// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.Shooter.Setpoints;
import frc.robot.Constants.SwerveDrive;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ShootAndIndexCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer
{

  private final ShooterSubsystem shooter   = new ShooterSubsystem();
  private final ClimbSubsystem   climb     = new ClimbSubsystem();
  private final IndexerSubsystem indexer   = new IndexerSubsystem();
  private final SwerveSubsystem  drivebase = new SwerveSubsystem();

  private final CommandPS5Controller driverController   = new CommandPS5Controller(0);
  private final CommandPS5Controller operatorController = new CommandPS5Controller(1);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverController.getLeftY() * -1,

                                                                () -> driverController.getLeftX() * -1) // set to 0
                                                            .withControllerRotationAxis(() ->
                                                                                            driverController.getRightX() *
                                                                                            -1)
                                                            .deadband(.1)
                                                            .scaleTranslation(.8)
                                                            .allianceRelativeControl(true);


  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveAngularVelocity);


  public RobotContainer()
  {
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    shooter.setDefaultCommand(shooter.setVelocityCommand(() -> Setpoints.maxRPM.times(MathUtil.clamp(operatorController.getRightY(),
                                                                                                     0,
                                                                                                     1))));
    climb.setDefaultCommand(climb.setDutycyleCommand(operatorController::getLeftY));
    indexer.setDefaultCommand(indexer.setDutycycleCommand(0));

    new EventTrigger("StartIntake").onTrue(new IntakeCommand(indexer, shooter));
    new EventTrigger("StopIntake").onTrue(new OuttakeCommand(indexer, shooter));
    NamedCommands.registerCommand("ShootBalls",
                                  shooter.setVelocityCommand(Shooter.Setpoints.autonomousPeriodRPM)
                                         .withTimeout(Seconds.of(4)));
    configureBindings();
  }


  private void configureBindings()
  {
    // Shooting commands
    operatorController.triangle().whileTrue(new ShootAndIndexCommand(indexer, shooter, Setpoints.lowRPM));
    operatorController.circle().whileTrue(new ShootAndIndexCommand(indexer, shooter, Setpoints.midRPM));
    operatorController.cross().whileTrue(new ShootAndIndexCommand(indexer, shooter, Setpoints.high));
    operatorController.square().whileTrue(new ShootAndIndexCommand(indexer, shooter, Setpoints.maxRPM));
    operatorController.L2().whileTrue(new AutoAimCommand(drivebase, driveAngularVelocity, 0.4));

    // auto-aim
    operatorController.R2().whileTrue(new ShootAndIndexCommand(indexer, shooter, drivebase));

    // Intake and outtake controls.
    // TODO: Tune later
    operatorController.R1().whileTrue(new IntakeCommand(indexer, shooter));
    operatorController.L1().whileTrue(new OuttakeCommand(indexer, shooter));

    // Shooting commands

    // Prevents Swerve Drive from moving by making an X
    driverController.touchpad().whileTrue(drivebase.lock());
    driverController.create().and(driverController.touchpad()).onTrue(drivebase.zeroGyroWithAllianceCommand());
    // Reset odom on field to known points.
    driverController.povUp().onTrue(drivebase.resetOdometryCommand(SwerveDrive.Setpoints.robotPoseAtOutpost));
    driverController.povDown().onTrue(drivebase.resetOdometryCommand(SwerveDrive.Setpoints.robotPoseAtHub));


  }


  public Command getAutonomousCommand()
  {
    return drivebase.getAutonomousCommand("Basic");
  }
}
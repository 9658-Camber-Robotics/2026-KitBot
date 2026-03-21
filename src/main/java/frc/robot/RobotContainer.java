// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.Shooter.Setpoints;
import frc.robot.Constants.SwerveDrive;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ShootAndIndexCommand;
//import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer
{

  private final ShooterSubsystem shooter   = new ShooterSubsystem();
  //private final ClimbSubsystem   climb     = new ClimbSubsystem();
  private final IndexerSubsystem indexer   = new IndexerSubsystem();
  private final SwerveSubsystem  drivebase = new SwerveSubsystem();

  private final CommandXboxController driverController   = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final CommandXboxController testController = new CommandXboxController(2);
  
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverController.getLeftY() * -1,

                                                                () -> driverController.getLeftX() * -1) // set to 0
                                                            .withControllerRotationAxis(() ->
                                                                                            driverController.getRightX() *
                                                                                            1)
                                                            .deadband(.1)

                                                             
                                                            .scaleTranslation(.8)

                                                            .allianceRelativeControl(true);


  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveAngularVelocity);


  public RobotContainer()
  {
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    shooter.setDefaultCommand(shooter.setVelocityCommand(() -> Setpoints.maxRPM.times(MathUtil.clamp(MathUtil.applyDeadband(-operatorController.getRightY(),0.1 ),
                                                                                                     0,
                                                                                                     1))));
    //climb.setDefaultCommand(climb.setDutycyleCommand(operatorController::getLeftY));
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
    operatorController.a().whileTrue(new ShootAndIndexCommand(indexer, shooter, Setpoints.lowRPM));
    operatorController.b().whileTrue(new ShootAndIndexCommand(indexer, shooter, Setpoints.midRPM));
    operatorController.x().whileTrue(new ShootAndIndexCommand(indexer, shooter, Setpoints.high));
    operatorController.y().whileTrue(new ShootAndIndexCommand(indexer, shooter, Setpoints.maxRPM));
    operatorController.rightTrigger(0.3).whileTrue(new ShootAndIndexCommand(indexer, shooter, drivebase));
    operatorController.povUp().whileTrue(indexer.setDutycycleCommand(-0.8));
    operatorController.povDown().whileTrue(indexer.setDutycycleCommand(0.8));
    operatorController.povLeft().whileTrue(shooter.setDutycycleCommand(-0.8));
    operatorController.povRight().whileTrue(shooter.setDutycycleCommand(0.8));


    // auto-aim
    driverController.leftTrigger(0.3).whileTrue(new AutoAimCommand(drivebase, driveAngularVelocity, 0.4));
    
    testController.x().whileTrue(new ShootAndIndexCommand(indexer, shooter, RPM.of(2750))); 
    testController.y().whileTrue(new ShootAndIndexCommand(indexer, shooter, RPM.of(3000)));
    testController.b().whileTrue(new ShootAndIndexCommand(indexer, shooter, RPM.of(3250)));    
    testController.a().whileTrue(new ShootAndIndexCommand(indexer, shooter, RPM.of(3500)));
    testController.rightBumper().whileTrue(new ShootAndIndexCommand(indexer, shooter, RPM.of(3750)));
    testController.leftBumper().whileTrue(new ShootAndIndexCommand(indexer, shooter, RPM.of(4000)));
    testController.rightTrigger().whileTrue(new ShootAndIndexCommand(indexer, shooter, drivebase)); //based on distance from hub
    
    // Intake and outtake controls.
    // TODO: Tune later
    operatorController.rightBumper().whileTrue(new IntakeCommand(indexer, shooter));
    operatorController.leftBumper().whileTrue(new OuttakeCommand(indexer, shooter));

    // Shooting commands

    // Prevents Swerve Drive from moving by making an X
    driverController.x().whileTrue(drivebase.lock());
    driverController.back().and(driverController.start()).onTrue(drivebase.zeroGyroWithAllianceCommand());
    // Reset odom on field to known points.
    driverController.povUp().onTrue(drivebase.resetOdometryCommand(SwerveDrive.Setpoints.robotPoseAtHub));
    driverController.povDown().onTrue(drivebase.resetOdometryCommand(SwerveDrive.Setpoints.robotPoseAtOutpost));


  }


  public Command getAutonomousCommand()
  {
    return Commands.none(); // drivebase.getAutonomousCommand("SimpleAuto");
  }
}
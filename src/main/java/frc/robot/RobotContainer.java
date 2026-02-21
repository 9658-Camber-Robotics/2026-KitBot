// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import swervelib.SwerveInputStream;

public class RobotContainer {
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final ClimbSubsystem climb = new ClimbSubsystem();
    private final KickerSubsystem kicker = new KickerSubsystem();
    private final IntakeRollerSubsystem intake = new IntakeRollerSubsystem();
    private final SwerveSubsystem drivebase = new SwerveSubsystem();

    private final CommandXboxController driverController = new CommandXboxController(0);

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                    () -> driverController.getLeftY() * -1,

                    () -> driverController.getLeftX() * -1) // set to 0
            .withControllerRotationAxis(() -> driverController.getRightX() * -1) // driverController::getRightX
            .deadband(.1)
            .scaleTranslation(.8)
            .allianceRelativeControl(true);


    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveAngularVelocity);

    Pose2d blueHub = new Pose2d(new Translation2d(4, 4), Rotation2d.kZero);
    Pose2d redHub = new Pose2d(new Translation2d(18, 4), Rotation2d.kZero);

    public RobotContainer() {
        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
        shooter.setDefaultCommand(shooter.set(0));
        climb.setDefaultCommand(shooter.set(0));
        kicker.setDefaultCommand(shooter.set(0));
        intake.setDefaultCommand(shooter.set(0));
        configureBindings();
    }


    private void configureBindings() {

        driverController.a().whileTrue(climb.up());
        driverController.b().whileTrue(climb.down());

        driverController.rightBumper().whileTrue(intake.out());
        driverController.leftBumper().whileTrue(intake.in());

        driverController.rightTrigger().whileTrue(shooter.shoot());

        //auto aim
        driverController.leftTrigger().whileTrue(Commands.startRun(() -> {
            driveAngularVelocity.aim(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue ? blueHub : redHub);
            driveAngularVelocity.aimWhile(true);
        }, () -> {
        }).finallyDo(() -> driveAngularVelocity.aimWhile(false)));

        driverController.povDown().whileTrue(drivebase.aimDown(driveAngularVelocity));
        driverController.povUp().whileTrue(drivebase.aimUp(driveAngularVelocity));
        driverController.povLeft().whileTrue(drivebase.aimLeft(driveAngularVelocity));
        driverController.povRight().whileTrue(drivebase.aimRight(driveAngularVelocity));
    }

    //    public Command whenShooterSpeed("speed")thenKickerShoots;
    public Command shoot(AngularVelocity speed) {
        return Commands.parallel(shooter.setVelocity(speed), Commands.waitUntil(shooter.isNear(speed)).andThen(kicker.indexFuel()).repeatedly());
        // for the shooter
        // fire one ball, shooter speed drops to expected point with debounce
        // stops so feeder can stop; reverse accelerate
        // shooter gets up to speed (start accerelator) start feeder, go back to top


        // create debounce stuff
        // DigitalInput input = new Digital Input (0);
        // need a debouncer
        // Debouncer m_debouncer = new Debouncer (0.1, Debouncer.DebounceType.kBoth);
        // false signal --> if(m_debouncer.calculate input.get()));

    }

    public Command getAutonomousCommand() {
        return null;
    }
}
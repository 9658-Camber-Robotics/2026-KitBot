// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import swervelib.SwerveInputStream;

public class RobotContainer
{
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final ClimbSubsystem climb = new ClimbSubsystem();
    private final HopperSubsystem hopper = new HopperSubsystem();
    private final IntakeRollerSubsystem intake = new IntakeRollerSubsystem();
    private final SwerveSubsystem drivebase = new SwerveSubsystem();

    private final CommandXboxController driverController = new CommandXboxController(0);

    public RobotContainer()
    {
        configureBindings();
    }

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                    () -> driverController.getLeftY() *-1 ,

                    () -> driverController.getLeftX() * -1) // set to 0
            .withControllerRotationAxis(() -> driverController.getRightX() * -1) // driverController::getRightX
            .deadband(.1)
            .scaleTranslation(.8)
            .allianceRelativeControl(true);

    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverController::getRightX,
                    driverController::getRightY)
            .headingWhile(true);

    Command driveFieldOrientedDriectAngle = drivebase.driveFieldOriented(driveDirectAngle);

    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
//Non reality code

    SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX())
            .withControllerRotationAxis(() -> driverController.getRawAxis(2))
            .deadband(.1)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);
    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
            .withControllerHeadingAxis(() -> Math.sin(
                            driverController.getRawAxis(
                                    2) * Math.PI) * (Math.PI * 2),
                    () -> Math.cos(
                            driverController.getRawAxis(
                                    2) * Math.PI) *
                            (Math.PI * 2))
            .headingWhile(true);

    Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

    Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

    private void configureBindings() {

    }

    public Command getAutonomousCommand()
    {
        return null;
    }
}
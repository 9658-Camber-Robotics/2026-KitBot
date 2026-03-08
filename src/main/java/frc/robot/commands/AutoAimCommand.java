package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.FieldConstants.Hub;
import swervelib.SwerveInputStream;


public class AutoAimCommand extends Command
{

  private final SwerveSubsystem   swerveSubsystem;
  private final SwerveInputStream swerveInputStream;

  public AutoAimCommand(SwerveSubsystem swerveSubsystem, SwerveInputStream swerveInputStream)
  {
    this.swerveSubsystem = swerveSubsystem;
    this.swerveInputStream = swerveInputStream.copy();
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.swerveSubsystem);
  }

  @Override
  public void initialize()
  {
    swerveInputStream.aim(AllianceFlipUtil.apply(new Pose2d(Hub.topCenterPoint.toTranslation2d(), Rotation2d.kZero)))
                     .aimWhile(true)
                     .scaleTranslation(0.3);

  }

  @Override
  public void execute()
  {
    swerveSubsystem.driveFieldOrientedSetpoint(swerveInputStream.get());
  }

  @Override
  public boolean isFinished()
  {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted)
  {
    swerveInputStream.aimWhile(false)
                     .scaleTranslation(1);
  }
}

package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Kicker;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.Shooter.Setpoints;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class ShootKickAndIndexCommand extends Command
{

  private final IntakeRollerSubsystem intakeRollerSubsystem;
  private final KickerSubsystem       kickerSubsystem;
  private final ShooterSubsystem      shooterSubsystem;

  private final AngularVelocity goal;
//  private final Timer timer = new Timer();

  public ShootKickAndIndexCommand(IntakeRollerSubsystem intakeRollerSubsystem, KickerSubsystem kickerSubsystem,
                                  ShooterSubsystem shooterSubsystem, AngularVelocity goalRPM)
  {
    this.intakeRollerSubsystem = intakeRollerSubsystem;
    this.kickerSubsystem = kickerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.goal = goalRPM;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.intakeRollerSubsystem, this.kickerSubsystem, this.shooterSubsystem);
  }

  @Override
  public void initialize()
  {
    kickerSubsystem.setVelocity(Kicker.Setpoints.kickingSpeed);
    intakeRollerSubsystem.setDutyCycle(0);
    shooterSubsystem.setVelocity(Kicker.Setpoints.kickingSpeed);
  }

  @Override
  public void execute()
  {
    if (Shooter.flyWheelRecoveryDebouncer.calculate(shooterSubsystem.getVelocity().isNear(goal, Setpoints.tolerance)))
    {
      /*
      timer.stop();
      timer.reset();
      kickerSubsystem.setVelocity(Kicker.Setpoints.kickingSpeed);
       */
      intakeRollerSubsystem.setDutyCycleCommand(0.5);
    } else
    {
      /*
      if(timer.isRunning() && timer.hasElapsed(Seconds.of(0.1)))
      {
        kickerSubsystem.setVelocity(Kicker.Setpoints.kickingSpeed);
      } else {
        timer.start();
        // Set the kicker to go in the opposite direction at half power.
        // We may want to put a motion profile here.
        kickerSubsystem.setVelocity(Kicker.Setpoints.kickingSpeed.times(-0.5));
      }*/

      intakeRollerSubsystem.setDutyCycleCommand(0);
    }
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
    kickerSubsystem.setDutycycle(0);
    shooterSubsystem.setDutycycle(0);
    intakeRollerSubsystem.setDutyCycle(0);
  }
}

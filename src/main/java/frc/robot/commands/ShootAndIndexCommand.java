package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Indexer;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.Shooter.Setpoints;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;


public class ShootAndIndexCommand extends Command
{

  private final IntakeRollerSubsystem intakeRollerSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final ShooterSubsystem      shooterSubsystem;

  private final Supplier<AngularVelocity> goal;
  private final List<Data>                 shots   = List.of(
      new Data(Meters.of(0.5), RPM.of(3000))
                                                            );
  private final InterpolatingDoubleTreeMap shotMap = InterpolatingDoubleTreeMap.ofEntries(shots.stream()
                                                                                               .map(Data::toEntry)
                                                                                               .toArray(Map.Entry[]::new));

  public ShootAndIndexCommand(IntakeRollerSubsystem intakeRollerSubsystem, IndexerSubsystem indexerSubsystem,
                              ShooterSubsystem shooterSubsystem, Supplier<AngularVelocity> goalRPM)
  {
    this.intakeRollerSubsystem = intakeRollerSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.goal = goalRPM;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.intakeRollerSubsystem, this.indexerSubsystem, this.shooterSubsystem);
  }

  public ShootAndIndexCommand(IntakeRollerSubsystem intakeRollerSubsystem, IndexerSubsystem indexerSubsystem,
                              ShooterSubsystem shooterSubsystem, AngularVelocity goalRPM)
  {
    this(intakeRollerSubsystem, indexerSubsystem, shooterSubsystem, () -> goalRPM);
  }

  public ShootAndIndexCommand(IntakeRollerSubsystem intakeRollerSubsystem, IndexerSubsystem indexerSubsystem,
                              ShooterSubsystem shooterSubsystem, SwerveSubsystem swerveSubsystem)
  {
    this.intakeRollerSubsystem = intakeRollerSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.goal = () -> {return RPM.of(shotMap.get(swerveSubsystem.distanceFromHubMeters()));};
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.intakeRollerSubsystem, this.indexerSubsystem, this.shooterSubsystem);
  }

  @Override
  public void initialize()
  {
    intakeRollerSubsystem.setDutyCycle(0);
    shooterSubsystem.setVelocity(goal.get());
  }

  @Override
  public void execute()
  {
    shooterSubsystem.setVelocity(goal.get());
    if (Shooter.flyWheelRecoveryDebouncer.calculate(shooterSubsystem.getVelocity()
                                                                    .isNear(goal.get(), Setpoints.tolerance)))
    {
      indexerSubsystem.setVelocity(Indexer.Setpoints.indexSpeed);

    } else
    {
      indexerSubsystem.setDutyCycleCommand(0);
    }
  }

  record Data(Distance distance, AngularVelocity velocity)
  {

    public Map.Entry<Double, Double> toEntry() {return Map.entry(distance.in(Meters), velocity.in(RPM));}
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
    indexerSubsystem.setDutycycle(0);
    shooterSubsystem.setDutycycle(0);
    intakeRollerSubsystem.setDutyCycle(0);
  }
}

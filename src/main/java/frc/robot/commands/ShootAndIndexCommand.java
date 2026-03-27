package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Indexer;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.Shooter.Setpoints;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;


public class ShootAndIndexCommand extends Command
{

  private final IndexerSubsystem indexerSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  private final Supplier<AngularVelocity>  goal;
  private final List<Data>                 shots   = List.of(
      // TUNE HERE
      new Data(Meters.of(8.41326147155525), RPM.of(3450), Second.of(0.64))

                                                            );

  private final InterpolatingDoubleTreeMap shotMap = InterpolatingDoubleTreeMap.ofEntries(shots.stream()
                                                                                               .map(Data::toEntry)
                                                                                               .toArray(Map.Entry[]::new));

  public ShootAndIndexCommand(IndexerSubsystem indexerSubsystem,
                              ShooterSubsystem shooterSubsystem, Supplier<AngularVelocity> goalRPM)
  {
    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.goal = goalRPM;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.indexerSubsystem, this.shooterSubsystem);
  }

  public ShootAndIndexCommand(IndexerSubsystem indexerSubsystem,
                              ShooterSubsystem shooterSubsystem, AngularVelocity goalRPM)
  {
    this(indexerSubsystem, shooterSubsystem, () -> goalRPM);
  }

  public ShootAndIndexCommand(IndexerSubsystem indexerSubsystem,
                              ShooterSubsystem shooterSubsystem, SwerveSubsystem swerveSubsystem)
  {
    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.goal = () -> {return RPM.of(shotMap.get(swerveSubsystem.distanceFromHubMeters()));};
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.indexerSubsystem, this.shooterSubsystem);
  }

  @Override
  public void initialize()
  {
    indexerSubsystem.setDutycycle(0);
//    shooterSubsystem.setVelocity(goal.get());
  }

  @Override
  public void execute()
  {
    shooterSubsystem.setVelocity(goal.get());
    if (Shooter.flyWheelRecoveryDebouncer.calculate(shooterSubsystem.getVelocity()
                                                                    .isNear(goal.get(), RPM.of(100))))
    {
//      indexerSubsystem.setVelocity(Indexer.Setpoints.indexSpeed);
      indexerSubsystem.setDutycycle(-0.3);


    } else
    {
      indexerSubsystem.setDutycycle(0);
    }
  }

  record Data(Distance distance, AngularVelocity velocity, Time timeOfFlight)
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
  }
}

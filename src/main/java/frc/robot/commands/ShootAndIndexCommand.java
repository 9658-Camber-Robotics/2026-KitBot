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
      new Data(Meters.of(2.65135), RPM.of(3450), Second.of(0.64)),
      new Data(Meters.of(2.867095988657588), RPM.of(3300), Second.of(0.68)),
      new Data(Meters.of(3.445), RPM.of(3500), Second.of(0.71)),
      new Data(Meters.of(3.4527), RPM.of(3550), Second.of(0.78)),
      new Data(Meters.of(3.6430114165296557), RPM.of(3800), Second.of(0.92)),
      new Data(Meters.of(3.721622502648553), RPM.of(3800), Second.of(0.97)),
      new Data(Meters.of(3.844716530369055), RPM.of(3800), Second.of(0.98)),
      new Data(Meters.of(4.06), RPM.of(3800), Second.of(1.01)),
      new Data(Meters.of(4.095096381790928), RPM.of(3900), Second.of(1.02)),
      new Data(Meters.of(4.13453), RPM.of(3900), Second.of(1.11)),
      new Data(Meters.of(4.571), RPM.of(4000), Second.of(1.12)),
      new Data(Meters.of(4.638), RPM.of(4000), Second.of(1.14)),
      new Data(Meters.of(5.057498750154967), RPM.of(4150), Second.of(1.14)),
      new Data(Meters.of(5.652219475166091), RPM.of(4300), Second.of(1.21))
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

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.Indexer;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SparkWrapper;

public class IndexerSubsystem extends SubsystemBase
{

  private SparkMax spark = new SparkMax(CANIDS.indexerCANID, MotorType.kBrushless);

  private SmartMotorController indexerSMC = new SparkWrapper(spark,
                                                             Indexer.motor,
                                                             Indexer.smc.clone().withSubsystem(this));

  private FlyWheel indexer = new FlyWheel(Indexer.config.clone().withSmartMotorController(indexerSMC));

    public void setDutycycle(double dutyCycle)
    {
      indexer.setDutyCycleSetpoint(dutyCycle);
    }

    public void setVelocity(AngularVelocity velocity)
    {
      indexer.setMechanismVelocitySetpoint(velocity);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
      indexer.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
      indexer.simIterate();
    }

  public Command setDutycycleCommand(double dutycycle)
  {
    return indexer.set(dutycycle);
  }

  public void setDutyCycleCommand(double dutycycle)
  {
    indexer.setDutyCycleSetpoint(dutycycle);
  }
}
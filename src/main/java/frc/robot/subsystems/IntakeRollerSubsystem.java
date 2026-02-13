package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeRollerSubsystem extends SubsystemBase
{

 // make SparkMax
 // make SmartMotorControllerConfig
 // make SmartMotorController
 // make DO NOT MAKE MECHANIMS

  public IntakeRollerSubsystem()
  {
    
  }

  @Override
  public void simulationPeriodic()
  {
    // SMC.simIterate

  }

  @Override
  public void periodic()
  {
    // SMC.updateTelemetry()
  }

  public Command setIntakeRoller(double speed)
  {
    return runOnce(() -> {
      m_smc.set(speed);
    });
  }

  public Command out(double speed)
  {
    return setIntakeRoller(speed * -1);
  }

  public Command in(double speed)
  {
    return setIntakeRoller(speed);
  }

  public Command stop(){
    return setIntakeRoller(0);
  }

  public Current getCurrent()
  {
    return m_smc.getStatorCurrent();
  }

  public boolean outtaking()
  {
    if(getCurrentCommand() != null)
        return getDutycycle() > 0.0 || getCurrentCommand().getName().equals("Outtake");
    return getDutycycle() > 0.0;
  }

  public double getDutycycle()
  {
    return m_smc.getDutycycle();
  }
}
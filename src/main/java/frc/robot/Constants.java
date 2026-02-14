// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import yams.gearing.MechanismGearing;

import static edu.wpi.first.units.Units.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class ElevatorConstants {

        public static final double Kp = 5;
        public static final double Ki = 0;
        public static final double Kd = 0;

        public static final double kS = 0.0; // volts (V)
        public static final double kG = 0.762; // volts (V)
        public static final double kV = 0.762; // volt per velocity (V/(m/s))
        public static final double kA = 0.0; // volt per acceleration (V/(m/s²))

        public static final Voltage MaxVolts = Volts.of(20);
        public static final MechanismGearing ElevatorGearBox = new MechanismGearing(25.0);
        public static final Distance DrumRadius = Meters.of((16 * Units.inchesToMeters(0.25)) / (2 * Math.PI));
        public static final Mass CarriageMass = Kilograms.of(4.0); // kg

        public static final Distance MinElevatorHeightMeters = 0.0;
        public static final Distance MaxElevatorHeightMeters = 10.25;
        

        public static final AngularVelocity MaxVelocity = 3.5;
        public static final AngularAcceleration MaxAcceleration = 2.5;

        public static Distance StartingHeight;
    }

    public static double maxSpeed;
}

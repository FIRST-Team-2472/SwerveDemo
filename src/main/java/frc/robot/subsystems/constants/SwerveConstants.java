package frc.robot.subsystems.constants;

import edu.wpi.first.math.util.Units;

public class SwerveConstants {
    //this is the max speed you allow the robot to drive, you can let the drivers choose 
    //or find the number by testing
    public static final double kPhysicalMaxSpeedMetersPerSecond = 4;

    //this is how fast the individual wheels turn, lower this number if its jittering
    //or raise it if you feel it to be too slow
    public static final double kModuleTurningP = 0.5;
    
    //this is the gear ratio of the drive motor you can find this on the module's datasheet
    public static final double kDriveMotorGearRatio = 1.0 / 6.75;
    //gear ratio of the turning motor
    public static final double kTurningMotorGearRatio = 1 / 12.8;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    //this is just calculating how far is traveled in one turn of the encoder
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

}

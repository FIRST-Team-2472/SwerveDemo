package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.constants.SwerveConstants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final SparkPIDController turningPidController;

    private CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffset;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        // this is the offset on the absolute encoder as the magnets arent always facing
        // the same way
        // we are just storing the number in a global variable here
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        // this is whether or not the encoder is reversed
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        absoluteEncoder = new CANcoder(absoluteEncoderId);
        // creating a settings object to give to the CANcoder
        CANcoderConfiguration config = new CANcoderConfiguration();
        // we choose to set the readings from 0-1, the other option is -.5 to .5
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        config.MagnetSensor.MagnetOffset = -absoluteEncoderOffset;
        // applying the settings to the CANcoder
        absoluteEncoder.getConfigurator().apply(config);

        driveMotor = new CANSparkMax(driveMotorId, CANSparkLowLevel.MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, CANSparkLowLevel.MotorType.kBrushless);

        // reseting the motors firmware setting so we can easily use new motors and dont
        // have to
        // worry about the settings
        driveMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        // we have encoder objects, and for easy access we are storing them in global
        // variables
        // we could just do driveMotor.getEncoder().getPosition() every time but this is
        // faster
        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        // setting the conversion factors for the encoders so odometry can work
        driveEncoder.setPositionConversionFactor(SwerveConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(SwerveConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(SwerveConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(SwerveConstants.kTurningEncoderRPM2RadPerSec);

        // this is how the motor angle is controlled
        // PID's control it smoothly, currently we only use P but we can add I and D if
        // needed
        turningPidController = turningMotor.getPIDController();
        // P sets the power based on how far we are off of the target
        // the further away the more power
        turningPidController.setP(SwerveConstants.kModuleTurningP);
        // we are setting the minimum and max input angles
        turningPidController.setPositionPIDWrappingMinInput(-Math.PI);
        turningPidController.setPositionPIDWrappingMaxInput(Math.PI);
        // wrapping allows the motor to continue spinning in the same direction
        // even if it passes the max input like changing from -175 to 175
        turningPidController.setPositionPIDWrappingEnabled(true);
        // PID's need the actual angle to know how far off they are so we give it the
        // encoder
        turningPidController.setFeedbackDevice(turningEncoder);

        driveMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
        turningMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
        // turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        // turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        // calls a method we made to set the encoders to thier correct spot when we
        // start
        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public SwerveModulePosition getPosition() {
        // we are just making an object storing the position of the module so WPIlib
        // can use it to control it
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAbsolutePosition()));
    }

    public double getAbsoluteEncoder() { // this is used for shuffleboard
        return absoluteEncoder.getAbsolutePosition().getValue();
    }

    public double getAbsolutePosition() {
        // converts from 0-360 to -PI to PI then applies abosluteEncoder offset and
        // reverse
        double angle = Units.degreesToRadians(absoluteEncoder.getAbsolutePosition().getValue());
        // angle flip the angle if it is reversed
        angle *= absoluteEncoderReversed ? -1 : 1;
        angle *= 360;
        // atan2 funtion range in -PI to PI, so it automaticaly converts (needs the sin
        // and cos to) any input angle to that range
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop(); // keeps it from flipping back forward when not moving
            return; // we return so we can skip the rest of the code
        }
        state = SwerveModuleState.optimize(state, getState().angle); // makes it so we can reverse the wheels instead of
                                                                     // spinning 180
        driveMotor.set(state.speedMetersPerSecond / SwerveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.getPIDController().setReference(state.angle.getRadians(), CANSparkBase.ControlType.kPosition);
        // turningMotor.set(turningPidController.calculate(getTurningPosition(),
        // state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}

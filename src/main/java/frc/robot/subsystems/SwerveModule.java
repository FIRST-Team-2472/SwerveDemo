package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


public class SwerveModule {
    
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final SparkMaxPIDController turningPidController;

    private CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderoffset;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, 
            int absoluteEncoderId, double absoluteEncoderoffset, boolean absoluteEncoderReversed){
        this.absoluteEncoderoffset = absoluteEncoderoffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId);
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        config.MagnetSensor.MagnetOffset = -absoluteEncoderoffset;
        absoluteEncoder.getConfigurator().apply(config);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad); 
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);        


        turningPidController = turningMotor.getPIDController();
        turningPidController.setP(ModuleConstants.kPTurning);
        turningPidController.setPositionPIDWrappingMinInput(-Math.PI);
        turningPidController.setPositionPIDWrappingMaxInput(Math.PI);
        turningPidController.setPositionPIDWrappingEnabled(true);
        turningPidController.setFeedbackDevice(turningEncoder);

        driveMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
        turningMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
        // turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        // turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        
        resetEncoders();
    }
    

    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }

    public double getTurningPosition(){
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity(){
        return turningEncoder.getVelocity();
    }
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAbsolutePosition()));
    }
    public double getAbsoluteEncoder(){ // this is used for shuffleboard
        return absoluteEncoder.getAbsolutePosition().getValue();
    }
    public double getUnfilteredPosition(){
        double angle = absoluteEncoder.getAbsolutePosition().getValue();
        return angle;
    }
    public double getAbsolutePosition() {
        // converts from 0-360 to -PI to PI then applies abosluteEncoder offset and
        // reverse
        double angle = Units.degreesToRadians(absoluteEncoder.getAbsolutePosition().getValue());
       // angle -= absoluteEncoderoffset;
       angle *= absoluteEncoderReversed ? -1 : 1;
       angle *= 360;
        // atan2 funtion range in -PI to PI, so it automaticaly converts (needs the sin
        // and cos to) any input angle to that range
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
    

    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsolutePosition());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stop(); // keeps it from flipping back forward when not moving
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle); // makes it so we can reverse the wheels instead of spinning 180
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.getPIDController().setReference(state.angle.getRadians(), CANSparkMax.ControlType.kPosition);
        // turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
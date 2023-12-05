package frc.robot.subsystems;

import org.opencv.core.Mat;

import com.revrobotics.CANEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.resources.TecbotSpeedController;

public class SwerveModule {
    private final TecbotSpeedController driveMotor;
    private final TecbotSpeedController turnMotor;

    private final CANEncoder driveEncoder;
    private final CANEncoder turnEncoder;

    private final PIDController turningPidController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final Rotation2d absoulteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turngMotorId, boolean driveMotorReversed, boolean turnMotorReversed, int absoluteEncoderId, Rotation2d absoulteEncoderOffset, boolean absoluteEncoderReversed){
        this.absoulteEncoderOffsetRad = absoulteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new TecbotSpeedController(RobotMap.driveTrainPorts[1], RobotMap.chassisMotor[0]);
        turnMotor = new TecbotSpeedController(RobotMap.driveTrainPorts[0], RobotMap.chassisMotor[0]);

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turnMotorReversed);

        driveEncoder = driveMotor.getCANSparkMax().getEncoder();
        turnEncoder = turnMotor.getCANSparkMax().getEncoder();

        driveEncoder.setPositionConversionFactor(Constants.ModulesConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(Constants.ModulesConstants.kDriveEncoderRPM2MeterPerSec);
        turnEncoder.setPositionConversionFactor(Constants.ModulesConstants.kTurningEncoderRot2Rad);
        turnEncoder.setVelocityConversionFactor(Constants.ModulesConstants.kDriveEncoderRPM2MeterPerSec);

        turningPidController = new PIDController(Constants.ModulesConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }

    public double getTurnPosition(){
        return turnEncoder.getPosition();
    }

    public double getDriveVelocity(){
    return driveEncoder.getVelocity();
    }

    public double getTurnVelocity(){
        return turnEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad(){
        double angle = absoluteEncoder.getVoltage() / RobotController.getCurrent5V();
        angle *= 2.0 * Math.PI;
        //angle -= absoulteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }

    public void setDesiredSatate(SwerveModuleState state){
        if (Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / Constants.ModulesConstants.kMaxSpeedMetersPerSecond);
        turnMotor.set(turningPidController.calculate(getTurnPosition(), state.angle.getRadians()));
    }

    public void stop(){
        driveMotor.set(0);
        turnMotor.set(0);
    }

}

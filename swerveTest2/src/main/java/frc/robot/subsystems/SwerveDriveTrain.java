package frc.robot.subsystems;

import java.io.Console;

import com.ctre.phoenixpro.controls.CoastOut;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDriveTrain extends SubsystemBase {
    private final SwerveModule frontRight = new SwerveModule(
        Constants.DriverConstants.kFrontRightDriveMotorPort,
        Constants.DriverConstants.kFrontRightTurnMotorPort, 
        Constants.DriverConstants.kFrontRightDriveEncoderReversed, 
        Constants.DriverConstants.kFrontRightTurningEncoderReversed,
        Constants.DriverConstants.kFrontRightDriveAbsoluteEncoderPort, 
        Constants.DriverConstants.kFrontRightDriveAbsoluteEncoderOffsetRad, 
        Constants.DriverConstants.kFrontRightDriveAbsoluteEncoderReversed);

        private final SwerveModule frontLeft = new SwerveModule(
            
        Constants.DriverConstants.kFrontLeftDriveMotorPort,
        Constants.DriverConstants.kFrontLeftTurnMotorPort, 
        Constants.DriverConstants.kFrontLeftDriveEncoderReversed, 
        Constants.DriverConstants.kFrontLeftTurningEncoderReversed,
        Constants.DriverConstants.kFrontLeftDriveAbsoluteEncoderPort, 
        Constants.DriverConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, 
        Constants.DriverConstants.kFrontLeftDriveAbsoluteEncoderReversed);

        private final SwerveModule backRight = new SwerveModule(
        Constants.DriverConstants.kBackRightDriveMotorPort,
        Constants.DriverConstants.kBackRightTurnMotorPort, 
        Constants.DriverConstants.kBackRightDriveEncoderReversed, 
        Constants.DriverConstants.kBackRightTurningEncoderReversed,
        Constants.DriverConstants.kBackRightDriveAbsoluteEncoderPort, 
        Constants.DriverConstants.kBackRightDriveAbsoluteEncoderOffsetRad, 
        Constants.DriverConstants.kBackRightDriveAbsoluteEncoderReversed);

        private final SwerveModule backLeft = new SwerveModule(
        Constants.DriverConstants.kBackLeftDriveMotorPort,
        Constants.DriverConstants.kBackLeftTurnMotorPort, 
        Constants.DriverConstants.kBackLeftDriveEncoderReversed, 
        Constants.DriverConstants.kBackLeftTurningEncoderReversed,
        Constants.DriverConstants.kBackLeftDriveAbsoluteEncoderPort, 
        Constants.DriverConstants.kBackLeftDriveAbsoluteEncoderOffsetRad, 
        Constants.DriverConstants.kBackLeftDriveAbsoluteEncoderReversed);

        public final AHRS gyro = new AHRS (SPI.Port.kMXP);

        public SwerveDriveTrain(){
            new Thread(() -> {
                try{
                    Thread.sleep(1000);
                    zeroHeading();
                } catch (Exception e){
                }
            }).start();
        }

        public void zeroHeading(){
            gyro.reset();
        }

        public double getHeading(){
            return Math.IEEEremainder(gyro.getAngle(), 360);
        }

        public Rotation2d getRotation(){
            return Rotation2d.fromDegrees(getHeading());
        }

        public void stopModules(){
            frontLeft.stop();
            frontRight.stop();
            backLeft.stop();
            backRight.stop();
        }

        public void setModulesStates(SwerveModuleState[] desiredStates){
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.ModulesConstants.kMaxSpeedMetersPerSecond);
            frontLeft.setDesiredSatate(desiredStates[0]);
            frontRight.setDesiredSatate(desiredStates[1]);
            backLeft.setDesiredSatate(desiredStates[2]);
            backRight.setDesiredSatate(desiredStates[3]);
        }
}

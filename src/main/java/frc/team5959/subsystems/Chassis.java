// By: Beatriz Marún 5959
package frc.team5959.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.*;
import com.pathplanner.lib.auto.AutoBuilder;

//prueba/////

public class Chassis extends SubsystemBase{

    Module frontLeft, frontRight, rearLeft, rearRight;

    Translation2d frontLeftTranslation = new Translation2d(Constants.Chassis.frontLeftTranslationX,Constants.Chassis.frontLeftTranslationY); //ubicaciones x,y de los modulos a partir del centro
    Translation2d frontRigthTranslation = new Translation2d(Constants.Chassis.frontRightTranslationX,Constants.Chassis.frontRightTranslationY);
    Translation2d rearLeftTranslation = new Translation2d(Constants.Chassis.rearLeftTranslationX, Constants.Chassis.rearLeftTranslationY); //medidas provisionales
    Translation2d rearRightTranslation = new Translation2d(Constants.Chassis.rearRightTranslationX, Constants.Chassis.rearRightTranslationY);

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftTranslation,frontRigthTranslation,rearLeftTranslation,rearRightTranslation); //muy imnportante mantener el orden establecido aquí
    
    SwerveModulePosition[] positions = {frontLeft.getPosition(), frontRight.getPosition(), rearLeft.getPosition(), rearRight.getPosition()};

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), positions, new Pose2d(0, 0, getRotation2d()));

    SwerveDrivePoseEstimator poseEstimator;
    AHRS navx = new AHRS(SPI.Port.kMXP);

    public Chassis(){

        frontLeft = new Module(0, 0, 0, 0, 0, 0); //parametros a cambiar
        frontRight = new Module(0, 0, 0, 0, 0, 0);
        rearLeft = new Module(0, 0, 0, 0, 0, 0);
        rearRight = new Module(0, 0, 0, 0, 0, 0);

        AutoBuilder.configureHolonomic(this::getPose2d, this::set0doPose, this::getChassisSpeeds, null, null, null, null);
    }
    //Chassis speed se manda a la kinemática y ahpi se divide entre los modulos
    //la cinemática ya sabe donde estan los modulos, entonces toma la velocudiad resultante del chassis y crea una lista de estados que está en el mismo orden de como se delcaró en las "kinematics"
    public void setChassisSpeeds(double xSpeed, double ySpeed, double zSpeed){ //x, y para joystick izq y rotación (z) en el derecho
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed,ySpeed,zSpeed));
    // el metodo de la cinematica genera una lista de estados de todos los modulos basados en las velocidades que estamos aquí
    //SwerveModuleStat[] es una lista que se generó de los 4 estados
       
        setModuleStates(states);
    }

    public ChassisSpeeds getChassisSpeeds(){
        return new ChassisSpeeds(0, 0, 0);
    }

    public void setFieldOrientedSpeed(double xSpeed, double ySpeed, double zSpeed){ //realmente se va usar este (es el que tiene FOD)
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, getRotation2d());

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Chassis.maxSpeed); //modifica, ajusta y reduce proporcionalmente los modulos
        setModuleStates(states);
    }

    public void setModuleStates (SwerveModuleState[] states){

        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        rearLeft.setDesiredState(states[2]);
        rearRight.setDesiredState(states[3]);

    }

    public double getAngle() {
        return navx.getAngle();
    }

    public Rotation2d getRotation2d(){
        return new Rotation2d(getAngle()); // gracias6107 (aportación de sebas)
    }

    public Pose2d getPose2d(){
        return odometry.getPoseMeters();
    }

    public void set0doPose (Pose2d pose){
        positions [0] = frontLeft.getPosition();
        positions [1] = frontRight.getPosition();
        positions [2] = rearLeft.getPosition();
        positions [3] = rearRight.getPosition();

        odometry.resetPosition(getRotation2d(), positions, pose);
    }
    @Override
    public void periodic(){ //sirve más para ver cosas minimas de tu subsistema, funciona incluso en modo deshabilitado
       NetworkTable table = NetworkTableInstance.getDefault().getTable("Limelight");
       double found = table.getEntry("tv").getDouble(0);
    
       
        double[] poseNums = new double[6];
        poseNums = table.getEntry("botpose_wpiblue").getDoubleArray(poseNums);
       
       Pose2d visionMeasurement = new Pose2d(poseNums[0], poseNums[1], getRotation2d());
       
        SmartDashboard.putNumber("Navx", getAngle());
        SmartDashboard.putNumber("pose x", getPose2d().getX());

        positions [0] = frontLeft.getPosition();
        positions [1] = frontRight.getPosition();
        positions [2] = rearLeft.getPosition();
        positions [3] = rearRight.getPosition();

        odometry.update(getRotation2d(), positions);

        if (found == 1){
           poseEstimator.addVisionMeasurement(visionMeasurement, Timer.getFPGATimestamp(), null);
        }
    }
}
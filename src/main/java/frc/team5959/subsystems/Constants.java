// By: Beatriz Marún 5959
package frc.team5959.subsystems;
//poder modificar cosas del robot sin irse a lo más específico
public class Constants {
    
    public static class Chassis{
        
        public static final double maxSpeed = 0.9;

        //Modulos 
        public static final double frontLeftTranslationX = -0.5;
        public static final double frontLeftTranslationY = 0.5;

        public static final double frontRightTranslationX = 0.5;
        public static final double frontRightTranslationY = 0.5;

        public static final double rearLeftTranslationX = -0.5;
        public static final double rearLeftTranslationY = -0.5;

        public static final double rearRightTranslationX = 0.5;
        public static final double rearRightTranslationY = -0.5;


        public static final double driveRevsToMeters = 4 * Math.PI / (29.27 * 8.14) * 1.98; //revoluciones a metros (cyberius)
    }


}

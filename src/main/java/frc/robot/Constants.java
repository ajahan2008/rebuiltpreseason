package frc.robot;

public final class Constants {
    
    public static class CTREDevices {
        public static final int canrange = 21;
        public static final int candleID = 22;
        public static final int candiID = 53;
    }

    public static class IntakeIDs {
        public static final int intakeMotor = 30;
        public static final int pivotMotor = 31;
    }

    public static class LL1Settings {
        // setpoints
        public static final double kXSetpoint = 0.77;
        public static final double kYSetpoint = 1.37;
        public static final double kRotSetpoint = -26.3;

        // tolerances
        public static final double kXTolerance = 0.01;
        public static final double kYTolerance = 0.01;
        public static final double kRotTolerance = 1;

        //times (i'd rather avoid these)
        public static final double dontSeeTagTime = 0.5;
        public static final double poseValidationTime = 0.3;

        // PID
        public static final double kXPID = 0.8;
        public static final double kYPID = 0.6;
        public static final double kRotPID = 0.048;
    }

    public static class LL2Settings {
        // setpoints
        public static final double kXSetpoint = -0.665;
        public static final double kYSetpoint = 1.37;
        public static final double kRotSetpoint = -40.5;

        // tolerances
        public static final double kXTolerance = 0.01;
        public static final double kYTolerance = 0.01;
        public static final double kRotTolerance = 1;

        //times (i'd rather avoid these)
        public static final double dontSeeTagTime = 0.5;
        public static final double poseValidationTime = 0.3;

        // PID
        public static final double kXPID = 0.8;
        public static final double kYPID = 0.6;
        public static final double kRotPID = 0.048;
    }

    

}
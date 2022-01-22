package frc.robot.util;


/**
 * @author Ryan Hirasaki
 */
public class MotionMagic {
    public final double kF;
    public final double kP;
    public final double kI;
    public final double kD;
    public final int kIntegralZone;
    public final double kPeakOutput;
    public final int kClosedLoopTime;

    /**
     * Object to hold PID related values
     * 
     * @param p Proportional Value
     * @param i Integral Value
     * @param d Derivative Value
     * @param f Feed Foward for the motor
     * @param integralZone
     * @param peakOutput Peak output between 0 to 1 to represent 0% to 100%
     * @param closedLoopTime Timeout for the PID closed loop
     */
    public MotionMagic(double p, double i, double d, double f, int integralZone, double peakOutput, int closedLoopTime) {
        kF = f;
        kP = p;
        kI = i;
        kD = d;
        kIntegralZone = integralZone;
        kPeakOutput = peakOutput;
        kClosedLoopTime = closedLoopTime;
    }

    /**
     * Minimal Constructor as not all values will need to be used
     * 
     * @param p Proportional Value
     * @param i Integral Value
     * @param d Derivative Value
     * @param f Feed Foward for the motor
     */
    public MotionMagic(double p, double i, double d, double f) {
        this(p, i, d, f, 0, 0, 0);
    }
}

package frc.AlectronaLib;
public class OPRSlewRateLimiter {
    private double maxRateLimit;
    private double jerkLimit;
    private double currRateLimit = 0.0;
    private double prevVal = 0.0;
    private double prevTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    public OPRSlewRateLimiter(double rateLimit, double jerkLimit) {
        this.maxRateLimit = rateLimit;
        this.jerkLimit = jerkLimit;
    }

    public double calculate(double input) {
        double currTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        double dt = currTime - prevTime;
        double output;

        // calculate the maximum change allowed this frame
        double maxChange = currRateLimit * dt;

        if (input > prevVal + maxChange) {
            output = prevVal + maxChange;
            currRateLimit = Math.min(maxRateLimit, currRateLimit + jerkLimit * dt);
        } else if (input < prevVal - maxChange) 
        {
            output = prevVal - maxChange;
            currRateLimit = Math.min(maxRateLimit, currRateLimit + jerkLimit * dt);
        } else 
        {
            double actualRate = dt > 0 ? Math.abs(input - prevVal) / dt : 0;
            
            output = input;
            currRateLimit = Math.min(maxRateLimit, actualRate); 
        }

        prevTime = currTime;
        prevVal = output;
        return output;
    }

    public void reset(double value) {
        prevVal = value;
        currRateLimit = 0;
        prevTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }
}
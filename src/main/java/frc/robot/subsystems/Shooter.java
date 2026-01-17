package frc.robot.subsystems;

public class Shooter {
    // formula for ballistic trajectory WITHOUT DRAG, change in the future to account for this if needed 
    private Double angleCalculator(Double v,  Double x, Double y) {
        return(Math.atan((Math.pow(v, 2) + Math.sqrt(Math.pow(v, 4) - 9.8 * (9.8 * Math.pow(x,2) + 2 * y * Math.pow(v,2))))/ (9.8 * x)));
    }

    
}

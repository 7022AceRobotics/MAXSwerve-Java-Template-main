package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightHelpersC extends SubsystemBase{
    double x = 0;
    double y = 0;
    double a = 0;
    double l = 0;

    double[] botpose = new double[6];

    public LimelightHelpersC(){}

    public double[] update(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tl = table.getEntry("tl");
        NetworkTableEntry pose = table.getEntry("botpose_orb_wpiblue");

        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        a = ta.getDouble(0.0);
        l = tl.getDouble(0.0);

        botpose = pose.getDoubleArray(botpose);
        
        return botpose;

    }
}

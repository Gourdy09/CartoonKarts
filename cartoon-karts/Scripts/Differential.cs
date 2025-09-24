using Godot;
using System;

public partial class Differential : Node
{
    private Gearbox gearbox;
    private CarEngine engine;
    float bias;
    [Export] public string driveTrainType = "AWD"; // AWD, FWD, RWD
    [Export] public float[] wheelTorques = new float[4]; // [FL, FR, BL, BR]

    public override void _Ready()
    {
        gearbox = GetParent().GetNode("Gearbox") as Gearbox;
        engine = GetParent().GetNode("Engine") as CarEngine;
    }

    public override void _PhysicsProcess(double delta)
    {
        bias = PlayerInput.Instance.steer;
        
        // Don't distribute torque when in neutral
        if (engine.isNeutral)
        {
            wheelTorques[0] = 0;
            wheelTorques[1] = 0;
            wheelTorques[2] = 0;
            wheelTorques[3] = 0;
            return;
        }
        
        // Distribute torque based on drivetrain type
        if (driveTrainType.Equals("FWD"))
        {
            wheelTorques[0] = gearbox.drivetrainTorque * (0.5f - 0.5f * bias);
            wheelTorques[1] = gearbox.drivetrainTorque * (0.5f + 0.5f * bias);
            wheelTorques[2] = 0;
            wheelTorques[3] = 0;
        }
        else if (driveTrainType.Equals("AWD"))
        {
            wheelTorques[0] = gearbox.drivetrainTorque / 2 * (0.5f - 0.5f * bias);
            wheelTorques[1] = gearbox.drivetrainTorque / 2 * (0.5f + 0.5f * bias);
            wheelTorques[2] = gearbox.drivetrainTorque / 2 * (0.5f - 0.5f * bias);
            wheelTorques[3] = gearbox.drivetrainTorque / 2 * (0.5f + 0.5f * bias);
        }
        else if (driveTrainType.Equals("RWD"))
        {
            wheelTorques[0] = 0;
            wheelTorques[1] = 0;
            wheelTorques[2] = gearbox.drivetrainTorque * (0.5f - 0.5f * bias);
            wheelTorques[3] = gearbox.drivetrainTorque * (0.5f + 0.5f * bias);
        }
    }
}
using Godot;
using System;

public partial class Differential : Node
{
    private Gearbox gearbox;
    float bias;
    [Export] public string driveTrainType; // AWD, FWD, RWD
    [Export] public float[] wheelTorques = new float[4]; // [FL, FR, BL, BR]

    public override void _Ready()
    {
        gearbox = GetParent().GetNode("Gearbox") as Gearbox;
    }

    public override void _PhysicsProcess(double delta)
    {
        bias = PlayerInput.Instance.steer;
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

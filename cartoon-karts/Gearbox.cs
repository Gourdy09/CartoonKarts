using Godot;
using System;

public partial class Gearbox : Node
{
    private CarEngine engine;
    private int currentGear;
    private float[] gearRatios = {3.54f, 2.05f, 1.36f, 1.00f, 0.82f};
    private float efficiency = 0.5f;

    [Export] public float drivetrainTorque { get; private set; }

    public override void _Ready()
    {
        engine = GetParent().GetNode("Engine") as CarEngine;
    }

    public override void _PhysicsProcess(double delta)
    {
        drivetrainTorque = engine.engineTorque * gearRatios[currentGear] * efficiency;
    }


}

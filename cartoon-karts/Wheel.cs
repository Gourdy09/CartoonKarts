using Godot;
using System;

public partial class Wheel : RigidBody3D
{
    private Differential differential;
    private RayCast3D groundingRaycast;

    [Export] public bool isFrontWheel = false;
    [Export] public float maxSteerAngle = 10f; // degrees
    [Export] public float lateralGrip = 150f;   // adjust for arcade vs realistc
    [Export] public float rollingResistance = 1f; // slows the car slightly
    [Export] public float steerSpeed = 120f; // degrees per second
    [Export] public float returnSpeed = 80f; // degrees per second when no input

    private float steerAngle = 0;

    public override void _Ready()
    {
        differential = GetParent().GetNode<Differential>("Differential");
        groundingRaycast = GetNode<RayCast3D>("RayCast3D");
        isFrontWheel = Name.Equals("FL") || Name.Equals("FR");
    }

    public override void _PhysicsProcess(double delta)
    {
        if (!groundingRaycast.IsColliding())
        {
            return;
        }
        Vector3 localForward = -GlobalTransform.Basis.Z;
        Vector3 right = GlobalTransform.Basis.X;

        // Steering
        if (isFrontWheel)
        {
            float steerInput = PlayerInput.Instance.steer;
            float targetAngle = maxSteerAngle * steerInput;

            if (Mathf.Abs(steerInput) > 0.01f)
            {
                // Turn toward target with limited rate
                steerAngle = Mathf.MoveToward(steerAngle, targetAngle, steerSpeed * (float)delta);
            }
            else
            {
                // Return to center when no input
                steerAngle = Mathf.MoveToward(steerAngle, 0, returnSpeed * (float)delta);
            }

            localForward = localForward.Rotated(Vector3.Up, Mathf.DegToRad(steerAngle));
        }

        // Lateral Grip
        // project velocity onto right vector to find sideways motion
        Vector3 lateralVel = LinearVelocity.Project(right);
        Vector3 gripForce = -lateralVel * lateralGrip;
        ApplyForce(gripForce);

        // Apply Torque to Wheels
        float wheelTorque = 0;
        
        switch (Name)
        {
            case "FL": wheelTorque = differential.wheelTorques[0]; break;
            case "FR": wheelTorque = differential.wheelTorques[1]; break;
            case "BL": wheelTorque = differential.wheelTorques[2]; break;
            case "BR": wheelTorque = differential.wheelTorques[3]; break;
        }

        ApplyForce(localForward * wheelTorque);

        // Rolling Resistance
        Vector3 forwardVel = LinearVelocity.Project(localForward);
        ApplyForce(-forwardVel * rollingResistance);
    }
}

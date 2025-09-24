using Godot;
using System;

public partial class Wheel : RigidBody3D
{
    private Differential differential;
    private RigidBody3D chassis; // Reference to main car body for speed-sensitive steering
    private RayCast3D groundingRaycast;

    [Export] public bool isFrontWheel = false;
    [Export] public float maxSteerAngle = 35f; // degrees - increased for better turning
    [Export] public float lateralGrip = 80f;   // increased for better handling
    [Export] public float rollingResistance = 2f; // slightly increased for more realistic feel
    [Export] public float brakeForce = 60f; // increased braking force
    [Export] public float steerSpeed = 180f; // degrees per second - faster steering
    [Export] public float returnSpeed = 120f; // degrees per second - faster return
    [Export] public float speedSensitiveSteeringMin = 0.3f; // minimum steering multiplier at high speed
    [Export] public float speedSensitiveSteeringThreshold = 50f; // km/h where steering starts reducing

    private float steerAngle = 0;

    public override void _Ready()
    {
        differential = GetParent().GetNode<Differential>("Differential");
        chassis = GetParent().GetNode<RigidBody3D>("Chassis"); // Get chassis for speed calculation
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

        // IMPROVED STEERING (only for front wheels)
        if (isFrontWheel)
        {
            float steerInput = PlayerInput.Instance.steer;
            
            // Speed-sensitive steering - less steering angle at high speeds
            float speed = chassis.LinearVelocity.Length() * 3.6f; // km/h
            float speedFactor = 1.0f;
            if (speed > speedSensitiveSteeringThreshold)
            {
                float speedRatio = (speed - speedSensitiveSteeringThreshold) / speedSensitiveSteeringThreshold;
                speedFactor = Mathf.Lerp(1.0f, speedSensitiveSteeringMin, Mathf.Clamp(speedRatio, 0f, 1f));
            }
            
            float effectiveMaxSteerAngle = maxSteerAngle * speedFactor;
            float targetAngle = effectiveMaxSteerAngle * steerInput;

            // Non-linear steering response for better feel
            float steerInputCurved = steerInput * steerInput * steerInput + steerInput * 0.5f;
            targetAngle = effectiveMaxSteerAngle * steerInputCurved;

            if (Mathf.Abs(steerInput) > 0.01f)
            {
                // Turn toward target with limited rate - faster at low speeds
                float currentSteerSpeed = steerSpeed * (speed < 20f ? 1.5f : 1.0f);
                steerAngle = Mathf.MoveToward(steerAngle, targetAngle, currentSteerSpeed * (float)delta);
            }
            else
            {
                // Return to center when no input - speed sensitive
                float currentReturnSpeed = returnSpeed * (speed < 20f ? 0.8f : 1.2f);
                steerAngle = Mathf.MoveToward(steerAngle, 0, currentReturnSpeed * (float)delta);
            }

            localForward = localForward.Rotated(Vector3.Up, Mathf.DegToRad(steerAngle));
        }

        // IMPROVED LATERAL GRIP - speed and load sensitive
        Vector3 lateralVel = LinearVelocity.Project(right);
        float lateralSpeed = lateralVel.Length();
        
        // Progressive grip loss at high slip angles (more realistic)
        float gripMultiplier = 1.0f;
        if (lateralSpeed > 5f) // Start losing grip above 5 m/s lateral speed
        {
            gripMultiplier = Mathf.Max(0.3f, 1.0f - (lateralSpeed - 5f) * 0.1f);
        }
        
        Vector3 gripForce = -lateralVel * lateralGrip * gripMultiplier;
        ApplyForce(gripForce);

        // Apply Engine Torque to Wheels
        float wheelTorque = 0;
        
        switch (Name)
        {
            case "FL": wheelTorque = differential.wheelTorques[0]; break;
            case "FR": wheelTorque = differential.wheelTorques[1]; break;
            case "BL": wheelTorque = differential.wheelTorques[2]; break;
            case "BR": wheelTorque = differential.wheelTorques[3]; break;
        }

        ApplyForce(localForward * wheelTorque);

        // IMPROVED BRAKING SYSTEM
        float brakeInput = PlayerInput.Instance.brake;
        if (brakeInput > 0)
        {
            // Progressive braking force based on speed
            float speed = LinearVelocity.Length();
            float speedMultiplier = Mathf.Clamp(speed / 10f, 0.3f, 1.0f); // Less braking at very low speeds
            
            Vector3 brakeDirection = -LinearVelocity.Normalized();
            Vector3 totalBrakeForce = brakeDirection * brakeForce * brakeInput * speedMultiplier;
            ApplyForce(totalBrakeForce);
            
            // ABS simulation - prevent wheel lockup
            if (lateralSpeed > 8f && brakeInput > 0.7f)
            {
                // Reduce brake force when sliding
                ApplyForce(-totalBrakeForce * 0.3f);
            }
        }

        // IMPROVED ROLLING RESISTANCE - speed dependent
        Vector3 forwardVel = LinearVelocity.Project(localForward);
        float forwardSpeed = forwardVel.Length();
        
        // Air resistance increases with speed squared
        float airResistance = forwardSpeed * forwardSpeed * 0.01f;
        float totalResistance = rollingResistance + airResistance;
        
        ApplyForce(-forwardVel * totalResistance);
    }
}
using Godot;
using System;

public partial class Wheel : RigidBody3D
{
    private Differential differential;
    private RigidBody3D chassis;
    private RayCast3D groundingRaycast;

    [Export] public bool isFrontWheel = false;
    [Export] public float maxSteerAngle = 30f;
    [Export] public float lateralGrip = 120f;
    [Export] public float rollingResistance = 0.8f; // Reduced for better coasting
    [Export] public float brakeForce = 80f;
    [Export] public float steerSpeed = 120f;
    [Export] public float returnSpeed = 90f;
    [Export] public float speedSensitiveSteeringMin = 0.4f;
    [Export] public float speedSensitiveSteeringThreshold = 40f;
    [Export] public float suspensionForce = 8000f;
    [Export] public float suspensionDamping = 800f;
    [Export] public float restLength = 0.3f;
    [Export] public float minimumRollingSpeed = 0.5f; // Below this speed, reduce rolling resistance
    [Export] public float airResistanceCoefficient = 0.005f; // Reduced air resistance

    private float steerAngle = 0;
    private Vector3 lastPosition;

    public override void _Ready()
    {
        differential = GetParent().GetNode<Differential>("Differential");
        chassis = GetParent().GetNode<RigidBody3D>("Chassis");
        groundingRaycast = GetNode<RayCast3D>("RayCast3D");
        isFrontWheel = Name.Equals("FL") || Name.Equals("FR");
        lastPosition = GlobalPosition;
    }

    public override void _PhysicsProcess(double delta)
    {
        // Apply custom suspension force
        ApplySuspension(delta);

        if (!groundingRaycast.IsColliding())
        {
            return;
        }

        Vector3 localForward = -GlobalTransform.Basis.Z;
        Vector3 right = GlobalTransform.Basis.X;
        Vector3 up = GlobalTransform.Basis.Y;

        // STEERING - Works in ALL conditions (neutral, drive, reverse)
        if (isFrontWheel)
        {
            HandleSteering(delta, up, ref localForward);
        }

        // TRACTION SYSTEM
        ApplyTraction(right, localForward);

        // ENGINE TORQUE - Only apply when not in neutral
        ApplyDrivetrainTorque(localForward);

        // BRAKING
        ApplyBraking();

        // IMPROVED ROLLING RESISTANCE - Allow better coasting
        ApplyRollingResistance(localForward);

        lastPosition = GlobalPosition;
    }

    private void HandleSteering(double delta, Vector3 up, ref Vector3 localForward)
    {
        float steerInput = PlayerInput.Instance.steer;
        
        // Speed-sensitive steering
        float speed = chassis.LinearVelocity.Length() * 3.6f; // km/h
        float speedFactor = 1.0f;
        if (speed > speedSensitiveSteeringThreshold)
        {
            float speedRatio = (speed - speedSensitiveSteeringThreshold) / speedSensitiveSteeringThreshold;
            speedFactor = Mathf.Lerp(1.0f, speedSensitiveSteeringMin, Mathf.Clamp(speedRatio, 0f, 1f));
        }
        
        float effectiveMaxSteerAngle = maxSteerAngle * speedFactor;
        
        // Progressive steering response
        float steerInputCurved = steerInput * Mathf.Abs(steerInput) * 0.8f + steerInput * 0.2f;
        float targetAngle = effectiveMaxSteerAngle * steerInputCurved;

        // Always respond to steering input regardless of engine state
        if (Mathf.Abs(steerInput) > 0.01f)
        {
            steerAngle = Mathf.MoveToward(steerAngle, targetAngle, steerSpeed * (float)delta);
        }
        else
        {
            steerAngle = Mathf.MoveToward(steerAngle, 0, returnSpeed * (float)delta);
        }

        // Apply steering rotation
        localForward = localForward.Rotated(up, Mathf.DegToRad(steerAngle));
    }

    private void ApplyTraction(Vector3 right, Vector3 localForward)
    {
        Vector3 wheelVelocity = LinearVelocity;
        Vector3 lateralVel = wheelVelocity.Project(right);
        Vector3 forwardVel = wheelVelocity.Project(localForward);
        
        float lateralSpeed = lateralVel.Length();
        float forwardSpeed = forwardVel.Length();
        
        // Progressive grip loss with slip angle
        float optimalSlipAngle = 8f; // degrees
        float currentSlipAngle = Mathf.RadToDeg(Mathf.Atan2(lateralSpeed, Mathf.Max(forwardSpeed, 0.1f)));
        
        float gripMultiplier = 1.0f;
        if (currentSlipAngle > optimalSlipAngle)
        {
            float excessSlip = currentSlipAngle - optimalSlipAngle;
            gripMultiplier = Mathf.Max(0.2f, 1.0f - (excessSlip / 30f));
        }

        // Apply lateral grip force (tire friction) - this works regardless of engine state
        Vector3 gripForce = -lateralVel * lateralGrip * gripMultiplier;
        ApplyForce(gripForce);
    }

    private void ApplyDrivetrainTorque(Vector3 localForward)
    {
        // Get engine reference to check if it's in neutral
        CarEngine engine = GetParent().GetNode<CarEngine>("Engine");
        
        // Only apply drivetrain torque when NOT in neutral
        if (!engine.isNeutral)
        {
            float wheelTorque = 0;
            switch (Name)
            {
                case "FL": wheelTorque = differential.wheelTorques[0]; break;
                case "FR": wheelTorque = differential.wheelTorques[1]; break;
                case "BL": wheelTorque = differential.wheelTorques[2]; break;
                case "BR": wheelTorque = differential.wheelTorques[3]; break;
            }

            // Apply torque in the direction the wheel is pointing
            ApplyForce(localForward * wheelTorque);
        }
        // In neutral, no engine torque is applied, allowing free rolling
    }

    private void ApplyBraking()
    {
        float brakeInput = PlayerInput.Instance.brake;
        if (brakeInput > 0)
        {
            Vector3 wheelVelocity = LinearVelocity;
            Vector3 brakeDirection = -wheelVelocity.Normalized();
            float brakeEffectiveness = Mathf.Clamp(wheelVelocity.Length() / 15f, 0.1f, 1.0f);
            Vector3 totalBrakeForce = brakeDirection * brakeForce * brakeInput * brakeEffectiveness;
            ApplyForce(totalBrakeForce);
        }
    }

    private void ApplyRollingResistance(Vector3 localForward)
    {
        Vector3 wheelVelocity = LinearVelocity;
        Vector3 forwardVel = wheelVelocity.Project(localForward);
        float forwardSpeed = forwardVel.Length();
        
        // Speed-dependent rolling resistance - less resistance at low speeds for better coasting
        float speedBasedRollingResistance = rollingResistance;
        if (forwardSpeed < minimumRollingSpeed)
        {
            // Reduce rolling resistance at very low speeds to allow coasting to a stop naturally
            speedBasedRollingResistance *= (forwardSpeed / minimumRollingSpeed) * 0.3f;
        }
        
        Vector3 resistanceForce = -forwardVel * speedBasedRollingResistance;
        
        // Air resistance increases with speed squared, but keep it minimal
        float airResistance = forwardSpeed * forwardSpeed * airResistanceCoefficient;
        resistanceForce -= forwardVel.Normalized() * airResistance;
        
        ApplyForce(resistanceForce);
    }

    private void ApplySuspension(double delta)
    {
        Vector3 suspensionDirection = -GlobalTransform.Basis.Y;
        
        if (groundingRaycast.IsColliding())
        {
            Vector3 hitPoint = groundingRaycast.GetCollisionPoint();
            float currentLength = (GlobalPosition - hitPoint).Length();
            float compression = restLength - currentLength;
            
            if (compression > 0)
            {
                // Spring force
                Vector3 springForce = suspensionDirection * compression * suspensionForce;
                
                // Damping force based on suspension velocity
                Vector3 suspensionVelocity = (GlobalPosition - lastPosition) / (float)delta;
                float dampingVelocity = suspensionVelocity.Dot(suspensionDirection);
                Vector3 dampingForce = -suspensionDirection * dampingVelocity * suspensionDamping;
                
                ApplyForce(springForce + dampingForce);
            }
        }
    }
}
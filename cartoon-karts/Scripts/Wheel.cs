using Godot;
using System;

public partial class Wheel : RigidBody3D
{
    private Differential differential;
    private RigidBody3D chassis;
    private RayCast3D groundingRaycast;

    [Export] public bool isFrontWheel = false;
    [Export] public float maxSteerAngle = 30f;
    [Export] public float lateralGrip = 300f; // Increased from 120 for better grip
    [Export] public float rollingResistance = 0.8f;
    [Export] public float brakeForce = 80f;
    [Export] public float steerSpeed = 120f;
    [Export] public float returnSpeed = 90f;
    [Export] public float speedSensitiveSteeringMin = 0.4f;
    [Export] public float speedSensitiveSteeringThreshold = 40f;
    
    // IMPROVED SUSPENSION PARAMETERS
    [Export] public float suspensionForce = 1500f; // Much softer for smooth ride
    [Export] public float suspensionDamping = 600f; // Moderate damping
    [Export] public float restLength = 0.4f; // Longer travel for better bump absorption
    [Export] public float maxCompressionForce = 3500f; // Lower cap for gentler response
    [Export] public float groundedThreshold = 0.1f; // How compressed before considered "grounded"
    
    // ANTI-ROLL PARAMETERS
    [Export] public float antiRollForce = 2000f; // Resistance to body roll
    [Export] public float antiRollDamping = 400f; // Damping for roll motion
    
    [Export] public float minimumRollingSpeed = 0.5f;
    [Export] public float airResistanceCoefficient = 0.005f;

    private float steerAngle = 0;
    private Vector3 lastPosition;
    private float lastCompression = 0f; // Track previous compression for damping calculation
    private bool wasGrounded = false; // Track if we were grounded last frame
    
    // Track opposite wheel for anti-roll
    private Wheel oppositeWheel = null;

    public override void _Ready()
    {
        differential = GetParent().GetNode<Differential>("Differential");
        chassis = GetParent().GetNode<RigidBody3D>("Chassis");
        groundingRaycast = GetNode<RayCast3D>("RayCast3D");
        isFrontWheel = Name.Equals("FL") || Name.Equals("FR");
        lastPosition = GlobalPosition;
        
        // Find opposite wheel for anti-roll bar simulation
        if (Name.Equals("FL"))
            oppositeWheel = GetParent().GetNode<Wheel>("FR");
        else if (Name.Equals("FR"))
            oppositeWheel = GetParent().GetNode<Wheel>("FL");
        else if (Name.Equals("BL"))
            oppositeWheel = GetParent().GetNode<Wheel>("BR");
        else if (Name.Equals("BR"))
            oppositeWheel = GetParent().GetNode<Wheel>("BL");
    }

    public override void _PhysicsProcess(double delta)
    {
        // Store previous compression before updating
        StorePreviousCompression();
        
        // Apply custom suspension force FIRST
        bool isGrounded = ApplySuspension(delta);

        if (!groundingRaycast.IsColliding())
        {
            wasGrounded = false;
            return;
        }

        Vector3 localForward = -GlobalTransform.Basis.Z;
        Vector3 right = GlobalTransform.Basis.X;
        Vector3 up = GlobalTransform.Basis.Y;

        // STEERING
        if (isFrontWheel)
        {
            HandleSteering(delta, up, ref localForward);
        }

        // TRACTION SYSTEM
        ApplyTraction(right, localForward);

        // ENGINE TORQUE - Only apply when properly grounded
        if (isGrounded)
        {
            ApplyDrivetrainTorque(localForward);
        }

        // BRAKING
        ApplyBraking();

        // ROLLING RESISTANCE
        ApplyRollingResistance(localForward);

        lastPosition = GlobalPosition;
        wasGrounded = isGrounded;
    }

    private void HandleSteering(double delta, Vector3 up, ref Vector3 localForward)
    {
        float steerInput = PlayerInput.Instance.steer;
        
        // Speed-sensitive steering
        float speed = chassis.LinearVelocity.Length() * 3.6f;
        float speedFactor = 1.0f;
        if (speed > speedSensitiveSteeringThreshold)
        {
            float speedRatio = (speed - speedSensitiveSteeringThreshold) / speedSensitiveSteeringThreshold;
            speedFactor = Mathf.Lerp(1.0f, speedSensitiveSteeringMin, Mathf.Clamp(speedRatio, 0f, 1f));
        }
        
        float effectiveMaxSteerAngle = maxSteerAngle * speedFactor;
        float steerInputCurved = steerInput * Mathf.Abs(steerInput) * 0.8f + steerInput * 0.2f;
        float targetAngle = effectiveMaxSteerAngle * steerInputCurved;

        if (Mathf.Abs(steerInput) > 0.01f)
        {
            steerAngle = Mathf.MoveToward(steerAngle, targetAngle, steerSpeed * (float)delta);
        }
        else
        {
            steerAngle = Mathf.MoveToward(steerAngle, 0, returnSpeed * (float)delta);
        }

        localForward = localForward.Rotated(up, Mathf.DegToRad(steerAngle));
    }

    private void ApplyTraction(Vector3 right, Vector3 localForward)
    {
        Vector3 wheelVelocity = LinearVelocity;
        Vector3 lateralVel = wheelVelocity.Project(right);
        Vector3 forwardVel = wheelVelocity.Project(localForward);
        
        float lateralSpeed = lateralVel.Length();
        float forwardSpeed = forwardVel.Length();
        
        // More aggressive grip curve for less sliding
        float optimalSlipAngle = 6f; // Reduced from 8 for earlier grip
        float currentSlipAngle = Mathf.RadToDeg(Mathf.Atan2(lateralSpeed, Mathf.Max(forwardSpeed, 0.1f)));
        
        float gripMultiplier = 1.0f;
        if (currentSlipAngle > optimalSlipAngle)
        {
            float excessSlip = currentSlipAngle - optimalSlipAngle;
            // More gradual falloff for controlled slides
            gripMultiplier = Mathf.Max(0.4f, 1.0f - (excessSlip / 40f));
        }

        // Apply stronger lateral grip force
        Vector3 gripForce = -lateralVel * lateralGrip * gripMultiplier;
        ApplyForce(gripForce);
        
        // Additional: Apply grip to chassis for better stability
        chassis.ApplyForce(gripForce * 0.3f, GlobalPosition - chassis.GlobalPosition);
    }

    private void ApplyDrivetrainTorque(Vector3 localForward)
    {
        CarEngine engine = GetParent().GetNode<CarEngine>("Engine");
        
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

            ApplyForce(localForward * wheelTorque);
        }
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
        
        float speedBasedRollingResistance = rollingResistance;
        if (forwardSpeed < minimumRollingSpeed)
        {
            speedBasedRollingResistance *= (forwardSpeed / minimumRollingSpeed) * 0.3f;
        }
        
        Vector3 resistanceForce = -forwardVel * speedBasedRollingResistance;
        float airResistance = forwardSpeed * forwardSpeed * airResistanceCoefficient;
        resistanceForce -= forwardVel.Normalized() * airResistance;
        
        ApplyForce(resistanceForce);
    }

    private bool ApplySuspension(double delta)
    {
        Vector3 suspensionDirection = -GlobalTransform.Basis.Y;
        
        if (!groundingRaycast.IsColliding())
        {
            lastCompression = 0f;
            return false;
        }

        Vector3 hitPoint = groundingRaycast.GetCollisionPoint();
        float currentLength = (GlobalPosition - hitPoint).Length();
        float compression = restLength - currentLength;
        
        // Calculate compression velocity for damping
        float compressionVelocity = (compression - lastCompression) / (float)delta;
        lastCompression = compression;
        
        // Wheel is grounded if raycast is hitting (even with no compression)
        bool isGrounded = true;
        
        // Only apply suspension when compressed
        if (compression > 0)
        {
            // Linear spring force for consistent, smooth response
            float springForce = compression * suspensionForce;
            
            // Velocity-based damping (resist both compression and extension)
            float dampingForce = -compressionVelocity * suspensionDamping;
            
            // ANTI-ROLL BAR: Transfer force between opposite wheels
            float antiRollTorque = 0f;
            if (oppositeWheel != null)
            {
                float compressionDifference = compression - oppositeWheel.lastCompression;
                
                // Anti-roll force proportional to compression difference
                antiRollTorque = compressionDifference * antiRollForce;
                
                // Anti-roll damping based on difference in compression velocity
                float oppositeCompressionVel = (oppositeWheel.lastCompression - oppositeWheel.GetPreviousCompression()) / (float)delta;
                float rollVelocity = compressionVelocity - oppositeCompressionVel;
                antiRollTorque += rollVelocity * antiRollDamping;
            }
            
            // Combine forces
            float totalForce = springForce + dampingForce + antiRollTorque;
            
            // Cap maximum force to prevent launches
            totalForce = Mathf.Clamp(totalForce, 0f, maxCompressionForce);
            
            // Apply force upward along suspension direction
            Vector3 suspensionForceVector = suspensionDirection * totalForce;
            ApplyForce(suspensionForceVector);
            
            // Additional: Apply downward force to chassis to balance
            chassis.ApplyForce(-suspensionForceVector, GlobalPosition - chassis.GlobalPosition);
        }
        
        return isGrounded;
    }
    
    // Helper to get previous compression for anti-roll calculation
    private float previousCompression = 0f;
    private float GetPreviousCompression()
    {
        return previousCompression;
    }
    
    private void StorePreviousCompression()
    {
        previousCompression = lastCompression;
    }
}
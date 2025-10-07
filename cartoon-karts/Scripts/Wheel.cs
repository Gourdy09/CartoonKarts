using Godot;
using System;

public partial class Wheel : RigidBody3D
{
    private Differential differential;
    private RigidBody3D chassis;
    private RayCast3D groundingRaycast;

    [Export] public bool isFrontWheel = false;
    [Export] public float maxSteerAngle = 30f;
    [Export] public float lateralGrip = 300f;
    [Export] public float rollingResistance = 0.8f;
    [Export] public float brakeForce = 80f;
    [Export] public float steerSpeed = 120f;
    [Export] public float returnSpeed = 90f;
    [Export] public float speedSensitiveSteeringMin = 0.4f;
    [Export] public float speedSensitiveSteeringThreshold = 40f;
    
    // SMOOTH SUSPENSION PARAMETERS
    [Export] public float suspensionForce = 1500f;
    [Export] public float suspensionDamping = 600f;
    [Export] public float restLength = 0.4f;
    [Export] public float maxCompressionForce = 3500f;
    [Export] public float groundedThreshold = 0.1f;
    
    // BUMP SMOOTHING - Key to ignoring bumps!
    [Export] public float bumpSmoothingSpeed = 8f; // How fast to smooth out bumps (lower = smoother)
    [Export] public float maxBumpHeight = 0.15f; // Maximum bump height to smooth over
    [Export] public float chassisStabilizationForce = 2000f; // Force to keep chassis level
    
    // ANTI-ROLL PARAMETERS
    [Export] public float antiRollForce = 2000f;
    [Export] public float antiRollDamping = 400f;
    
    [Export] public float minimumRollingSpeed = 0.5f;
    [Export] public float airResistanceCoefficient = 0.005f;

    private float steerAngle = 0;
    private Vector3 lastPosition;
    private float lastCompression = 0f;
    private bool wasGrounded = false;
    private Wheel oppositeWheel = null;
    
    // Smooth compression tracking
    private float targetCompression = 0f;
    private float smoothedCompression = 0f;
    private float previousCompression = 0f;

    public override void _Ready()
    {
        differential = GetParent().GetNode<Differential>("Differential");
        chassis = GetParent().GetNode<RigidBody3D>("Chassis");
        groundingRaycast = GetNode<RayCast3D>("RayCast3D");
        isFrontWheel = Name.Equals("FL") || Name.Equals("FR");
        lastPosition = GlobalPosition;
        
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
        previousCompression = smoothedCompression;
        
        // Apply smooth suspension with bump filtering
        bool isGrounded = ApplySmoothSuspension(delta);

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

        // ENGINE TORQUE
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
        
        float optimalSlipAngle = 6f;
        float currentSlipAngle = Mathf.RadToDeg(Mathf.Atan2(lateralSpeed, Mathf.Max(forwardSpeed, 0.1f)));
        
        float gripMultiplier = 1.0f;
        if (currentSlipAngle > optimalSlipAngle)
        {
            float excessSlip = currentSlipAngle - optimalSlipAngle;
            gripMultiplier = Mathf.Max(0.4f, 1.0f - (excessSlip / 40f));
        }

        Vector3 gripForce = -lateralVel * lateralGrip * gripMultiplier;
        ApplyCentralForce(gripForce);
        
        // Apply grip force to chassis at its center of mass to avoid pitching
        chassis.ApplyCentralForce(gripForce * 0.3f);
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

            ApplyCentralForce(localForward * wheelTorque);
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
            ApplyCentralForce(totalBrakeForce);
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
        
        ApplyCentralForce(resistanceForce);
    }

    private bool ApplySmoothSuspension(double delta)
    {
        Vector3 suspensionDirection = -GlobalTransform.Basis.Y;
        
        if (!groundingRaycast.IsColliding())
        {
            targetCompression = 0f;
            smoothedCompression = Mathf.Lerp(smoothedCompression, 0f, (float)delta * bumpSmoothingSpeed);
            lastCompression = smoothedCompression;
            return false;
        }

        Vector3 hitPoint = groundingRaycast.GetCollisionPoint();
        float currentLength = (GlobalPosition - hitPoint).Length();
        float rawCompression = restLength - currentLength;
        
        // TARGET COMPRESSION - what the suspension "wants" to be at
        targetCompression = rawCompression;
        
        // SMOOTH THE COMPRESSION - this is the key to ignoring bumps!
        // Small bumps get filtered out by the smoothing
        float compressionDelta = targetCompression - smoothedCompression;
        float smoothingRate = bumpSmoothingSpeed;
        
        // Speed up smoothing for large bumps (to prevent bottoming out)
        if (Mathf.Abs(compressionDelta) > maxBumpHeight)
        {
            smoothingRate *= 2f;
        }
        
        smoothedCompression = Mathf.Lerp(smoothedCompression, targetCompression, (float)delta * smoothingRate);
        
        // Use smoothed compression for force calculations
        float compression = smoothedCompression;
        
        // Calculate smoothed compression velocity
        float compressionVelocity = (compression - previousCompression) / (float)delta;
        
        bool isGrounded = true;
        
        // Apply forces based on SMOOTHED compression
        if (compression > 0)
        {
            // Spring force based on smoothed compression
            float springForce = compression * suspensionForce;
            
            // Damping based on smoothed velocity
            float dampingForce = -compressionVelocity * suspensionDamping;
            
            // ANTI-ROLL BAR
            float antiRollTorque = 0f;
            if (oppositeWheel != null)
            {
                float compressionDifference = compression - oppositeWheel.smoothedCompression;
                antiRollTorque = compressionDifference * antiRollForce;
                
                float oppositeCompressionVel = (oppositeWheel.smoothedCompression - oppositeWheel.previousCompression) / (float)delta;
                float rollVelocity = compressionVelocity - oppositeCompressionVel;
                antiRollTorque += rollVelocity * antiRollDamping;
            }
            
            float totalForce = springForce + dampingForce + antiRollTorque;
            totalForce = Mathf.Clamp(totalForce, 0f, maxCompressionForce);
            
            Vector3 suspensionForceVector = suspensionDirection * totalForce;
            ApplyCentralForce(suspensionForceVector);
            
            // Apply equal and opposite force to chassis
            Vector3 chassisOffset = GlobalPosition - chassis.GlobalPosition;
            chassis.ApplyForce(-suspensionForceVector, chassisOffset);
            
            // Dampen excessive chassis vertical velocity
            float chassisVerticalVel = chassis.LinearVelocity.Dot(Vector3.Up);
            if (Mathf.Abs(chassisVerticalVel) > 0.5f)
            {
                Vector3 dampingForceVec = -Vector3.Up * chassisVerticalVel * 800f * (float)delta;
                chassis.ApplyCentralForce(dampingForceVec);
            }
        }
        
        lastCompression = compression;
        return isGrounded;
    }
    
    public float GetSmoothedCompression()
    {
        return smoothedCompression;
    }
    
    public float GetPreviousCompression()
    {
        return previousCompression;
    }
}
using Godot;
using System;

public partial class PlayerCamera : Camera3D
{
    [Export] public NodePath TargetPath;
    [Export] public float FollowSpeed = 5.0f;
    [Export] public float ZoomFactor = 5.0f;
    [Export] public float MaxSpeedForZoom = 30f;

    private RigidBody3D target;
    private int currentIndex = 0;

    // Define offsets in the inspector (relative to car local space)
    [Export] public Vector3[] CameraOffsets;

    public override void _Ready()
    {
        target = GetNode<RigidBody3D>(TargetPath);

        if (CameraOffsets == null || CameraOffsets.Length == 0)
        {
            // Default camera setups
            CameraOffsets = new Vector3[]
            {
                new Vector3(0, 5, -10), // behind
                new Vector3(0, 5, 10),  // front
                new Vector3(0, 15, 0),  // above
            };
        }
    }

    public override void _Process(double delta)
    {
        if (target == null) return;

        Basis basis = target.GlobalTransform.Basis;
        Vector3 forward = -basis.Z.Normalized();
        Vector3 up = basis.Y.Normalized();
        Vector3 right = basis.X.Normalized();

        // Get speed for zoom
        float speed = target.LinearVelocity.Length();
        float zoomAmount = Mathf.Clamp(speed / MaxSpeedForZoom, 0, 1) * ZoomFactor;

        // Current offset in local car space
        Vector3 offset = CameraOffsets[currentIndex];
        Vector3 dynamicOffset = offset + new Vector3(0, 0, -zoomAmount);

        // Transform offset into world space relative to car
        Vector3 desiredPos = target.GlobalTransform.Origin
                           + right * dynamicOffset.X
                           + up * dynamicOffset.Y
                           + forward * dynamicOffset.Z;

        // Smooth follow
        GlobalPosition = GlobalPosition.Lerp(desiredPos, (float)(delta * FollowSpeed));

        // Always look forward with the car
        LookAt(GlobalPosition + forward, Vector3.Up);

        // Handle switching
        if (PlayerInput.Instance.switchCameraForward)
        {
            currentIndex = (currentIndex + 1) % CameraOffsets.Length;
        }
    }
}

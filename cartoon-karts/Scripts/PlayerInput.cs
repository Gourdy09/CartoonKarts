using Godot;
using System;

public partial class PlayerInput : Node
{
	public static PlayerInput Instance { get; private set; }
	public float throttle { get; private set; }
	public float reverse { get; private set; }
	public float brake { get; private set; }
	public float steer { get; private set; }
	public bool switchCameraForward { get; private set; }
	public bool restart { get; private set; }

	public override void _Ready()
	{
		Instance = this;
	}

	// Poll for Inputs
	public override void _PhysicsProcess(double delta)
	{
		throttle = Input.GetActionStrength("throttle");
		reverse = Input.GetActionStrength("reverse");
		brake = Input.GetActionStrength("brake");
		steer = Input.GetAxis("steer_left", "steer_right");
		switchCameraForward = Input.IsActionJustPressed("switch_camera_forward");
		restart = Input.IsActionJustPressed("restart");

		if (restart)
		{
			GetTree().ChangeSceneToFile("Scenes/Track1.tscn");
		}
	}
}

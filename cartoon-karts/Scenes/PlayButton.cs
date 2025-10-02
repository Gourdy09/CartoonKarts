using Godot;
using System;

public partial class PlayButton : Button
{
	[Export] public string ScenePath = "res://scenes/game.tscn";
	
	public override void _Ready()
	{
		Pressed += OnPlayButtonPressed;
	}

	private void OnPlayButtonPressed()
	{
		if (string.IsNullOrEmpty(ScenePath))
		{
			GD.PrintErr("Scene path is not set!");
			return;
		}
		
		// Change to the specified scene
		GetTree().ChangeSceneToFile(ScenePath);
	}
}
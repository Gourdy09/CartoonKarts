using Godot;

public partial class GearDisplay : Label
{
    private Gearbox gearbox;
	[Export] public NodePath gearboxNode;

    public override void _Ready()
	{
		gearbox = GetNode<Gearbox>(gearboxNode);
	}

    public override void _Process(double delta)
    {
        if (gearbox != null)
        {
            int gear = gearbox.currentGearDisplay;
            Text = gear switch
            {
                -1 => "R",
                0 => "N",
                _ => gear.ToString()
            };
        }
    }
}
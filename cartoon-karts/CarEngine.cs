using Godot;
using System;

public partial class CarEngine : Node
{
	private float engineRPM = 0;
	[Export] public float engineTorque { private set; get; } = 0;
	private float maxEngineRPM = 7000;
	private float engineRPMPickup = 1;

	public override void _PhysicsProcess(double delta)
	{
		calculateRPM(PlayerInput.Instance.throttle, delta);
		calculateEngineTorque(engineRPM, 300, 50, 1, 4000, 2000);
	}

	private void calculateEngineTorque(float RPM, float peakTorque, float baseTorque, float curveSteepness, float peakTorqueRPM, float torqueBandWidth)
	{
		/* peakTorque: max torque at peak
		*  baseTorque: minimum torque (idle)
		* curveSteepness: how quick torque drops off
		* peakTorqueRPM: RPM at which peak torque occurs
		* torqueBandWidth: Spread of torque curve
		*/

		if (RPM <= peakTorqueRPM)
		{
			// Gaussian-style rise toward peak torque
			double exponent = -Math.Pow(curveSteepness * (RPM - peakTorqueRPM), 2) / Math.Pow(torqueBandWidth, 2);
			float torque = (peakTorque - baseTorque) * (float)Math.Exp(exponent) + baseTorque;
			engineTorque = MathF.Min(torque, peakTorque);
		}
		else
		{
			engineTorque = peakTorque;
		}
	}

	private void calculateRPM(float throttle, double delta)
	{
		if (throttle > 0)
		{
			// Accelerate toward max RPM depending on throttle
			engineRPM = (float)Mathf.Lerp(engineRPM, maxEngineRPM, throttle * delta * engineRPMPickup);
		}
		else
		{
			// Fall back to idle when throttle is released
			float idleRPM = 800f;
			engineRPM = (float)Mathf.Lerp(engineRPM, idleRPM, delta * engineRPMPickup);
			engineRPM = MathF.Max(engineRPM, idleRPM); // prevent going below idle
		}
	}
}

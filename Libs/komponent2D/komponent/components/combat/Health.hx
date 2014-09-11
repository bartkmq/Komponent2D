package komponent.components.combat;

import komponent.Component;

/**
 * Dispatches:
 * -> onHealthChange(this) when the health value is changed.
 * -> onInvulnerableChange(this) when this GameObject becomes invulnerable or vulnerable.
 * -> onDeath(this) when health is under 0.
 */
class Health extends Component
{
	
	public var health(default, set):Float = 100;
	public var maxHealth:Float = 100;
	public var invulnerable(default, set):Bool = false;
	public var dead(default, set):Bool = false;
	
	private function checkStatus()
	{
		if (health <= 0)
			dead = true;
		else if (dead && health > 0)
			dead = false;
	}
	
	public function set_health(value:Float):Float 
	{
		if (!(invulnerable && value < health))
		{
			health = value;
			if (health > maxHealth)
				health = maxHealth;
			sendMessage("onHealthChange", this);
			checkStatus();
		}
		return health;
	}
	
	public function set_invulnerable(value:Bool):Bool
	{
		if (value != invulnerable)
		{
			invulnerable = value;
			sendMessage("onInvulnerableChange", this);
		}
		return invulnerable;
	}
	
	public function set_dead(value:Bool):Bool
	{
		if (value && value != dead)
		{
			dead = value;
			sendMessage("onDeath", this);
		}
		return dead = value;
	}
	
}
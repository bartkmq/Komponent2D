package components;

import kha.Key;
import nape.phys.FluidProperties;

import nape.phys.Material;
import nape.shape.Polygon;
import nape.geom.Vec2;

import komponent.components.Physics;
import komponent.components.graphic.Animation;
import komponent.components.physics.Hitbox;
import komponent.components.combat.Health;
import komponent.input.Keyboard;
import komponent.utils.Screen;
import komponent.Component;
import komponent.utils.Time;

class Player extends Component
{
	public var movementForce:Float = 500;
	
	private var physics:Physics;
	private var animation:Animation;
	private var hitbox:Hitbox;

	override public function added() 
	{		
		Keyboard.define("up", ["w"], [Key.UP]);
		Keyboard.define("down", ["s"], [Key.DOWN]);
		Keyboard.define("left", ["a"], [Key.LEFT]);
		Keyboard.define("right", ["d"], [Key.RIGHT]);
		
		physics = addComponent(Physics);
		
		hitbox = addComponent(Hitbox);
		hitbox.setSize(16, 26);
		hitbox.shape.material.dynamicFriction = 0.00001;
		
		animation = addComponent(Animation);
		animation.loadSpritemap("player_default", 16, 26);
		animation.add("up", [1, 2, 0], 6, true);
		animation.add("down", [4, 5, 3], 6, true);
		animation.add("left", [7, 8, 6], 6, true);
		animation.add("right", [10, 11, 9], 6, true);
		animation.play("down");
		
		addComponent(Health);
	}
	
	override public function update()
	{
		handleInput();
		handleDrag();
	}
	
	private function handleInput()
	{
		var impulse:Vec2 = Vec2.weak();
		if (Keyboard.check("up"))
		{
			impulse.y = -movementForce;
			animation.play("up");
		}
		else if (Keyboard.check("down"))
		{
			impulse.y = movementForce;
			animation.play("down");
		}
		else if (Keyboard.check("left"))
		{
			impulse.x = -movementForce;
			animation.play("left");
		}
		else if (Keyboard.check("right"))
		{
			impulse.x = movementForce;
			animation.play("right");
		}
		else
		{
			animation.stop();
		}
		
		impulse.muleq(Time.elapsed);
		physics.body.applyImpulse(impulse);
	}
	
	private function handleDrag()
	{
		var velocity = physics.body.velocity;
		physics.body.applyImpulse(velocity.mul(-2, true).muleq(Time.elapsed));
	}
	
}
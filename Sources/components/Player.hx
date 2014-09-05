package components;

import kha.Key;

import hxcollision.Collision;

import komponent.GameObject;
import komponent.components.physics.Hitbox;
import komponent.components.graphic.Animation;
import komponent.components.combat.Health;
import komponent.input.Keyboard;
import komponent.utils.Screen;
import komponent.Component;
import komponent.utils.Time;

class Player extends Component
{
	
	public var movementSpeed:Float = 5;
	
	private var animation:Animation;
	private var hitbox:Hitbox;
	private var ground:Hitbox;

	override public function added() 
	{		
		Keyboard.define("up", ["w"], [Key.UP]);
		Keyboard.define("down", ["s"], [Key.DOWN]);
		Keyboard.define("left", ["a"], [Key.LEFT]);
		Keyboard.define("right", ["d"], [Key.RIGHT]);
		
		animation = addComponent(Animation);
		animation.loadSpritemap("player_default", 16, 26);
		animation.add("up", [1, 2, 0], 6, true);
		animation.add("down", [4, 5, 3], 6, true);
		animation.add("left", [7, 8, 6], 6, true);
		animation.add("right", [10, 11, 9], 6, true);
		animation.play("down");
		
		hitbox = addComponent(Hitbox);
		hitbox.setSize(16, 26);
		
		addComponent(Health);
	}
	
	override public function update()
	{
		handleInput();
		
		if (ground == null)
		{
			ground = getGameObjectByName("Ground").getComponent(Hitbox);
		}
		var collision = Collision.test(hitbox.shape, ground.shape);
		if (collision != null)
		{
			transform.localX += collision.separation.x;
			transform.localY += collision.separation.y;
		}
	}
	
	private function handleInput()
	{
		if (Keyboard.check("up"))
		{
			animation.play("up");
			transform.localY -= movementSpeed;
		}
		else if (Keyboard.check("down"))
		{
			animation.play("down");
			transform.localY += movementSpeed;
		}
		else if (Keyboard.check("left"))
		{
			animation.play("left");
			transform.localX -= movementSpeed;
		}
		else if (Keyboard.check("right"))
		{
			animation.play("right");
			transform.localX += movementSpeed;
		}
		else
		{
			animation.stop();
		}
	}
	
	private function onCollision()
	{
		trace("Ground Collision");
	}
	
}
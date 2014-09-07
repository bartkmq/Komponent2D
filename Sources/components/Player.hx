package components;

import kha.Key;
import komponent.components.Physics;

import komponent.physics.CollisionData2D;

import komponent.GameObject;
import komponent.components.physics.Hitbox;
import komponent.components.graphic.Animation;
import komponent.components.combat.Health;
import komponent.input.Keyboard;
import komponent.input.Input;
import komponent.input.Input.AxisInput;
import komponent.utils.Screen;
import komponent.utils.Misc;
import komponent.Component;
import komponent.utils.Time;

class Player extends Component
{
	
	public var movementSpeed:Float = 200;
	
	private var animation:Animation;
	private var physics:Physics;

	override public function added() 
	{		
		Keyboard.define("up", ["w"], [Key.UP]);
		Keyboard.define("down", ["s"], [Key.DOWN]);
		Keyboard.define("left", ["a"], [Key.LEFT]);
		Keyboard.define("right", ["d"], [Key.RIGHT]);
		
		Input.defineAxis("horizontal", [KEYBOARD("left", -1), KEYBOARD("right", 1)]);
		Input.defineAxis("vertical", [KEYBOARD("up", -1), KEYBOARD("down", 1)]);
		
		animation = addComponent(Animation);
		animation.loadSpritemap("player_default", 16, 26);
		animation.add("up", [1, 2, 0], 6, true);
		animation.add("down", [4, 5, 3], 6, true);
		animation.add("left", [7, 8, 6], 6, true);
		animation.add("right", [10, 11, 9], 6, true);
		animation.play("down");
		
		addComponent(Hitbox).setSize(16, 26);
		
		physics = addComponent(Physics);
		physics.moveByTypes.push("ground");
		physics.elasticityY = 0.8;
		
		addComponent(Health);
	}
	
	override public function update()
	{
		handleInput();
	}
	
	private function handleInput()
	{
		var deltaX = Input.getAxis("horizontal") * movementSpeed * Time.elapsed;
		var deltaY = Input.getAxis("vertical") * movementSpeed * Time.elapsed;
		
		physics.moveBy(deltaX, deltaY);
	}
	
	private function onCollision(collision:CollisionData2D)
	{
		var separation = collision.separation;
		trace(collision.collider1.gameObject.name + ", " + collision.collider2.gameObject.name +  ": " + collision.event);
	}
	
}
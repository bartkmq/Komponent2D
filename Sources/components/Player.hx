package components;

import kha.Key;

import komponent.physics.CollisionData2D;

import komponent.Component;
import komponent.GameObject;
import komponent.components.Physics;
import komponent.components.physics.Hitbox;
import komponent.components.graphic.Animation;
import komponent.components.graphic.Image;
import komponent.components.combat.Health;
import komponent.input.Keyboard;
import komponent.input.Input;
import komponent.input.Input;
import komponent.utils.Screen;
import komponent.utils.Misc;
import komponent.utils.Time;

class Player extends Component
{
	
	public var movementSpeed:Float = 2000;
	
	private var animation:Animation;
	private var physics:Physics;

	override public function added() 
	{		
		Keyboard.define("up", [Key.UP]);
		Keyboard.define("down", [Key.DOWN]);
		Keyboard.define("left", [Key.LEFT]);
		Keyboard.define("right", [Key.RIGHT]);
		Keyboard.define("rotate_left", ["y"]);
		Keyboard.define("rotate_right", ["x"]);
		
		Input.defineAxis("horizontal", [KEYBOARD("left", -1), KEYBOARD("right", 1)]);
		Input.defineAxis("vertical", [KEYBOARD("up", -1), KEYBOARD("down", 1)]);
		Input.defineAxis("rotation", [KEYBOARD("rotate_left", -1), KEYBOARD("rotate_right", 1)]);
		
		/*
		animation = addComponent(Animation);
		animation.loadSpritemap("player_default", 16, 26);
		animation.add("up", [1, 2, 0], 6, true);
		animation.add("down", [4, 5, 3], 6, true);
		animation.add("left", [7, 8, 6], 6, true);
		animation.add("right", [10, 11, 9], 6, true);
		animation.play("down");
		*/
		var image = addComponent(Image);
		image.load("wabbit");
		
		addComponent(Hitbox).setSize(16, 26);
		
		physics = addComponent(Physics);
		physics.moveByTypes.push("ground");
		physics.maxVelocity = VelocityLimit.REALISTIC(200);
		physics.dragX = 400;
		physics.dragY = 400;
		
		addComponent(Health);
	}
	
	override public function update()
	{
		handleInput();
	}
	
	private function handleInput()
	{
		physics.accelerationX = Input.getAxis("horizontal") * movementSpeed;
		physics.accelerationY = Input.getAxis("vertical") * movementSpeed;
		transform.localRotation += Input.getAxis("rotation");
	}
	
	private function onCollision(collision:CollisionData2D)
	{
		
	}
	
}
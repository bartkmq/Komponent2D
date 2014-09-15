package scene;

import kha.Button;
import kha.Color;
import kha.Key;
import kha.math.Vector2;

import komponent.GameObject;
import komponent.Scene;
import komponent.input.Keyboard;
import komponent.input.Mouse;
import komponent.utils.Screen;
import komponent.components.misc.Camera;
import komponent.components.misc.Debug;
import komponent.components.graphic.Animation;
import komponent.components.physics.Hitbox;
import komponent.components.Physics;
import komponent.components.graphic.Text;
import komponent.utils.Time;
import komponent.input.Input;

import components.Player;

class GameScene extends Scene
{
	
	public var player:GameObject;
	public var ground:GameObject;
	
	public var config:Dynamic;

	override public function begin()
	{				
		new GameObject("Camera", 0, 0).addComponent(Camera);
		
		Keyboard.define("restart", ["r"], [Key.CTRL], true);
		//Keyboard.define("debug", ["d"], [Key.CTRL], true);
		
		Keyboard.define("camera_up", ["w"]);
		Keyboard.define("camera_down", ["s"]);
		Keyboard.define("camera_left", ["a"]);
		Keyboard.define("camera_right", ["d"]);
		Keyboard.define("camera_rotation_left", ["q"]);
		Keyboard.define("camera_rotation_right", ["e"]);
		
		Input.defineAxis("camera_horizontal", [KEYBOARD("camera_left", -1), KEYBOARD("camera_right", 1)]);
		Input.defineAxis("camera_vertical", [KEYBOARD("camera_up", -1), KEYBOARD("camera_down", 1)]);
		Input.defineAxis("camera_rotation", [KEYBOARD("camera_rotation_left", -1), KEYBOARD("camera_rotation_right", 1)]);
		
		Screen.color = Color.fromString("#57A2DF");
		
		player = new GameObject("Player", 0, 0, "player");
		player.addComponent(Player);
		
		ground = new GameObject("Ground", 0, Screen.bottom - 10, "ground");
		var hitbox = ground.addComponent(Hitbox);
		hitbox.setSize(Screen.width, 10);
		
		#if debug
		for (go in gameObjects)
			go.addComponent(Debug);
		#end
		Time.scale = 1;
	}
	
	override public function update()
	{
		super.update();
		if (Keyboard.check("restart"))
			engine.currentScene = new GameScene();
		if (Keyboard.check("debug"))
			engine.debug = !engine.debug;
			
		Screen.camera.x += Input.getAxis("camera_horizontal") * 10;
		Screen.camera.y += Input.getAxis("camera_vertical") * 10;
		Screen.camera.gameObject.transform.localRotation += Input.getAxis("camera_rotation") * 5;
			
		if (Mouse.wheel)
			Screen.camera.zoom += Mouse.wheelDelta * 0.1;
	}
	
}
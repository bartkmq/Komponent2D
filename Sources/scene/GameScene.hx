package scene;

import kha.Button;
import kha.Key;

import komponent.GameObject;
import komponent.Scene;
import komponent.input.Keyboard;
import komponent.input.Mouse;
import komponent.utils.Screen;
import komponent.components.misc.Camera;
import komponent.components.graphic.Image;
import komponent.components.misc.Debug;
import komponent.components.physics.Hitbox;

import components.FollowMouse;

class GameScene extends Scene
{
	
	public var config:Dynamic;

	override public function begin()
	{
		//var configData = Loader.the.getBlob("example").toString();
		//config = Yaml.parse(configData, Parser.options().useObjects());
		
		engine.debug = true;
		
		new GameObject("Camera", 0, 0).addComponent(Camera);
		
		Keyboard.define("restart", ["r"]);
		Keyboard.define("debug", ["d"]);
		
		Keyboard.define("up", [Key.UP]);
		Keyboard.define("down", [Key.DOWN]);
		Keyboard.define("left", [Key.LEFT]);
		Keyboard.define("right", [Key.RIGHT]);
		
		var wabbit = new GameObject("Wabbit", 50, 50);
		wabbit.addComponent(Image).load("wabbit");
		wabbit.addComponent(Debug);
		//wabbit.addComponent(FollowMouse);
		
		var ground = new GameObject("Ground", Screen.left, Screen.bottom - 10);
		ground.addComponent(Hitbox).setSize(Screen.width, 10);
		ground.addComponent(Debug);
	}
	
	override public function update()
	{
		super.update();
		
		if (Keyboard.check("restart"))
			engine.currentScene = new GameScene();
		if (Keyboard.check("debug"))
			engine.debug = !engine.debug;
			
		if (Keyboard.check("up"))
			Screen.camera.y -= 5;
		else if (Keyboard.check("down"))
			Screen.camera.y += 5;
		else if (Keyboard.check("left"))
			Screen.camera.x -= 5;
		else if (Keyboard.check("right"))
			Screen.camera.x += 5;
			
		if (Mouse.wheel)
			Screen.camera.zoom += Mouse.wheelDelta * 0.01;
	}
	
}
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
		Keyboard.define("debug", ["d"], [Key.CTRL], true);
		
		Screen.color = Color.fromString("#57A2DF");
		
		player = new GameObject("Player", 200, 200, "player");
		player.addComponent(Player);
		
		ground = new GameObject("Ground", Screen.halfWidth, Screen.bottom - 10, "ground");
		var hitbox = ground.addComponent(Hitbox);
		hitbox.setSize(Screen.width, 10);
	}
	
	override public function update()
	{
		super.update();
		if (Keyboard.check("restart"))
			engine.currentScene = new GameScene();
		if (Keyboard.check("debug"))
			engine.debug = !engine.debug;
			
		if (Mouse.wheel)
			Screen.camera.zoom += Mouse.wheelDelta * 0.01;
	}
	
}
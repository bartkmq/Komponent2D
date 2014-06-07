package scene;

import kha.Loader;

import komponent.components.graphic.Animation;

import komponent.GameObject;
import komponent.Scene;
import komponent.utils.Input;
import komponent.utils.Screen;
import komponent.extension.Nape;

import yaml.Parser;
import yaml.Yaml;

class GameScene extends Scene
{
	
	public var config:Dynamic;

	override public function begin()
	{
		var configData = Loader.the.getBlob("example").toString();
		config = Yaml.parse(configData, Parser.options().useObjects());
		
		//engine.debug = false;
		getExtension(Nape).setGravity(0, 30);
		
		Input.loadConfig(config.Input);
		Screen.loadConfig(config.Screen);
		
		GameObject.loadPrefab(config.Wabbit);
		GameObject.loadPrefab(config.Coin);
		GameObject.loadPrefab(config.Ground);
		GameObject.loadPrefab(config.HUD);
	}
	
	override public function update()
	{
		if (Input.check("restart"))
			engine.currentScene = new GameScene();
		if (Input.check("debug"))
			engine.debug = !engine.debug;
	}
	
}
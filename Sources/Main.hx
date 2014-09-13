package;

import kha.Starter;

import komponent.Engine;

import scene.GameScene;


class Main
{
	
	public static function main()
	{
		var starter = new Starter();
		starter.start(new Engine("Test", ["default"], GameScene));
	}
	
}
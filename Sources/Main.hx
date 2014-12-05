package;

import kha.Starter;

import komponent.Engine;

import scene.MatrixTest;

class Main
{
	
	public static function main()
	{
		var starter = new Starter();
		starter.start(new Engine("Test", ["default"], MatrixTest));
	}
	
}
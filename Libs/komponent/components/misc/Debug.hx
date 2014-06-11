package komponent.components.misc;

import kha.Font;
import kha.Loader;
import kha.FontStyle;
import kha.Color;

import komponent.utils.Input;
import komponent.utils.Screen;
import komponent.components.graphic.Image;

import komponent.utils.Painter;

class Debug extends Component
{
	/***
	 * Shows the name of the GameObject and wich components it contains
	 */
	
	public var font:Font;
	
	override public function added()
	{
		font = Loader.the.loadFont("roboto", FontStyle.Default, 18);
	}
	
	override public function debugDraw()
	{
		var distance = Math.sqrt(Math.pow((transform.x - Input.mouseX), 2) + Math.pow(transform.y - Input.mouseY, 2));
		if (distance < 20)
		{
			var image = getComponent(Image);
		
			var width = 0;
			if (image != null)
				width = image.width;
		
			var x = transform.x + width + 10;
			var y = transform.y;
			
			for (camera in Screen.cameras)
			{
				Painter.camera = camera;
				Painter.set(Color.Black, 1, font);
				Painter.drawString(gameObject.toString(), x - camera.x, y - camera.y);
			}
		}
	}
	
}
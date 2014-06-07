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
		font = Loader.the.loadFont("tahoma", FontStyle.Default, 18);
	}
	
	override public function debugDraw()
	{
		var _transform = transform;
		var distance = Math.sqrt(Math.pow((_transform.x - Input.mouseX), 2) + Math.pow(_transform.y - Input.mouseY, 2));
		if (distance < 20)
		{
			var _image = getComponent(Image);
		
			var width = 0;
			if (_image != null)
				width = _image.width;
		
			var x = _transform.x + width + 10;
			var y = _transform.y;
		
			if (Screen.camera != null)
			{
				x -= Screen.camera.x;
				y -= Screen.camera.y;
			}
		
			Painter.set(Color.Black, 1, font);
			Painter.drawString(gameObject.toString(), x, y);
		}
	}
	
}
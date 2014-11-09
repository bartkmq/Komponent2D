package komponent.components.misc;

import kha.Font;
import kha.Loader;
import kha.FontStyle;
import kha.Color;

import komponent.input.Mouse;
import komponent.utils.Screen;
import komponent.utils.Painter;
import komponent.utils.Painter2;
import komponent.components.graphic.Image;

class Debug extends Component
{
	/***
	 * Shows the name of the GameObject and wich components it contains.
	 */
	
	public var font:Font;
	
	override public function added()
	{
		font = Loader.the.loadFont("roboto", FontStyle.Default, 18);
	}
	
	override public function debugDraw()
	{
		var distance = Math.sqrt(Math.pow((transform.x - Mouse.scene.x), 2) + Math.pow(transform.y - Mouse.scene.y, 2));
		if (distance < 20 * ((Screen.camera.fullScaleX + Screen.camera.fullScaleY) / 2))
		{
			var image = getComponent(Image);
		
			var width = 0;
			if (image != null)
				width = image.width;
			
			Painter.set(Color.White, 1, font);
			
			for (camera in Screen.cameras)
			{
				if (!Painter.fallbackPainter)
				{
					Painter.matrix = camera.matrix * transform.matrix;
					Painter.drawString(gameObject.toString(), width + 10, 0);
					Painter.matrix = null;
				}
				else
				{
					Painter2.camera = camera;
					Painter2.drawString(gameObject.toString(), transform.x - camera.x, transform.y - camera.y);
				}
			}
		}
	}
	
}
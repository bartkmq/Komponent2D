package komponent.components.graphic;

import kha.Color;
import kha.Loader;
import kha.Rectangle;
import kha.Image in KhaImage;

import komponent.utils.Misc;
import komponent.components.Graphic;
import komponent.utils.Painter;
import komponent.utils.Screen;
import komponent.ds.Matrix;

using komponent.utils.Parser;

class Image extends Graphic
{
	
	/**
	 * If one of these values is higher than the images width/height
	 * the image will be tiled (repeated) to match the specified size.
	 */
	public var tiledWidth:Int;
	public var tiledHeight:Int;
	
	/**
	 * If the image should be repeated to fill the whole screen.
	 * Overwrites tiledWidth/tiledHeight.
	 */
	public var fillScreen:Bool;
	
	/**
	 * Defines wich area of the source image should be displayed.
	 * If null (default) the whole image will be displayed.
	 */
	public var sourceRect:Rectangle;
	
	private var image:KhaImage;

	override public function added() 
	{
		super.added();
		
		tiledWidth = tiledHeight = 0;
		fillScreen = false;
	}
	
	override public function render()
	{	
		if (visible && image != null)
		{
			Painter.set(color, alpha);
			for (camera in Screen.cameras)
			{				
				Painter.matrix = camera.matrix * matrix;
				if (sourceRect == null)
					Painter.drawImage(image, 0, 0)
				else
					Painter.drawSubImage(image, 0, 0, sourceRect.x, sourceRect.y, sourceRect.width, sourceRect.height);
				Painter.matrix = null;
			}
		}
	}
	
	public inline function load(assetName:String)
	{
		this.image = Loader.the.getImage(assetName);
	}
	
	public inline function setSourceRectangle(x:Float, y:Float, width:Float, height:Float)
	{
		if (sourceRect == null)
		{
			sourceRect = new Rectangle(x, y, width, height);
		}
		else
		{
			sourceRect.x = x;
			sourceRect.y = y;
			sourceRect.width = width;
			sourceRect.height = height;
		}
	}
	
	public inline function setTiledSize(tiledWidth:Int, tiledHeight:Int):Void
	{
		this.tiledWidth = tiledWidth;
		this.tiledHeight = tiledHeight;
	}
	
	override public function loadConfig(data:Dynamic):Void
	{
		super.loadConfig(data);
		
		var file:String = data.file;
		if (file != null) load(file);
		
		flipX = data.flipX.parse(false);
		flipY = data.flipY.parse(false);
		tiledWidth = data.tiledWidth.parse(0);
		tiledHeight = data.tiledHeight.parse(0);
		fillScreen = data.fillScreen.parse(false);
		sourceRect = data.sourceRectangle.parseRect(null);
	}
	
	private override function get_width():Float { return (sourceRect == null) ? image.width : sourceRect.width; }
	private override function get_height():Float { return (sourceRect == null) ? image.height : sourceRect.height; }
}
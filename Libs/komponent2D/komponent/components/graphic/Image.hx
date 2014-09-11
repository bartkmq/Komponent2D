package komponent.components.graphic;

import kha.Color;
import kha.Loader;
import kha.Rectangle;
import kha.Image in KhaImage;

import komponent.components.Graphic;
import komponent.utils.Painter;
import komponent.utils.Screen;

using komponent.utils.Parser;

class Image extends Graphic
{
	
	public var width(get, never):Int;
	public var height(get, never):Int;
	
	public var alpha:Float;
	public var color:Color;
	
	/**
	 * If the image should be flipped horizontally or vertically.
	 */
	public var flipX:Bool;
	public var flipY:Bool;
	
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
	
	private var _image:KhaImage;

	override public function added() 
	{
		super.added();
		alpha = 1;
		color = Color.White;
		flipX = flipY = false;
		
		tiledWidth = tiledHeight = 0;
		fillScreen = false;
	}
	
	override public function render()
	{	
		if (visible && _image != null)
		{
			Painter.set(color, alpha);
			for (camera in Screen.cameras)
			{
				
				Painter.camera = camera;
				Painter.setScale(transform.localScaleX, transform.localScaleY);
				Painter.drawImage5(_image, transform.x, transform.y, transform.rotation, 0, 0,
									flipX, flipY, sourceRect, tiledWidth, tiledHeight, fillScreen);
			}
		}
	}
	
	public inline function load(image:String)
	{
		_image = Loader.the.getImage(image);
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
		
		alpha = data.alpha.parse(1.0);
		color = data.color.parseColor(Color.White);
		flipX = data.flipX.parse(false);
		flipY = data.flipY.parse(false);
		tiledWidth = data.tiledWidth.parse(0);
		tiledHeight = data.tiledHeight.parse(0);
		fillScreen = data.fillScreen.parse(false);
		sourceRect = data.sourceRectangle.parseRect();
	}
	
	private inline function get_width():Int { return _image.width; }
	private inline function get_height():Int { return _image.height; }
}
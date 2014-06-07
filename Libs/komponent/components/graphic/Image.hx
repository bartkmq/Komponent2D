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
	
	public var flipX:Bool;
	public var flipY:Bool;
	
	public var tiledWidth:Int;
	public var tiledHeight:Int;
	public var fillScreen:Bool;
	
	public var sourceRect:Rectangle;
	
	private var _image:KhaImage;

	private function new() 
	{
		super();
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
			var _transform = transform;
			
			var useCamera:Bool = Screen.camera != null;
		
			Painter.set(color, alpha);
			Painter.drawImage5(_image, _transform.x, _transform.y, _transform.khaRotation,
								_transform.scaleX, _transform.scaleY, flipX, flipY, true,
								useCamera, sourceRect, tiledWidth, tiledHeight, fillScreen);
		}
	}
	
	public inline function load(image:String)
	{
		_image = Loader.the.getImage(image);
	}
	
	public inline function setSourceRectangle(x:Float, y:Float, width:Float, height:Float)
	{
		sourceRect = new Rectangle(x, y, width, height);
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
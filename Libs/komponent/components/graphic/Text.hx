package komponent.components.graphic;

import kha.Color;
import kha.Font;
import kha.FontStyle;
import kha.Loader;

import komponent.components.Graphic;
import komponent.utils.Painter;
import komponent.utils.Screen;

using komponent.utils.Parser;

class Text extends Graphic
{
	
	public var width(get, never):Float;
	public var height(get, never):Float;
	
	public var text:String;
	public var font:Font;
	public var size(get, set):Float;
	public var fontstyle(get, set):FontStyle;
	
	public var alpha:Float;
	public var color:Color;
	
	
	private function new() 
	{
		super();
		text = "";
		alpha = 1;
		color = Color.White;
	}
	
	override public function render()
	{
		if (visible && font != null)
		{
			var _transform = transform;
			Painter.set(color, alpha, font);
			
			if (Screen.camera != null)
				Painter.drawString(text, _transform.x - Screen.camera.x, _transform.y - Screen.camera.y);
			else
				Painter.drawString(text, _transform.x, _transform.y);
		}
	}
	
	public inline function set(text:String, font:Font)
	{
		this.text = text;
		this.font = font;
	}
	
	override public function loadConfig(data:Dynamic):Void
	{
		visible = data.visible.parse(true);
		text = data.text.parse("");
		alpha = data.alpha.parse(1.0);
		color = data.color.parse(Color.White);
		
		var file:String = data.font.file;
		var size:Float = data.font.size;
		var style:FontStyle = data.style.parseFontStyle(FontStyle.Default);
		if (file != null && data.font.size != null && size > 0.0)
			font = Loader.the.loadFont(file, style, size);
	}
	
	private inline function get_width():Float { return font.stringWidth(text); }
	
	private inline function get_height():Float { return font.getHeight(); }
	
	private inline function get_size():Float { return font.size; }
	private inline function set_size(value:Float):Float
	{
		font = Loader.the.loadFont(font.name, font.style, value);
		return value;
	}
	
	private inline function get_fontstyle():FontStyle { return font.style; }
	private inline function set_fontstyle(value:FontStyle):FontStyle
	{
		font = Loader.the.loadFont(font.name, value, font.size);
		return value;
	}
}
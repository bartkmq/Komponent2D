package komponent.utils;

import kha.Color;

import komponent.components.misc.Camera;

using komponent.utils.Parser;

class Screen
{
	public static var color:Null<Color>;
	
	public static var camera:Camera;
	
	public static var scaleX(default, set):Float = 1;
	public static var scaleY(default, set):Float = 1;
	public static var scale(default, set):Float = 1;
	
	public static var fullScaleX(default, null):Float = 1;
	public static var fullScaleY(default, null):Float = 1;
	
	public static var width(get, set):Int;
	public static var height(get, set):Int;
	public static var halfWidth(get, never):Float;
	public static var halfHeight(get, never):Float;
	
	public static var left(get, never):Float;
	public static var right(get, never):Float;
	public static var top(get, never):Float;
	public static var bottom(get, never):Float;
	
	public static function loadConfig(data:Dynamic):Void
	{
		color = data.color.parse(Color.White);
		scaleX = data.scaleX.parse(1.0);
		scaleY = data.scaleY.parse(1.0);
		scale = data.zoom.parse(1.0);
	}
	
	private static inline function set_scaleX(value:Float):Float
	{
		fullScaleX = value * scale;
		return scaleX = value;
	}
	
	private static inline function set_scaleY(value:Float):Float
	{
		fullScaleY = value * scale;
		return scaleY = value;
	}
	
	private static inline function set_scale(value:Float):Float
	{
		fullScaleX = scaleX * value;
		fullScaleY = scaleY * value;
		return scale = value;
	}
	
	private static inline function get_width():Int { return Misc.engine.width; }
	private static inline function set_width(value:Int):Int { return Misc.engine.width = value; }
	
	private static inline function get_height():Int { return Misc.engine.height; }
	private static inline function set_height(value:Int):Int { return Misc.engine.height = value; }
	
	private static inline function get_halfWidth():Float { return width / 2; }
	private static inline function get_halfHeight():Float { return height / 2; }
	
	private static inline function get_left():Int { return 0; }
	private static inline function get_right():Int { return width; }
	private static inline function get_top():Int { return 0; }
	private static inline function get_bottom():Int { return height; }
}
package komponent.utils;

import kha.Color;

import komponent.components.misc.Camera;

using komponent.utils.Parser;

class Screen
{
	public static var color:Null<Color>;
	
	// The first added camera.
	public static var camera:Camera;
	
	// All cameras.
	public static var cameras:Array<Camera> = [];
	
	public static var width(get, set):Int;
	public static var height(get, set):Int;
	public static var halfWidth(get, never):Float;
	public static var halfHeight(get, never):Float;
	
	public static var left(get, never):Float;
	public static var right(get, never):Float;
	public static var top(get, never):Float;
	public static var bottom(get, never):Float;
	
	public static inline function loadConfig(data:Dynamic)
	{
		color = data.parseColor(null);
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
package komponent.utils;

import kha.Button;
import kha.Color;
import kha.FontStyle;
import kha.Key;
import kha.Rectangle;

using komponent.utils.Parser;

class Parser
{
	
	public static inline function parse<T>(data:Dynamic, defaultValue:T):T
	{
		return (data != null) ? data : defaultValue;
	}
	
	public static inline function parseClass<T:Class<Dynamic>>(data:Dynamic, defaultClass:T):T
	{
		if (data != null)
			defaultClass = cast Type.resolveClass(data);
		return defaultClass;
	}
	
	public static inline function parseRect(data:Dynamic, x:Float = 0, y:Float = 0, width:Float = 0, height:Float = 0):Rectangle
	{
		var rect:Rectangle;
		if (data != null)
			rect = new Rectangle(data.x.parse(0.0), data.y.parse(0.0), data.width.parse(10.0), data.height.parse(10.0));
		else if (x == 0 && y == 0 && width == 0 && height == 0)
			rect = null;
		else
			rect = new Rectangle(x, y, width, height);
		return rect;
	}
	
	public static inline function parseFontStyle(data:Dynamic, defaultStyle:FontStyle):FontStyle
	{
		if (data != null)
			defaultStyle = new FontStyle(data.bold.parse(false), data.italic.parse(false), data.underlined.parse(false));
		return defaultStyle;
	}
	
	public static inline function parseColor(data:Dynamic, defaultColor:Null<Color>):Color
	{
		if (data != null)
		{
			if (Std.is(data, Int))
				defaultColor = Color.fromValue(data);
			else if (Std.is(data, Array) && data.length != 0 && Std.is(data[0], Int))
				defaultColor = Color.fromBytes(data[0], data[1], data[2]);
			else if (Std.is(data, Array) && data.length != 0 && Std.is(data[0], Float))
				defaultColor = Color.fromFloats(data[0], data[1], data[2]);
			else if (Std.is(data, String))
			{
				if (data.toLowerCase() == "white")
					defaultColor = Color.White;
				else if (data.toLowerCase() == "black")
					defaultColor = Color.Black;
				else
					defaultColor = Color.fromString(data);
			}
		}
		return defaultColor;
	}
	
	public static inline function parseButton(data:Dynamic, defaultButton:Button):Button
	{
		return (data != null) ? Type.createEnum(Button, data.toUpperCase()) : defaultButton;
	}
	
	public static inline function parseKey(data:Dynamic, defaultKey:Key):Key
	{
		return (data != null) ? Type.createEnum(Key, data.toUpperCase()) : defaultKey;
	}

}
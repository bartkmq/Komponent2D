package komponent.utils;

import kha.Color;
import kha.FontStyle;
import kha.Rectangle;

import nape.phys.Material;

class Misc
{
	
	public static var engine:Engine;
	
	public static var emptyArray:Array<Dynamic> = new Array<Dynamic>();
	
	// multiply this with a rad to convert it to deg
	public static var toDeg:Float = 180 / Math.PI;
	
	// multiply this with a deg to convert it to rad
	public static var toRad:Float = Math.PI / 180;
	
	// Returns 1 if bool is true and -1 if it's false
	public static inline function sign(bool:Bool):Float
	{
		return bool ? 1.0 : -1.0;
	}
	
	// Rounds a float to the specified amount of decimal places
	public static inline function round(v:Float, decimalPlaces:Int = 2):Float
	{
		var power = Math.pow(10, decimalPlaces);
		return Math.ffloor(v * power + 0.5) / power;
	}
	
	// calls a function on a class instance
	public static inline function callFunction(receiver:Dynamic, functionName:String, message:Dynamic):Void
	{
		if (message == null)
			message = emptyArray;
		
		var functionReference = Reflect.field(receiver, functionName);
		if (functionReference != null)
			Reflect.callMethod(receiver, functionReference, [message]);
	}
	
	// clears an array
	public static inline function clear(array:Array<Dynamic>)
	{
		#if (cpp || php)
		array.splice(0, array.length)
		#else
		untyped array.length = 0;
		#end
	}
	
	// checks if the array contains the element
	public static inline function contains<T>(array:Array<T>, element:T):Bool
	{
		return array.indexOf(element) != -1;
	}
	
	// clamps a value in a specific range
	public static inline function clamp(value:Float, min:Float, max:Float):Float
	{
		if (value > max) value = max;
		if (value < min) value = min;
		return value;
	}
}
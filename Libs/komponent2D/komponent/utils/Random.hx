package komponent.utils;

class Random
{
	
	public static inline function bool(trueChance:Float = 0.5):Bool
	{
		return Math.random() < trueChance;
	}
	
	public static inline function int(from:Int, to:Int):Int
	{
		return from + Math.floor(((to - from + 1) * Math.random()));
	}
	
	public static inline function float(from:Float, to:Float):Float
	{
		return from + ((to - from) * Math.random());
	}
	
	public static inline function fromArray<T>(array:Array<T>):Null<T>
	{
		if (array != null && array.length > 0)
			return array[int(0, array.length - 1)];
		else
			return null;
	}
	
	public static inline function shuffle<T>(array:Array<T>):Array<T>
	{
		if (array != null)
		{
			for (i in 0...array.length)
			{
				var j = int(0, array.length - 1);
				var a:T = array[i];
				var b:T = array[j];
				array[i] = b;
				array[j] = a;
			}
		}
		return array;
	}
}
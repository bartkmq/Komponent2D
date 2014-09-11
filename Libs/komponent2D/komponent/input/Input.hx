package komponent.input;

import kha.Key;

import komponent.input.Mouse;
import komponent.utils.Misc;

/**
 * Values are added to the Axis value.
 * Factors are multiplied with gamepad button/axis and then added to the axis value.
 */
enum AxisInput
{
	KEYBOARD(name:String, value:Float);
	MOUSE(mouseButton:MouseButton, value:Float);
	BUTTON(button:Int, gamepad:Gamepad, factor:Float);
	AXIS(axis:Int, gamepad:Gamepad, factor:Float);
}

@:allow(komponent.Engine)
class Input
{
	
	private static var definitions:Map<String, Axis> = new Map();
	
	/**
	 * Defines an Axis.
	 */
	public static function defineAxis(name:String, inputs:Array<AxisInput>)
	{
		definitions[name] = new Axis(inputs);
	}
	
	/**
	 * Return the axis value which can range from -1.0 to 1.0.
	 */
	public static function getAxis(name:String):Float
	{
		var axis = definitions[name];
		return (axis == null) ? 0 : axis.value;
	}
	
	private static function init()
	{
		Keyboard.init();
		Mouse.init();
		Touch.init();
	}
	
	private static function update()
	{
		Keyboard.update();
		Mouse.update();
		Gamepad.update();
		Touch.update();
		
		for (axis in definitions)
			axis.updated = false;
	}
	
}

private class Axis
{
	public var updated:Bool = false;
	public var inputs:Array<AxisInput>;
	@:isVar
	public var value(get, null):Float = 0;
	
	public function new(inputs:Array<AxisInput>)
	{
		this.inputs = inputs;
	}
	
	public function get_value():Float
	{
		if (!updated)
		{
			value = 0;
			for (input in inputs)
			{
				switch (input)
				{
					case KEYBOARD(name, inputValue):
						if (Keyboard.check(name)) value += inputValue;
					case MOUSE(mouseButton, inputValue):
						if (Mouse.check(mouseButton)) value += inputValue;
					case BUTTON(button, gamepad, factor):
						value += gamepad.getButton(button) * factor;
					case AXIS(axis, gamepad, factor):
						value += gamepad.getAxis(axis) * factor;
				}
			}
			value = Misc.clamp(value, -1, 1);
			updated = true;
		}
		return value;
	}
}

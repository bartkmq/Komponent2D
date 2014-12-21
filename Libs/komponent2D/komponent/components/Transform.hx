package komponent.components;

import kha.Color;

import komponent.GameObject;
import komponent.utils.Painter;
import komponent.utils.Screen;
import komponent.utils.Misc;
import komponent.ds.Matrix;
import komponent.components.misc.Camera;

using komponent.utils.Parser;


/**
 * Dispatches:
 * -> onTransformChange(this) when the (local/world) transformation is changed.
 */
class Transform extends Component
{
	/**
	 * The following variables represent the GameObjects Transformations in world space.
	 */
	public var x(default, null):Float;
	public var y(default, null):Float;
	
	public var rotation(default, null):Float;
	
	public var scaleX(default, null):Float;
	public var scaleY(default, null):Float;
	
	public var layer(default, null):Int;
	
	public var matrix(get, null):Matrix;
	
	/**
	 * Variables starting with "local" are the offset of this Transform to it's parent.
	 * If this Transform has no parent the variables equal the ones in world space
	 */
	public var localX(default, set):Float;
	public var localY(default, set):Float;
	
	public var localRotation(default, set):Float;
	
	public var localScaleX(default, set):Float;
	public var localScaleY(default, set):Float;
	
	public var localLayer(default, set):Int;
	
	/**
	 * The parent Transform of this Transform.
	 */
	public var parent(default, set):Transform;
	
	/**
	 * All Children of this GameObject.
	 */
	public var children:Array<Transform>;
	
	/**
	 * The top most entity in the hierarchy.
	 */
	public var root(get, never):Transform;
	
	override public function added()
	{
		children = [];
		resetWorld();
		reset();
	}
	
	override public function debugDraw()
	{
		if (!hasComponent(Camera))
		{
			Painter.set(Color.Blue, 1);
			for (camera in Screen.cameras)
			{
				Painter.matrix = camera.matrix * transform.matrix;
				Painter.drawCross(0, 0, 10, 10, 2);
				Painter.matrix = null;
			}
		}
	}
	
	public inline function setPos(x:Float, y:Float):Void
	{
		this.localX = x;
		this.localY = y;
	}
	
	public inline function setScale(scaleX:Float, scaleY:Float):Void
	{
		this.localScaleX = scaleX;
		this.localScaleY = scaleY;
	}
	
	/**
	 * Makes this Transform to the child of another GameObject.
	 * @param	otherGameObject
	 */
	public inline function attachTo(otherGameObject:GameObject):Void
	{
		parent = otherGameObject.transform;
	}
	
	/**
	 * Attaches another GameObject as child.
	 * @param	otherGameObject
	 */
	public inline function attach(otherGameObject:GameObject):Void
	{
		otherGameObject.transform.parent = this;
	}
	
	/**
	 * Removes another GameObject.
	 * @param	otherGameObject
	 */
	public inline function detach(otherGameObject:GameObject)
	{
		otherGameObject.transform.parent = null;
	}
	
	/**
	 * Calculates the distance between this Transform and a point.
	 * @param	x
	 * @param	y
	 * @return distance in px
	 */
	public inline function distanceTo(x:Float, y:Float):Float
	{
		return Math.sqrt(Math.pow(this.x - x, 2) + Math.pow(this.y - y, 2));
	}
	
	/**
	 * Resets the local transformations of this Transform.
	 */
	public inline function reset()
	{
		localX = 0;
		localY = 0;
		localRotation = 0;
		localScaleX = 1;
		localScaleY = 1;
		localLayer = 0;
	}
	
	private function updateWorldTransformation()
	{
		resetWorld();
		matrix = null;
		var current:Transform = this;
		while (current != null)
		{
			x += current.localX;
			y += current.localY;
			rotation += current.localRotation;
			scaleX *= current.localScaleX;
			scaleY *= current.localScaleY;
			layer += current.localLayer;
			
			current = current.parent;
		}
		sendMessage("onTransformChange", this);
		
		for (child in children)
			child.updateWorldTransformation();
	}
	
	private inline function resetWorld()
	{
		x = 0;
		y = 0;
		rotation = 0;
		scaleX = 1;
		scaleY = 1;
		layer = 0;
	}
	
	override public function loadConfig(data:Dynamic):Void
	{
		localRotation = data.rotation.parse(0.0);
		localX = data.centerX.parse(0.0);
		localY = data.centerY.parse(0.0);
		localScaleX = data.scaleX.parse(1.0);
		localScaleY = data.scaleY.parse(1.0);
		
		x = data.x.parse(0.0);
		y = data.y.parse(0.0);
	}
	
	private inline function get_root():Transform
	{
		var transform:Transform = this;
		while (transform.parent != null)
		{
			transform = transform.parent;
		}
		return transform;
	}
	
	private inline function set_parent(value:Transform)
	{ 
		if (value == null)
			parent.children.remove(this);
		else if (!Misc.contains(value.children, this))
			value.children.push(this);
		parent = value;
		updateWorldTransformation();
		return value;
	}
	
	private inline function set_localX(value:Float):Float { localX = value; updateWorldTransformation(); return value; }
	private inline function set_localY(value:Float):Float { localY = value; updateWorldTransformation(); return value; }
	
	private inline function set_localRotation(value:Float):Float { localRotation = value; updateWorldTransformation(); return value; }
	
	private inline function set_localScaleX(value:Float):Float { localScaleX = value; updateWorldTransformation(); return value; }
	private inline function set_localScaleY(value:Float):Float { localScaleY = value; updateWorldTransformation(); return value; }
	
	private inline function set_localLayer(value:Int):Int { localLayer = value; updateWorldTransformation(); return value; }
	
	private function get_matrix():Matrix
	{
		if (matrix == null)
		{
			matrix = Matrix.translation(x, y) *
			 		 Matrix.scale(scaleX, scaleY) *
					 Matrix.rotation(rotation * Math.PI / 180);
		}
		return matrix;
	}
}
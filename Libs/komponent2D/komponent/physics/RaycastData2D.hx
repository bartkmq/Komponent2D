package komponent.physics;

import hxcollision.data.RayData;
import hxcollision.shapes.Ray;
import hxcollision.shapes.Shape;

import komponent.components.Collider;

/**
 * RaycastData with added collider information.
 * See documention of hxCollision.
 */ 
class RaycastData2D
{
	
	public var collider:Collider;
	public var shape:Shape;
	public var ray:Ray;
	
	private var data:RayData;
	
	/**
	 * distance along ray that the intersection occurred at.
	 */
	public var start:Float;
	public var end:Float;
	
	public function new(data:RayData, coll:Collider) 
	{
		this.data = data;
		collider = coll;
	}
	
	private inline function get_shape():Shape { return data.shape; }
	private inline function get_ray():Ray { return data.ray; }
}
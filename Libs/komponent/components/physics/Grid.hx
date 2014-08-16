package komponent.components.physics;

import nape.space.Space;
import nape.shape.Shape;
import nape.shape.Polygon in NapePolygon;

import komponent.components.Collider;

class Grid extends Collider
{
	
	// row = tilesY
	// column = tilesX
	
	public var tileWidth(default, null):Int;
	public var tileHeight(default, null):Int;
	
	public var tilesX(default, null):Int;
	public var tilesY(default, null):Int;
	
	private var shapes:Array<Array<Shape>>;

	public function new() 
	{
		tileWidth = 0;
		tileHeight = 0;
		tilesX = 0;
		tilesY = 0;
		gridData = [];
	}
	
	override private function setDefaultShape()
	{
		
	}
	
	override public function debugDraw()
	{	
		Painter.set(Color.fromBytes(91, 194, 54), 1);
		
		for (camera in Screen.cameras)
		{
			Painter.camera = camera;
			for (shape in shapes)
			{
				Painter.drawPolygon(cast shape, -Screen.camera.x, -Screen.camera.y);
			}
		}
	}
	
	public function loadMap(mapData:Array<Array<Bool>>, tileWidth:Int = 0, tileHeight = 0, tilesX:Int, tilesY:Int)
	{
		for (x in 0...tilesX)
		{
			for (y in 0...tilesY)
			{
				if (mapData[y][x])
					addTile(x, y);
			}
		}
		
		this.tileWidth = tileWidth;
		this.tileHeight = tileHeight;
		this.tilesX = tilesX;
		this.tilesY = tilesY;
	}
	
	public function addTile(tileX:Int, tileY:Int)
	{
		addTileXY(tileX / tileWidth, tileY / tileHeight);
	}
	
	public function addTileXY(x:Int, y:Int)
	{
		var shape:Shape = new NapePolygon(NapePolygon.rect(x - width / 2, y - height / 2, tileWidth, tileHeight);
		if (body.isStatic)
		{
			var space:Space = body.space;
			body.space = null;
		
			body.shapes.add(shape);
			shapes[cast x / tileWidth][cast y / tileHeight] = shape;
		
			body.space = space;
		}
		else
		{
			body.shapes.add(shape);
			shapes[cast x / tileWidth][cast y / tileHeight] = shape;
		}
	}
	
	override public function loadConfig(data:Dynamic)
	{
		super.loadConfig(data);
	}
	
}
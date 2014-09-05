package hxcollision.shapes;

import kha.Painter;

class KhaDrawer
{
	
	public var painter:Painter;

	public function new(painter:Painter) 
	{
		this.painter = painter;
	}
	
	override public function drawLine(p0:Vector, p1:Vector, ?startPoint:Bool = true)
	{
		if (startPoint)
			painter.drawLine(p0.x, p0.y, p1.x, p1.y);
		else
			painter.drawLine(0, 0, p1.x, p1.y);
	}
	
}
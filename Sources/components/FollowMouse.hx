package components;

import komponent.Component;
import komponent.input.Mouse;

class FollowMouse extends Component
{
	
	override public function update()
	{
		transform.localX = Mouse.sceneX;
		transform.localY = Mouse.sceneY;
	}
	
}
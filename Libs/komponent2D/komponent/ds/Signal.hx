package komponent.ds;

using komponent.utils.Misc;

/**
 * A Signal<M> can bind one or more callback functions that will be
 * called with messages of type M.
 * Typically M is some type of event that will be passed to the callback.
 */

class Signal<M>
{
	
	public var listeners:Array<M->Void>;

	public function new()
	{
		listeners = [];
	}

	public function add(callback:M->Void)
	{
		if (listeners.contains(callback))
			listeners.push(callback);
	}

	public function remove(callback:M->Void)
	{
		listeners.remove(callback);
	}

	public function clear()
	{
		listeners.clear();
	}

	public function dispatch(message:M)
	{
		for (callback in listeners) callback(message);
	}
}

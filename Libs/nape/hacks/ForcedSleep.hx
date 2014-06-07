package nape.hacks;

import nape.phys.Body;
import nape.constraint.Constraint;
import nape.phys.Compound;
import nape.space.Space;

//different import for swc (using externs) or haxe (yay)
#if nape_swc
	import zpp_nape.dynamics.ZPP_Arbiter;
#else
	import zpp_nape.dynamics.Arbiter.ZPP_Arbiter;
#end

#if haxe3
    typedef IntHash<T> = Map<Int,T>;
#end

/**
 * Methods for manual sleeping and insertion of sleeping objects to a Space.
 * <br/><br/>
 * Haxe users can make use of the 'using' keyword to permit usage like:
 * <pre>
 * body.sleepBody();
 * space.addSleepingBody(body);
 * </pre>
 * As opposed to the more verbose:
 * <pre>
 * ForcedSleep.sleepBody(body);
 * ForcedSleep.addSleepingBody(space, body);
 * </pre>
 *
 * @requires nape-hacks module.
 */
@:keep class ForcedSleep {
    /**
     * Force Body to go to sleep.
     * <br/><br/>
     * This method will not put to sleep anything that the Body is touching or connected to.
     *
     * @param body The Body to put to sleep.
     * @throws # If body is null, or is not in a Space.
     */
	public static function sleepBody(body:Body) {
		if(body==null) throw "Error: Can't force sleep null body";
		if(body.space==null) throw "Error: This hack requires the Body to already be in a Space!";
		if(body==body.space.world) throw "Error: Cannot force sleep space.world!";
		if(body.space.zpp_inner.midstep) throw "Error: Even this hack can't operate from within space.step()!";

		var b = body.zpp_inner;
		if(b.component.sleeping) return;
		b.component.sleeping = true;

		//remove from live object lists.
		if(body.isDynamic()) b.space.live.remove(b);
		else {
			if(body.isKinematic())
				b.space.kinematics.remove(b);
			b.space.staticsleep.remove(b);
		}

		//force any arbiters to be put to sleep.
		var arbi = b.arbiters.head;
		while(arbi!=null) {
			var arb = arbi.elt;
			if(arb.cleared || arb.sleeping) { arbi = arbi.next; continue; }

			arb.sleeping = true;
			arb.sleep_stamp = b.space.stamp;

			if(arb.type==ZPP_Arbiter.COL) {
				var carb = arb.colarb;
				if(carb.stat) b.space.c_arbiters_true.remove(carb);
				else          b.space.c_arbiters_false.remove(carb);
			}else if(arb.type==ZPP_Arbiter.FLUID)
			     b.space.f_arbiters.remove(arb.fluidarb);
			else b.space.s_arbiters.remove(arb.sensorarb);

			arbi = arbi.next;
		}

		if(!body.isStatic()) {
			b.validate_mass();
			b.validate_inertia();
			b.validate_aabb();
			b.validate_gravMass();
			b.validate_worldCOM();
			b.validate_axis();

			for(shape in body.shapes) {
				var s = shape.zpp_inner;
				if(s.isPolygon())
					s.polygon.validate_gaxi();
			}
		}

		if(!b.space.bphase.is_sweep)
			for(s in body.shapes) b.space.bphase.sync(s.zpp_inner);
	}

    /**
     * Force Constraint to go to sleep.
     * <br/><br/>
     * This method will not put to sleep anything that the Constraint is touching or connected to.
     *
     * @param constraint The Constraint to put to sleep.
     * @throws # If constraint is null, or is not in a Space.
     */
	public static function sleepConstraint(constraint:Constraint) {
		if(constraint==null) throw "Error: Can't force sleep null constraint";
		if(constraint.space==null) throw "Error: This hack requires the Constraint to already be in a Space";
		if(constraint.space.zpp_inner.midstep) throw "Error: Even this hack can't operate from within space.step()!";

		var c = constraint.zpp_inner;
		if(c.component.sleeping) return;
		c.component.sleeping = true;

		c.space.live_constraints.remove(c);
	}

    /**
     * Force Compound to go to sleep.
     * <br/><br/>
     * This method will not put to sleep anything that the Compound's constituents are touching or connected to.
     *
     * @param compound The Compound to put to sleep.
     * @throws # If compound is null, or is not in a Space.
     */
	public static function sleepCompound(compound:Compound) {
		if(compound==null) throw "Error: Can't force sleep null compound";
		if(compound.space==null) throw "Error: This hack requires the Compound to already be in a Space";
		if(compound.space.zpp_inner.midstep) throw "Error: Even this hack can't operate from within space.step()!";

        compound.visitBodies(sleepBody);
		compound.visitConstraints(sleepConstraint);
	}

    /**
     * Force Body, and all Bodies connected to go to sleep.
     * <br/><br/>
     * This method will traverse constraints visible via body.constraints list and put this Body
     * as well as all Bodies it is directly or indirectly connected to; to sleep.
     *
     * @param body The Body to seed search for manual sleeping.
     * @throws # If body is null, or is not in a Space.
     */
	public static function sleepConnected(body:Body) {
		if(body==null) throw "Error: Can't force sleep connected from null Body";
		if(body.space==null) throw "Error: This hack requires the Body to already be in a Space";

		var set = new IntHash<Body>();
		var stack = [body];
		while(stack.length>0) {
			var b = stack.pop();
			if(set.exists(b.id)) continue;

			set.set(b.id,b);
			for(c in b.constraints)
				c.visitBodies(function (b) stack.unshift(b));
		}

		for(b in set) {
			if(b.space==null) throw "Error: This hack requires Body's to already be in a Space regarding body found during sleepConnected";

			if(b!=b.space.world)
				sleepBody(b);

			for(c in b.constraints) sleepConstraint(c);
		}
	}

    /**
     * Add a sleeping Body to a Space.
     * <br/><br/>
     * This method should be used in preference to inserting a Body, and then
     * manually putting it to sleep.
     * <br/><br/>
     * If the Body is already in the Space, then this method will simply put it
     * to sleep. If this Body is in a different Space, then calling this method
     * will first remove the Body from its existing Space.
     *
     * @param space The Space to add Body to.
     * @param body The Body to insert as a sleeping Body.
     * @throws # If space is null, or body is null.
     */
	public static function addSleepingBody(space:Space, body:Body) {
		if(body==null) throw "Error: Cannot add null body to Space";
		if(space==null) throw "Error: Cannot add body to null Space";

		if(body.space==space) { sleepBody(body); return; }
		else if(body.space!=null) body.space = null;

		var b = body.zpp_inner;
		var s = space.zpp_inner;
		//mostly copied from PR(Space)::addBody
		b.space = s;
		b.addedToSpace();
		b.component.sleeping = true;

		//mostly copied from PR(Space)::wake (well the one line anyways :D)
		b.component.waket = s.stamp+1; //not sure about this btw :)
		//addedToSpace will deal with broadphase with actual insertion
		//for dyn-aabb being deferred until step at which point body
		//is asleep and will be inserted into correct tree :)

		//mostly copied from PR(Space)::addBody
		for(shape in body.shapes) s.added_shape(shape.zpp_inner,true);
		if(body.isStatic())
			s.static_validation(b);

		//aaaand one extra step not present in addBody because
		//of how bodies are added
		s.bodies.add(b);

		//also need to do this for non-statics to operate with
		//broadphases correctly
		//mostly copied from PR(Space)::validation
		if(!body.isStatic()) {
			b.validate_mass();
			b.validate_inertia();
			b.validate_aabb();
			b.validate_gravMass();
			b.validate_worldCOM();
			b.validate_axis();

			for(shape in body.shapes) {
				var s = shape.zpp_inner;
				if(s.isPolygon())
					s.polygon.validate_gaxi();
			}
		}
	}

    /**
     * Add a sleeping Constraint to a Space.
     * <br/><br/>
     * This method should be used in preference to inserting a Constraint, and then
     * manually putting it to sleep.
     * <br/><br/>
     * If the Constraint is already in the Space, then this method will simply put it
     * to sleep. If this Constraint is in a different Space, then calling this method
     * will first remove the Constraint from its existing Space.
     *
     * @param space The Space to add Constraint to.
     * @param constraint The Constraint to insert as a sleeping Constraint.
     * @throws # If space is null, or constraint is null.
     */
	public static function addSleepingConstraint(space:Space, constraint:Constraint) {
		if(constraint==null) throw "Error: Cannot add null constraint to Space";
		if(space==null) throw "Error: Cannot add constraint to null Space";

		if(constraint.space==space) { sleepConstraint(constraint); return; }
		else if(constraint.space!=null) constraint.space = null;

		var c = constraint.zpp_inner;
		var s = space.zpp_inner;
		//mostly copied from PR(Space)::addConstraint
		c.space = s;
		c.addedToSpace();
		if(c.active) {
			c.component.sleeping = true;
			//mostly copied from PR(Space)::wake_constraint
			c.component.waket = s.stamp+1; //not sure about this :)
		}

		//also need to do this! (here only)
		s.constraints.add(c);
	}

    /**
     * Add a sleeping Compound to a Space.
     * <br/><br/>
     * This method should be used in preference to inserting the Compound, and then
     * manually putting its constituents to sleep.
     * <br/><br/>
     * If the Compound is already in the Space, then this method will simply put it
     * to sleep. If this Compound is in a different Space, then calling this method
     * will first remove the Compound from its existing Space.
     *
     * @param space The Space to add Compound to.
     * @param constraint The Compound to insert as a sleeping Compound.
     * @throws # If space is null, or compound is null.
     */
	public static function addSleepingCompound(space:Space, compound:Compound) {
		if(compound==null) throw "Error: Cannot add null compound to Space";
		if(space==null) throw "Error: Cannot add compound to null Space";

		if(compound.space==space) {
			compound.visitBodies(sleepBody);
			compound.visitConstraints(sleepConstraint);
		}else if(compound.space!=null) compound.space = null;

		var c = compound.zpp_inner;
		var s = space.zpp_inner;
		//mostly copied from PR(Space)::addCompound
		c.space = s;
		c.addedToSpace();
		for(b in compound.bodies) addSleepingBody(space,b);
		for(c in compound.constraints) addSleepingConstraint(space,c);
		for(c in compound.compounds) addSleepingCompound(space,c);
	}
}

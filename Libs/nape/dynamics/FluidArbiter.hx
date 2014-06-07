package nape.dynamics;
import zpp_nape.Const;
import zpp_nape.constraint.PivotJoint;
import zpp_nape.ID;
import zpp_nape.constraint.Constraint;
import zpp_nape.constraint.WeldJoint;
import zpp_nape.constraint.UserConstraint;
import zpp_nape.constraint.LineJoint;
import zpp_nape.constraint.DistanceJoint;
import zpp_nape.constraint.LinearJoint;
import zpp_nape.constraint.MotorJoint;
import zpp_nape.constraint.AngleJoint;
import zpp_nape.phys.Interactor;
import zpp_nape.phys.FeatureMix;
import zpp_nape.phys.Material;
import zpp_nape.constraint.PulleyJoint;
import zpp_nape.phys.FluidProperties;
import zpp_nape.phys.Compound;
import zpp_nape.callbacks.OptionType;
import zpp_nape.phys.Body;
import zpp_nape.callbacks.CbSetPair;
import zpp_nape.callbacks.CbType;
import zpp_nape.callbacks.Callback;
import zpp_nape.callbacks.CbSet;
import zpp_nape.callbacks.Listener;
import zpp_nape.geom.GeomPoly;
import zpp_nape.geom.Mat23;
import zpp_nape.geom.ConvexRayResult;
import zpp_nape.geom.Cutter;
import zpp_nape.geom.Vec2;
import zpp_nape.geom.Ray;
import zpp_nape.geom.Convex;
import zpp_nape.geom.MatMath;
import zpp_nape.geom.Triangular;
import zpp_nape.geom.PartitionedPoly;
import zpp_nape.geom.Simplify;
import zpp_nape.geom.AABB;
import zpp_nape.geom.SweepDistance;
import zpp_nape.geom.Simple;
import zpp_nape.geom.VecMath;
import zpp_nape.geom.Monotone;
import zpp_nape.geom.Vec3;
import zpp_nape.geom.MatMN;
import zpp_nape.geom.PolyIter;
import zpp_nape.geom.MarchingSquares;
import zpp_nape.geom.Geom;
import zpp_nape.shape.Circle;
import zpp_nape.geom.Collide;
import zpp_nape.shape.Shape;
import zpp_nape.shape.Edge;
import zpp_nape.space.Broadphase;
import zpp_nape.shape.Polygon;
import zpp_nape.space.SweepPhase;
import zpp_nape.space.DynAABBPhase;
import zpp_nape.dynamics.Contact;
import zpp_nape.space.Space;
import zpp_nape.dynamics.InteractionGroup;
import zpp_nape.dynamics.InteractionFilter;
import zpp_nape.dynamics.SpaceArbiterList;
import zpp_nape.dynamics.Arbiter;
import zpp_nape.util.Array2;
import zpp_nape.util.Lists;
import zpp_nape.util.Flags;
import zpp_nape.util.Queue;
import zpp_nape.util.Debug;
import zpp_nape.util.RBTree;
import zpp_nape.util.FastHash;
import zpp_nape.util.WrapLists;
import zpp_nape.util.Pool;
import zpp_nape.util.Names;
import zpp_nape.util.Math;
import zpp_nape.util.UserData;
import zpp_nape.util.DisjointSetForest;
import nape.TArray;
import nape.Config;
import zpp_nape.util.Circular;
import nape.constraint.PivotJoint;
import nape.constraint.WeldJoint;
import nape.constraint.UserConstraint;
import nape.constraint.Constraint;
import nape.constraint.DistanceJoint;
import nape.constraint.LineJoint;
import nape.constraint.LinearJoint;
import nape.constraint.ConstraintList;
import nape.constraint.AngleJoint;
import nape.constraint.MotorJoint;
import nape.constraint.ConstraintIterator;
import nape.phys.GravMassMode;
import nape.phys.BodyList;
import nape.phys.Interactor;
import nape.phys.InertiaMode;
import nape.phys.InteractorList;
import nape.constraint.PulleyJoint;
import nape.phys.MassMode;
import nape.phys.Material;
import nape.phys.InteractorIterator;
import nape.phys.FluidProperties;
import nape.phys.BodyIterator;
import nape.phys.Compound;
import nape.phys.CompoundList;
import nape.phys.BodyType;
import nape.phys.CompoundIterator;
import nape.callbacks.InteractionListener;
import nape.callbacks.OptionType;
import nape.callbacks.PreListener;
import nape.callbacks.BodyListener;
import nape.callbacks.ListenerIterator;
import nape.callbacks.CbType;
import nape.callbacks.ListenerType;
import nape.callbacks.PreFlag;
import nape.callbacks.CbEvent;
import nape.callbacks.InteractionType;
import nape.callbacks.PreCallback;
import nape.callbacks.InteractionCallback;
import nape.phys.Body;
import nape.callbacks.ListenerList;
import nape.callbacks.BodyCallback;
import nape.callbacks.CbTypeList;
import nape.callbacks.Callback;
import nape.callbacks.ConstraintListener;
import nape.callbacks.CbTypeIterator;
import nape.callbacks.ConstraintCallback;
import nape.callbacks.Listener;
import nape.geom.Mat23;
import nape.geom.ConvexResultIterator;
import nape.geom.GeomPoly;
import nape.geom.Ray;
import nape.geom.GeomPolyIterator;
import nape.geom.Vec2Iterator;
import nape.geom.RayResult;
import nape.geom.Winding;
import nape.geom.Vec2List;
import nape.geom.RayResultIterator;
import nape.geom.AABB;
import nape.geom.IsoFunction;
import nape.geom.GeomVertexIterator;
import nape.geom.ConvexResult;
import nape.geom.GeomPolyList;
import nape.geom.Vec2;
import nape.geom.RayResultList;
import nape.geom.Vec3;
import nape.geom.MatMN;
import nape.geom.MarchingSquares;
import nape.geom.ConvexResultList;
import nape.shape.Circle;
import nape.shape.ValidationResult;
import nape.geom.Geom;
import nape.shape.ShapeIterator;
import nape.shape.Polygon;
import nape.shape.Edge;
import nape.shape.EdgeList;
import nape.shape.Shape;
import nape.shape.EdgeIterator;
import nape.shape.ShapeList;
import nape.shape.ShapeType;
import nape.space.Broadphase;
import nape.dynamics.Contact;
import nape.dynamics.InteractionGroupList;
import nape.dynamics.Arbiter;
import nape.dynamics.InteractionGroup;
import nape.dynamics.ContactIterator;
import nape.dynamics.InteractionFilter;
import nape.dynamics.ArbiterList;
import nape.space.Space;
import nape.dynamics.ArbiterIterator;
import nape.dynamics.InteractionGroupIterator;
import nape.dynamics.ContactList;
import nape.dynamics.ArbiterType;
import nape.dynamics.CollisionArbiter;
import nape.util.Debug;
import nape.util.BitmapDebug;
import nape.util.ShapeDebug;
/**
 * Fluid interaction subtype for Arbiter.
 */
@:final#if nape_swc@:keep #end
class FluidArbiter extends Arbiter{
    /**
     * Centre of buoyancy for fluid interaction.
     * <br/><br/>
     * This value can be modified during a related PreListener handler.
     */
    #if nape_swc@:isVar #end
    public var position(get_position,set_position):Vec2;
    inline function get_position():Vec2{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.inactiveme())throw "Error: Arbiter not currently in use";
        #end
        if(zpp_inner.fluidarb.wrap_position==null)zpp_inner.fluidarb.getposition();
        return zpp_inner.fluidarb.wrap_position;
    }
    inline function set_position(position:Vec2):Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(!zpp_inner.fluidarb.mutable)throw "Error: Arbiter is mutable only within a pre-handler";
            #end
            #if(!NAPE_RELEASE_BUILD)
            if(position==null)throw "Error: FluidArbiter::position cannot be null";
            #end
            this.position.set(position);
        }
        return get_position();
    }
    /**
     * Overlap area of Shapes in fluid interaction.
     * <br/><br/>
     * This value is strictly positive, and represents the amount of overlap between the Shapes
     * used in buoyancy computations.
     * <br/><br/>
     * This value can be modified during a related PreListener handler.
     */
    #if nape_swc@:isVar #end
    public var overlap(get_overlap,set_overlap):Float;
    inline function get_overlap():Float{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.inactiveme())throw "Error: Arbiter not currently in use";
        #end
        return zpp_inner.fluidarb.overlap;
    }
    inline function set_overlap(overlap:Float):Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(!zpp_inner.fluidarb.mutable)throw "Error: Arbiter is mutable only within a pre-handler";
            #end
            #if(!NAPE_RELEASE_BUILD)
            if((overlap!=overlap))throw "Error: FluidArbiter::overlap cannot be NaN";
            #end
            #if(!NAPE_RELEASE_BUILD)
            if(overlap<=0||overlap==Math.POSITIVE_INFINITY)throw "Error: FluidArbiter::overlap must be strictly positive and non infinite";
            #end
            zpp_inner.fluidarb.overlap=overlap;
        }
        return get_overlap();
    }
    /**
     * Determine impulse on a given body due to buoyancy.
     * <br/><br/>
     * If the body is null, then the buoyancy impulse will be returned without consideration to any specific
     * body involved, and no angular impulses can be derived.
     *
     * @param body The body to query impulse for. (default null)
     * @return The buoyancy impulse for given body.
     * @throws # If body is non-null, and unrelated to this Arbiter.
     */
    #if nape_swc@:keep #end
    public function buoyancyImpulse(body:Body=null):Vec3{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.inactiveme())throw "Error: Arbiter not currently in use";
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(body!=null&&body!=body1&&body!=body2)throw "Error: Arbiter does not relate to body";
        #end
        var farb=zpp_inner.fluidarb;
        if(body==null){
            return Vec3.get(farb.buoyx,farb.buoyy,0);
        }
        else if(body.zpp_inner==zpp_inner.b2){
            return Vec3.get(farb.buoyx,farb.buoyy,(farb.buoyy*farb.r2x-farb.buoyx*farb.r2y));
        }
        else{
            return Vec3.get(-farb.buoyx,-farb.buoyy,-(farb.buoyy*farb.r1x-farb.buoyx*farb.r1y));
        }
    }
    /**
     * Determine impulse on a given body due to fluid drag.
     * <br/><br/>
     * If the body is null, then the drag impulse will be returned without consideration to any specific
     * body involved.
     *
     * @param body The body to query impulse for. (default null)
     * @return The drag impulse for given body.
     * @throws # If body is non-null, and unrelated to this Arbiter.
     */
    #if nape_swc@:keep #end
    public function dragImpulse(body:Body=null):Vec3{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.inactiveme())throw "Error: Arbiter not currently in use";
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(body!=null&&body!=body1&&body!=body2)throw "Error: Arbiter does not relate to body";
        #end
        var farb=zpp_inner.fluidarb;
        var scale=(body==null||body.zpp_inner==zpp_inner.b2?1:-1);
        return Vec3.get(farb.dampx*scale,farb.dampy*scale,farb.adamp*scale);
    }
    /**
     * Determine total impulse on a given body due to fluid interaction.
     * <br/><br/>
     * If the body is null, then the total impulse will be computed without consideration to any specific
     * body involved, and no angular impulses can be derived for the linear portion of the impulses.
     *
     * @param body The body to query impulse for. (default null)
     * @param freshOnly This parameter is unused for FluidArbiters. (default false)
     * @return The total impulse for given body.
     * @throws # If body is non-null, and unrelated to this Arbiter.
     */
    #if nape_swc@:keep #end
    public override function totalImpulse(body:Body=null,freshOnly:Bool=false){
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.inactiveme())throw "Error: Arbiter not currently in use";
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(body!=null&&body!=body1&&body!=body2)throw "Error: Arbiter does not relate to body";
        #end
        var tmp=this.buoyancyImpulse(body);
        var ret=this.dragImpulse(body);
        ret.x+=tmp.x;
        ret.y+=tmp.y;
        ret.z+=tmp.z;
        tmp.dispose();
        return ret;
    }
    /**
     * @private
     */
    public function new(){
        #if(!NAPE_RELEASE_BUILD)
        if(!ZPP_Arbiter.internal)throw "Error: Cannot instantiate FluidArbiter derp!";
        #end
        super();
    }
}

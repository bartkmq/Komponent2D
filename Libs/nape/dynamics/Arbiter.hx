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
import nape.dynamics.InteractionGroup;
import nape.dynamics.ContactIterator;
import nape.dynamics.InteractionFilter;
import nape.dynamics.ArbiterList;
import nape.space.Space;
import nape.dynamics.ArbiterIterator;
import nape.dynamics.InteractionGroupIterator;
import nape.dynamics.FluidArbiter;
import nape.dynamics.ContactList;
import nape.dynamics.ArbiterType;
import nape.dynamics.CollisionArbiter;
import nape.util.Debug;
import nape.util.BitmapDebug;
import nape.util.ShapeDebug;
/**
 * Arbiter representing the state of an interaction between two Bodys.
 * <br/><br/>
 * These objects are automatically reused, and you should not keep your own
 * references to them.
 */
#if nape_swc@:keep #end
class Arbiter{
    /**
     * @private
     */
    public var zpp_inner:ZPP_Arbiter=null;
    /**
     * Flag representing arbiter sleep state.
     * <br/><br/>
     * When true, this arbiter is sleeping.
     */
    #if nape_swc@:isVar #end
    public var isSleeping(get_isSleeping,never):Bool;
    inline function get_isSleeping():Bool{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.inactiveme())throw "Error: Arbiter not currently in use";
        #end
        return zpp_inner.sleeping;
    }
    /**
     * The type of this Arbiter.
     */
    #if nape_swc@:isVar #end
    public var type(get_type,never):ArbiterType;
    inline function get_type():ArbiterType{
        return ZPP_Arbiter.types[zpp_inner.type];
    }
    /**
     * Equivalent to: <code>arb.type == ArbiterType.COLLISION</code>
     * </br><br/>
     *
     * @return True if this Arbiter is a Collision type arbiter.
     */
    #if nape_swc@:keep #end
    public#if NAPE_NO_INLINE#else inline #end
    function isCollisionArbiter(){
        return zpp_inner.type==ZPP_Arbiter.COL;
    }
    /**
     * Equivalent to: <code>arb.type == ArbiterType.FLUID</code>
     * </br><br/>
     *
     * @return True if this Arbiter is a Fluid type arbiter.
     */
    #if nape_swc@:keep #end
    public#if NAPE_NO_INLINE#else inline #end
    function isFluidArbiter(){
        return zpp_inner.type==ZPP_Arbiter.FLUID;
    }
    /**
     * Equivalent to: <code>arb.type == ArbiterType.SENSOR</code>
     * </br><br/>
     *
     * @return True if this Arbiter is a Sensor type arbiter.
     */
    #if nape_swc@:keep #end
    public#if NAPE_NO_INLINE#else inline #end
    function isSensorArbiter(){
        return zpp_inner.type==ZPP_Arbiter.SENSOR;
    }
    /**
     * Fast equivalent to casting this object to a CollisionArbiter.
     * <br/><br/>
     * This value is null when this arbiter is not a collision type.
     */
    #if nape_swc@:isVar #end
    public var collisionArbiter(get_collisionArbiter,never):Null<CollisionArbiter>;
    inline function get_collisionArbiter():Null<CollisionArbiter>{
        return if(isCollisionArbiter())zpp_inner.colarb.outer_zn else null;
    }
    /**
     * Fast equivalent to casting this object to a FluidArbiter.
     * <br/><br/>
     * This value is null when this arbiter is not a fluid type.
     */
    #if nape_swc@:isVar #end
    public var fluidArbiter(get_fluidArbiter,never):Null<FluidArbiter>;
    inline function get_fluidArbiter():Null<FluidArbiter>{
        return if(isFluidArbiter())zpp_inner.fluidarb.outer_zn else null;
    }
    /**
     * The first shape in Arbiter interaction.
     * <br/><br/>
     * It will always be the case that <code>arb.shape1.id < arb.shape2.id</code>
     */
    #if nape_swc@:isVar #end
    public var shape1(get_shape1,never):Shape;
    inline function get_shape1():Shape{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.inactiveme())throw "Error: Arbiter not currently in use";
        #end
        return(zpp_inner.ws1.id>zpp_inner.ws2.id)?zpp_inner.ws2.outer:zpp_inner.ws1.outer;
    }
    /**
     * The second shape in Arbiter interaction.
     * <br/><br/>
     * It will always be the case that <code>arb.shape1.id < arb.shape2.id</code>
     */
    #if nape_swc@:isVar #end
    public var shape2(get_shape2,never):Shape;
    inline function get_shape2():Shape{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.inactiveme())throw "Error: Arbiter not currently in use";
        #end
        return(zpp_inner.ws1.id>zpp_inner.ws2.id)?zpp_inner.ws1.outer:zpp_inner.ws2.outer;
    }
    /**
     * The first body in Arbiter interaction.
     * <br/><br/>
     * It will always be the case that <code>arb.shape1.body == arb.body1</code>
     */
    #if nape_swc@:isVar #end
    public var body1(get_body1,never):Body;
    inline function get_body1():Body{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.inactiveme())throw "Error: Arbiter not currently in use";
        #end
        return(zpp_inner.ws1.id>zpp_inner.ws2.id)?zpp_inner.b2.outer:zpp_inner.b1.outer;
    }
    /**
     * The second body in Arbiter interaction.
     * <br/><br/>
     * It will always be the case that <code>arb.shape2.body == arb.body2</code>
     */
    #if nape_swc@:isVar #end
    public var body2(get_body2,never):Body;
    inline function get_body2():Body{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.inactiveme())throw "Error: Arbiter not currently in use";
        #end
        return(zpp_inner.ws1.id>zpp_inner.ws2.id)?zpp_inner.b1.outer:zpp_inner.b2.outer;
    }
    /**
     * The interaction state of this Arbiter.
     * <br/><br/>
     * This flag will, except for in a PreListener handler, always be either
     * <code>ImmState.ACCEPT</code> or <code>ImmState.IGNORE</code>
     * <br/>
     * During a PreListener handler, you can query this property to see what
     * the current state of the arbiter has been set to, and returning null from
     * the handler will keep the state unchanged.
     */
    #if nape_swc@:isVar #end
    public var state(get_state,never):PreFlag;
    inline function get_state():PreFlag{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.inactiveme())throw "Error: Arbiter not currently in use";
        #end
        return switch(zpp_inner.immState){
            case x if(x==ZPP_Flags.id_ImmState_ACCEPT|ZPP_Flags.id_ImmState_ALWAYS):PreFlag.ACCEPT;
            case ZPP_Flags.id_ImmState_ACCEPT:PreFlag.ACCEPT_ONCE;
            case x if(x==ZPP_Flags.id_ImmState_IGNORE|ZPP_Flags.id_ImmState_ALWAYS):PreFlag.IGNORE;
            default:PreFlag.IGNORE_ONCE;
        }
    }
    /**
     * Evaluate the total impulse this arbiter applied to the given body for
     * the previous space step including angular impulse based on things like
     * contact position, or centre of buoyancy etc.
     * <br/><br/>
     * If body is null, then the constraint space impulse will be returned instead
     *
     * @param body The body to query impulse for. (default null)
     * @param freshOnly If true, then only 'new' contact points will be queried for
     *                  collision type arbiters. This field has no use on fluid type
     *                  arbiters. (default false)
     * @return The total impulse applied to the given body, or the constraint
     *         space impule if the body is null.
     * @throws # If body is non-null, but not related to this Arbiter.
     */
    #if nape_swc@:keep #end
    public function totalImpulse(body:Body=null,freshOnly:Bool=false){
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.inactiveme())throw "Error: Arbiter not currently in use";
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(body!=null&&body!=body1&&body!=body2)throw "Error: Arbiter does not relate to body";
        #end
        return Vec3.get(0,0,0);
    }
    /**
     * @private
     */
    public function new(){
        if(!ZPP_Arbiter.internal){
            #if(!NAPE_RELEASE_BUILD)
            throw "Error: Cannot instantiate Arbiter derp!";
            #end
        }
    }
    /**
     * @private
     */
    @:keep public function toString(){
        var ret=if(isCollisionArbiter())"CollisionArbiter";
        else if(isFluidArbiter())"FluidArbiter";
        else "SensorArbiter";
        #if NAPE_POOL_STATS ret+="#"+zpp_inner.arbid;
        #end
        if(zpp_inner.cleared)return ret+"(object-pooled)";
        else return ret+"("+shape1.toString()+"|"+shape2.toString()+")"+(isCollisionArbiter()?"["+["SD","DD"][zpp_inner.colarb.stat?0:1]+"]":"")+"<-"+state.toString();
    }
}

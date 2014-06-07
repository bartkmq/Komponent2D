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
 * InteractionFilter provides bit flags for low-level filtering of interactions.
 * <br/><br/>
 * For a given interaction type, two Shapes will be permitted to interact only if
 * <code>(shape1.group & shape2.mask) != 0 && (shape2.group & shape1.mask) != 0</code>
 * <br/><br/>
 * There are 32 real groups corresponding to a set bit in the group/mask fields. For instance
 * a group value of 0x120 corresponds to the 'real' groups 5 and 8 as <code>0x120 = (1<<5) | (1<<8)</code>
 * <br/><br/>
 * Nape provides group/mask for each interaction type. The actual precedence of interactions
 * is further defined simply as: Sensor > Fluid > Collision.
 * <br/>
 * Two static bodies can never interact, and with the exception of sensor interaction, at least one
 * of the two bodies must be dynamic.
 * <br/>
 * Sensor interactions have the highest precedence, followed by fluid and then collisions.
 * Sensor interaction is permitted only if one of the shapes is sensorEnabled, whilst fluid
 * is permitted only if one of the shapes is fluidEnabled.
 * <pre>
 * if ((shapeA.sensorEnabled || shapeB.sensorEnabled) && shapeA.filter.shouldSense(shapeB.filter)) {
 *     SENSOR INTERACTION!!
 * }
 * else if (bodyA.isDynamic() || bodyB.isDynamic()) {
 *     if ((shapeA.fluidEnabled || shapeB.fluidEnabled) && shapeA.filter.shouldFlow(shapeB.filter)) {
 *         FLUID INTERACTION!!
 *     }
 *     else if (shapeA.filter.shouldCollide(shapeB.filter)) {
 *         COLLISION INTERACTION!!
 *     }
 * }
 * </pre>
 */
@:final#if nape_swc@:keep #end
class InteractionFilter{
    /**
     * @private
     */
    public var zpp_inner:ZPP_InteractionFilter=null;
    /**
     * Dynamic object for user to store additional data.
     * <br/><br/>
     * This object cannot be set, only its dynamically created
     * properties may be set. In AS3 the type of this property is &#42
     * <br/><br/>
     * This object will be lazily constructed so that until accessed
     * for the first time, will be null internally.
     *
     * @default {}
     */
    #if nape_swc@:isVar #end
    public var userData(get_userData,never):Dynamic<Dynamic>;
    inline function get_userData():Dynamic<Dynamic>{
        if(zpp_inner.userData==null){
            zpp_inner.userData=cast{};
        }
        return zpp_inner.userData;
    }
    /**
     * Set of all active shapes using this object.
     * <br/><br/>
     * Activeness of a shape in the sense that the Shape's Body is inside of a Space.
     * <br/><br/>
     * This list is immutable.
     */
    #if nape_swc@:isVar #end
    public var shapes(get_shapes,never):ShapeList;
    inline function get_shapes():ShapeList{
        if(zpp_inner.wrap_shapes==null)zpp_inner.wrap_shapes=ZPP_ShapeList.get(zpp_inner.shapes,true);
        return zpp_inner.wrap_shapes;
    }
    /**
     * Group bitfield for Collision type interactions.
     * @default 1
     */
    #if nape_swc@:isVar #end
    public var collisionGroup(get_collisionGroup,set_collisionGroup):Int;
    inline function get_collisionGroup():Int{
        return zpp_inner.collisionGroup;
    }
    inline function set_collisionGroup(collisionGroup:Int):Int{
        {
            if(this.collisionGroup!=collisionGroup){
                zpp_inner.collisionGroup=collisionGroup;
                zpp_inner.invalidate();
            }
        }
        return get_collisionGroup();
    }
    /**
     * Mask bitfield for Collision type interactions.
     * @default -1 (all bits set)
     */
    #if nape_swc@:isVar #end
    public var collisionMask(get_collisionMask,set_collisionMask):Int;
    inline function get_collisionMask():Int{
        return zpp_inner.collisionMask;
    }
    inline function set_collisionMask(collisionMask:Int):Int{
        {
            if(this.collisionMask!=collisionMask){
                zpp_inner.collisionMask=collisionMask;
                zpp_inner.invalidate();
            }
        }
        return get_collisionMask();
    }
    /**
     * Group bitfield for Sensor type interactions.
     * @default 1
     */
    #if nape_swc@:isVar #end
    public var sensorGroup(get_sensorGroup,set_sensorGroup):Int;
    inline function get_sensorGroup():Int{
        return zpp_inner.sensorGroup;
    }
    inline function set_sensorGroup(sensorGroup:Int):Int{
        {
            if(this.sensorGroup!=sensorGroup){
                zpp_inner.sensorGroup=sensorGroup;
                zpp_inner.invalidate();
            }
        }
        return get_sensorGroup();
    }
    /**
     * Mask bitfield for Sensor type interactions.
     * @default -1 (all bits set)
     */
    #if nape_swc@:isVar #end
    public var sensorMask(get_sensorMask,set_sensorMask):Int;
    inline function get_sensorMask():Int{
        return zpp_inner.sensorMask;
    }
    inline function set_sensorMask(sensorMask:Int):Int{
        {
            if(this.sensorMask!=sensorMask){
                zpp_inner.sensorMask=sensorMask;
                zpp_inner.invalidate();
            }
        }
        return get_sensorMask();
    }
    /**
     * Group bitfield for Fluid type interactions.
     * @default 1
     */
    #if nape_swc@:isVar #end
    public var fluidGroup(get_fluidGroup,set_fluidGroup):Int;
    inline function get_fluidGroup():Int{
        return zpp_inner.fluidGroup;
    }
    inline function set_fluidGroup(fluidGroup:Int):Int{
        {
            if(this.fluidGroup!=fluidGroup){
                zpp_inner.fluidGroup=fluidGroup;
                zpp_inner.invalidate();
            }
        }
        return get_fluidGroup();
    }
    /**
     * Mask bitfield for Fluid type interactions.
     * @default -1 (all bits set)
     */
    #if nape_swc@:isVar #end
    public var fluidMask(get_fluidMask,set_fluidMask):Int;
    inline function get_fluidMask():Int{
        return zpp_inner.fluidMask;
    }
    inline function set_fluidMask(fluidMask:Int):Int{
        {
            if(this.fluidMask!=fluidMask){
                zpp_inner.fluidMask=fluidMask;
                zpp_inner.invalidate();
            }
        }
        return get_fluidMask();
    }
    /**
     * Construct a new InteractionFilter.
     *
     * @param collisionGroup The Group bitfield for Collision interactions. (default 1)
     * @param collisionMask  The Mask bitfield for Collision interactions. (default -1)
     * @param sensorGroup    The Group bitfield for Sensor interactions. (default 1)
     * @param sensorMask     The Mask bitfield for Sensor interactions. (default -1)
     * @param fluidGroup     The Group bitfield for Fluid interactions. (default 1)
     * @param fluidMask      The Mask bitfield for Fluid interactions. (default -1)
     * @return The newly constructed InteractionFilter.
     */
    #if flib@:keep function flibopts_6(){}
    #end
    public function new(collisionGroup=1,collisionMask=-1,sensorGroup=1,sensorMask=-1,fluidGroup=1,fluidMask=-1){
        {
            if(ZPP_InteractionFilter.zpp_pool==null){
                zpp_inner=new ZPP_InteractionFilter();
                #if NAPE_POOL_STATS ZPP_InteractionFilter.POOL_TOT++;
                ZPP_InteractionFilter.POOL_ADDNEW++;
                #end
            }
            else{
                zpp_inner=ZPP_InteractionFilter.zpp_pool;
                ZPP_InteractionFilter.zpp_pool=zpp_inner.next;
                zpp_inner.next=null;
                #if NAPE_POOL_STATS ZPP_InteractionFilter.POOL_CNT--;
                ZPP_InteractionFilter.POOL_ADD++;
                #end
            }
            zpp_inner.alloc();
        };
        zpp_inner.outer=this;
        this.collisionGroup=collisionGroup;
        this.collisionMask=collisionMask;
        this.sensorGroup=sensorGroup;
        this.sensorMask=sensorMask;
        this.fluidGroup=fluidGroup;
        this.fluidMask=fluidMask;
    }
    /**
     * Determine if objects are permitted to collide based on InteractionFilters
     * <br/><br/>
     * A collision type interaction can occur only if this returns True.
     *
     * @param filter The filter to evaluate possibility of collision with.
     * @return True, if based on interaction filters only the two objects would be able to collide.
     * @throws # If filter is null.
     */
    #if nape_swc@:keep #end
    public function shouldCollide(filter:InteractionFilter){
        #if(!NAPE_RELEASE_BUILD)
        if(filter==null)throw "Error: filter argument cannot be null for shouldCollide";
        #end
        return zpp_inner.shouldCollide(filter.zpp_inner);
    }
    /**
     * Determine if objects are permitted to sense based on InteractionFilters
     * <br/><br/>
     * A sensor type interaction can occur only if this returns True.
     *
     * @param filter The filter to evaluate possibility of sensor with.
     * @return True, if based on interaction filters only the two objects would be able to sense.
     * @throws # If filter is null.
     */
    #if nape_swc@:keep #end
    public function shouldSense(filter:InteractionFilter){
        #if(!NAPE_RELEASE_BUILD)
        if(filter==null)throw "Error: filter argument cannot be null for shouldSense";
        #end
        return zpp_inner.shouldSense(filter.zpp_inner);
    }
    /**
     * Determine if objects are permitted to interact as fluids based on InteractionFilters
     * <br/><br/>
     * A fluid type interaction can occur only if this returns True.
     *
     * @param filter The filter to evaluate possibility of fluid with.
     * @return True, if based on interaction filters only the two objects would be able to interact as fluids.
     * @throws # If filter is null.
     */
    #if nape_swc@:keep #end
    public function shouldFlow(filter:InteractionFilter){
        #if(!NAPE_RELEASE_BUILD)
        if(filter==null)throw "Error: filter argument cannot be null for shouldFlow";
        #end
        return zpp_inner.shouldFlow(filter.zpp_inner);
    }
    /**
     * Produce a copy of this InteractionFilter
     *
     * @return The copy of this filter.
     */
    #if nape_swc@:keep #end
    public function copy(){
        return new InteractionFilter(collisionGroup,collisionMask,sensorGroup,sensorMask,fluidGroup,fluidMask);
    }
    /**
     * @private
     */
    #if nape_swc@:keep #end
    public function toString(){
        return "{ collision: "+StringTools.hex(collisionGroup,8)+"~"+StringTools.hex(collisionMask,8)+" sensor: "+StringTools.hex(sensorGroup,8)+"~"+StringTools.hex(sensorMask,8)+" fluid: "+StringTools.hex(fluidGroup,8)+"~"+StringTools.hex(fluidMask,8)+" }";
    }
}

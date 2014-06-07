package nape.geom;
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
import nape.dynamics.FluidArbiter;
import nape.dynamics.ContactList;
import nape.dynamics.ArbiterType;
import nape.dynamics.CollisionArbiter;
import nape.util.Debug;
import nape.util.BitmapDebug;
import nape.util.ShapeDebug;
/**
 * Parametrically defined ray used in ray casting functions.
 */
@:final#if nape_swc@:keep #end
class Ray{
    /**
     * @private
     */
    public var zpp_inner:ZPP_Ray=null;
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
     * Origin of ray.
     * <br/><br/>
     * This property can be set, and is equivalent to performing:
     * <code>ray.origin.set(newOrigin)</code>
     */
    #if nape_swc@:isVar #end
    public var origin(get_origin,set_origin):Vec2;
    inline function get_origin():Vec2{
        return zpp_inner.origin;
    }
    inline function set_origin(origin:Vec2):Vec2{
        {
            {
                #if(!NAPE_RELEASE_BUILD)
                if(origin!=null&&origin.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                #end
            };
            #if(!NAPE_RELEASE_BUILD)
            if(origin==null){
                throw "Error: Ray::origin cannot be null";
            }
            #end
            zpp_inner.origin.set(origin);
        }
        return get_origin();
    }
    /**
     * Direction of ray.
     * <br/><br/>
     * This property can be set, and is equivalent to performing:
     * <code>ray.direction.set(newDirection)</code> with the additional
     * constraint that the input direction must not be degenerate.
     * <br/><br/>
     * This direction vector need not be normalised.
     */
    #if nape_swc@:isVar #end
    public var direction(get_direction,set_direction):Vec2;
    inline function get_direction():Vec2{
        return zpp_inner.direction;
    }
    inline function set_direction(direction:Vec2):Vec2{
        {
            {
                #if(!NAPE_RELEASE_BUILD)
                if(direction!=null&&direction.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                #end
            };
            #if(!NAPE_RELEASE_BUILD)
            if(direction==null){
                throw "Error: Ray::direction cannot be null";
            }
            #end
            zpp_inner.direction.set(direction);
            zpp_inner.invalidate_dir();
        }
        return get_direction();
    }
    /**
     * The maximum distance for ray to be queried.
     * <br/><br/>
     * When used in ray test functions, no search will extend beyond this
     * distance.
     * <br/><br/>
     * This value represents a true distance, even if direction vector is
     * not normalised. This value may be equal to infinity.
     *
     * @default infinity
     */
    #if nape_swc@:isVar #end
    public var maxDistance(get_maxDistance,set_maxDistance):Float;
    inline function get_maxDistance():Float{
        return zpp_inner.maxdist;
    }
    inline function set_maxDistance(maxDistance:Float):Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if((maxDistance!=maxDistance)){
                throw "Error: maxDistance cannot be NaN";
            }
            #end
            zpp_inner.maxdist=maxDistance;
        }
        return get_maxDistance();
    }
    /**
     * Compute bounding box of ray.
     * <br/><br/>
     * This function will take into account the maxDistance property of this ray.
     * <br/>
     * The return AABB may have in the general case infinite values :)
     *
     * @return An AABB representing bounding box of ray.
     */
    public function aabb():AABB{
        return zpp_inner.rayAABB().wrapper();
    }
    /**
     * Compute point along ray at given distance.
     * <br/><br/>
     * Even if ray direction is not normalised, this value still repersents
     * a true distance. The distance may also be negative.
     * <br/><br/>
     * The Vec2 returned will be allocated from the global object pool.
     *
     * @param distance The distance along ray to compute point for.
     * @param weak If true then a weakly allocated Vec2 will be returned
     *             which will be automatically released to global object
     *             pool when used as argument to another Nape function.
     *             (default false)
     * @return Vec2 representing point at given distance along ray.
     */
    public function at(distance:Float,weak:Bool=false):Vec2{
        zpp_inner.validate_dir();
        return Vec2.get(origin.x+(distance*zpp_inner.dirx),origin.y+(distance*zpp_inner.diry),weak);
    }
    /**
     * Construct new Ray.
     *
     * @param origin Origin of ray.
     * @param direction Direction of ray.
     * @throws # If origin or direction are null, or disposed of.
     * @throws # If direction is degenerate.
     */
    #if flib@:keep function flibopts_0(){}
    #end
    public function new(origin:Vec2,direction:Vec2){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(origin!=null&&origin.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(direction!=null&&direction.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        zpp_inner=new ZPP_Ray();
        this.origin=origin;
        this.direction=direction;
        zpp_inner.maxdist=ZPP_Const.POSINF();
    }
    /**
     * Produce a copy of this ray.
     * <br/><br/>
     * All ray properties will be copied including maxDistance.
     *
     * @return The copy of this Ray.
     */
    public function copy(){
        var ret=new Ray(origin,direction);
        ret.maxDistance=maxDistance;
        return ret;
    }
    /**
     * Create ray representing a line segment.
     * <br/><br/>
     * This function will a ray who's origin is the start point
     * and who's direction is towards the end point with the
     * maxDistance property appropriately set to not extend
     * beyond the end point.
     *
     * @param start Start point of line segment
     * @param end End point of line segment
     * @return A Ray representing this line segment.
     * @throws # If start or end are either null or disposed of.
     * @throws # If start and end point are equal so that the
     *         direction of the ray would be degenerate.
     */
    public static function fromSegment(start:Vec2,end:Vec2){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(start!=null&&start.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(end!=null&&end.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(start==null){
            throw "Error: Ray::fromSegment::start is null";
        }
        if(end==null){
            throw "Error: Ray::fromSegment::end is null";
        }
        #end
        var dir=end.sub(start,true);
        var ret=new Ray(start,dir);
        ret.maxDistance=Math.sqrt(ZPP_VecMath.vec_dsq(start.x,start.y,end.x,end.y));
        ({
            if(({
                start.zpp_inner.weak;
            })){
                start.dispose();
                true;
            }
            else{
                false;
            }
        });
        ({
            if(({
                end.zpp_inner.weak;
            })){
                end.dispose();
                true;
            }
            else{
                false;
            }
        });
        return ret;
    }
}

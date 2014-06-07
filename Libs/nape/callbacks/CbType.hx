package nape.callbacks;
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
import nape.dynamics.FluidArbiter;
import nape.dynamics.ContactList;
import nape.dynamics.ArbiterType;
import nape.dynamics.CollisionArbiter;
import nape.util.Debug;
import nape.util.BitmapDebug;
import nape.util.ShapeDebug;
/**
 * Callback Type applied to Interactors and Constraints.
 * <br/><br/>
 * Callback types are ranged over by listeners.
 */
@:final#if nape_swc@:keep #end
class CbType{
    /**
     * @private
     */
    public var zpp_inner:ZPP_CbType=null;
    /**
     * Unique identifier for this CbType.
     */
    #if nape_swc@:isVar #end
    public var id(get_id,never):Int;
    inline function get_id():Int{
        return zpp_inner.id;
    }
    /**
     * Construct a new CbType object.
     *
     * @return A new CbType.
     */
    public function new(){
        zpp_inner=new ZPP_CbType();
        zpp_inner.outer=this;
    }
    
    /**
     * Default CbType given to all Bodys
     *
     * Due to the way the Callback system in Nape works, you can use this
     * CbType to match against 'all'
     * Bodys
     * In a Listener (Assuming you do not 'remove' this type from the object)
     */
    #if nape_swc@:isVar #end
    public static var ANY_BODY(get_ANY_BODY,never):CbType;
    inline static function get_ANY_BODY():CbType{
        return ZPP_CbType.ANY_BODY;
    }
    /**
     * Default CbType given to all Constraints
     *
     * Due to the way the Callback system in Nape works, you can use this
     * CbType to match against 'all'
     * Constraints
     * In a Listener (Assuming you do not 'remove' this type from the object)
     */
    #if nape_swc@:isVar #end
    public static var ANY_CONSTRAINT(get_ANY_CONSTRAINT,never):CbType;
    inline static function get_ANY_CONSTRAINT():CbType{
        return ZPP_CbType.ANY_CONSTRAINT;
    }
    /**
     * Default CbType given to all Shapes
     *
     * Due to the way the Callback system in Nape works, you can use this
     * CbType to match against 'all'
     * Shapes
     * In a Listener (Assuming you do not 'remove' this type from the object)
     */
    #if nape_swc@:isVar #end
    public static var ANY_SHAPE(get_ANY_SHAPE,never):CbType;
    inline static function get_ANY_SHAPE():CbType{
        return ZPP_CbType.ANY_SHAPE;
    }
    /**
     * Default CbType given to all Compounds
     *
     * Due to the way the Callback system in Nape works, you can use this
     * CbType to match against 'all'
     * Compounds
     * In a Listener (Assuming you do not 'remove' this type from the object)
     */
    #if nape_swc@:isVar #end
    public static var ANY_COMPOUND(get_ANY_COMPOUND,never):CbType;
    inline static function get_ANY_COMPOUND():CbType{
        return ZPP_CbType.ANY_COMPOUND;
    }
    /**
     * Construct OptionType with given extra includes.
     * <br/><br/>
     * Equivalent to <code>new OptionType(this).including(includes)</code>
     * <br/><br/>
     * The includes argument is typed Dynamic, and is permitted to be one of:
     * <code>CbType, CbTypeList, Array&lt;CbType&gt;, flash.Vector&lt;CbType&gt;</code>
     *
     * @param includes The CbTypes to include.
     * @return A new OptionType whose includes are equal to this CbType
     *         and all the CbTypes given as argument.
     * @throws # If includes is null.
     */
    public function including(includes:Dynamic):OptionType{
        return(new OptionType(this)).including(includes);
    }
    /**
     * Construct OptionType with given excludes.
     * <br/><br/>
     * Equivalent to <code>new OptionType(this).excluding(excludes)</code>
     * <br/><br/>
     * The excludes argument is typed Dynamic, and is permitted to be one of:
     * <code>CbType, CbTypeList, Array&lt;CbType&gt;, flash.Vector&lt;CbType&gt;</code>
     *
     * @param excludes The CbTypes to exclude.
     * @return A new OptionType whose included types are just 'this' and whose
     *         excluded types are those given as argument.
     * @throws # If excludes is null.
     */
    public function excluding(excludes:Dynamic):OptionType{
        return(new OptionType(this)).excluding(excludes);
    }
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
     * List of all Interactors using this CbType.
     * <br/><br/>
     * This list contains only those Interactors that are inside of a Space
     * <br/><br/>
     * This list is not only readonly, but also immutable.
     */
    #if nape_swc@:isVar #end
    public var interactors(get_interactors,never):InteractorList;
    inline function get_interactors():InteractorList{
        if(zpp_inner.wrap_interactors==null){
            zpp_inner.wrap_interactors=ZPP_InteractorList.get(zpp_inner.interactors,true);
        }
        return zpp_inner.wrap_interactors;
    }
    /**
     * List of all Constraints using this CbType.
     * <br/><br/>
     * This list contains only those Constraints that are inside of a Space
     * <br/><br/>
     * This list is not only readonly, but also immutable.
     */
    #if nape_swc@:isVar #end
    public var constraints(get_constraints,never):ConstraintList;
    inline function get_constraints():ConstraintList{
        if(zpp_inner.wrap_constraints==null){
            zpp_inner.wrap_constraints=ZPP_ConstraintList.get(zpp_inner.constraints,true);
        }
        return zpp_inner.wrap_constraints;
    }
    /**
     * @private
     */
    @:keep public function toString():String{
        return if(this==ANY_BODY)"ANY_BODY";
        else if(this==ANY_SHAPE)"ANY_SHAPE";
        else if(this==ANY_COMPOUND)"ANY_COMPOUND";
        else if(this==ANY_CONSTRAINT)"ANY_CONSTRAINT";
        else "CbType#"+id;
    }
}

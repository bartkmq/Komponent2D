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
import nape.dynamics.FluidArbiter;
import nape.dynamics.ContactList;
import nape.dynamics.ArbiterType;
import nape.dynamics.CollisionArbiter;
import nape.util.Debug;
import nape.util.BitmapDebug;
import nape.util.ShapeDebug;
/**
 * OptionType representing matching behaviour for Listeners.
 * <br/><br/>
 * An object's set of CbType's 'matches' against an OptionType iff.
 * the OptionType's includes list intersects the object's set of CbTypes
 * and the OptionType's excludes list 'does not' intersect the object's set
 * of CbTypes.
 * <pre>
 * option = new OptionType([A, B], [C, D]);
 * obj.cbTypes = [] // => does not match option.
 * obj.cbTypes = [A] // => matches the option
 * obj.cbTypes = [A, C] // => does not match option.
 * </pre>
 * The optionType's includes and excludes list are managed to be always
 * disjoint: The action of including an already excluded type serves to
 * remove it from the excludes list, equalliy excluding an already included
 * type serves to remove it from the includes list.
 * <pre>
 * var option = new OptionType();
 * option.including(A); // option = {[A]:[]}
 * option.excluding(A); // option = {[]:[]}
 * option.excluding(A); // option = {[]:[A]}
 * option.including(A); // option = {[A]:[]}
 * </pre>
 */
@:final#if nape_swc@:keep #end
class OptionType{
    /**
     * @private
     */
    public var zpp_inner:ZPP_OptionType=null;
    /**
     * List of included CbTypes.
     * <br/><br/>
     * This list is both readonly, and immutable. To remove an element
     * from this list you can use: <code>option.excluding(cbType)</code>
     *
     * @default []
     */
    #if nape_swc@:isVar #end
    public var includes(get_includes,never):CbTypeList;
    inline function get_includes():CbTypeList{
        if(zpp_inner.wrap_includes==null)zpp_inner.setup_includes();
        return zpp_inner.wrap_includes;
    }
    /**
     * List of excluded CbTypes.
     * <br/><br/>
     * This list is both readonly, and immutable. To remove an element
     * from this list you can use: <code>option.including(cbType)</code>
     *
     * @default []
     */
    #if nape_swc@:isVar #end
    public var excludes(get_excludes,never):CbTypeList;
    inline function get_excludes():CbTypeList{
        if(zpp_inner.wrap_excludes==null)zpp_inner.setup_excludes();
        return zpp_inner.wrap_excludes;
    }
    /**
     * Construct a new OptionType.
     * <br/><br/>
     * The type of the arguments is Dynamic, and is permitted to be one of:
     * <code>CbType, CbTypeList, Array&lt;CbType&gt; flash.Vector&lt;CbType&gt;</code>
     *
     * @param includes The set of CbTypes to be included in the matching process.
     *                 (default null)
     * @param excludes The set of CbTypes to be excluded in the matching process.
     *                 (default null)
     * @return Return new OptionType with give sets of CbTypes.
     * @throws # If either argument is not of the expected Type.
     */
    #if flib@:keep function flibopts_2(){}
    #end
    public function new(includes:Dynamic=null,excludes:Dynamic=null){
        zpp_inner=new ZPP_OptionType();
        zpp_inner.outer=this;
        if(includes!=null)including(includes);
        if(excludes!=null)excluding(excludes);
    }
    /**
     * Append set of types to includes list.
     * <br/><br/>
     * This method was originally named the more appropriate 'include'
     * but this conflicted with the AS3 keyword include and had to be
     * change.
     * <br/><br/>
     * The argument is typed Dynamic, and is permitted to be one of:
     * <code>CbType, CbTypeList, Array&lt;CbType&gt; flash.Vector&lt;CbType&gt;</code>
     *
     * @param includes The set of CbTypes to be included. (default null)
     * @return A reference to this OptionType.
     * @throws # If argument is not of the expected Type.
     */
    public function including(includes:Dynamic=null):OptionType{
        zpp_inner.append(zpp_inner.includes,includes);
        return this;
    }
    /**
     * Append set of types to excludes list.
     * <br/><br/>
     * This method was originally named the more appropriate 'exclude'
     * but to match the necessary change for the include function, this was
     * renamed as excluding.
     * <br/><br/>
     * The argument is typed Dynamic, and is permitted to be one of:
     * <code>CbType, CbTypeList, Array&lt;CbType&gt; flash.Vector&lt;CbType&gt;</code>
     *
     * @param excludes The set of CbTypes to be excluded. (default null)
     * @return A reference to this OptionType.
     * @throws # If argument is not of the expected Type.
     */
    public function excluding(excludes:Dynamic=null):OptionType{
        zpp_inner.append(zpp_inner.excludes,excludes);
        return this;
    }
    /**
     * @private
     */
    @:keep public function toString():String{
        var inc=includes.toString();
        var exc=excludes.toString();
        return "@{"+inc+" excluding "+exc+"}";
    }
}

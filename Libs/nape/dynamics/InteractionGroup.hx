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
 * InteractionGroups are another way of filtering interactions.
 * <br/><br/>
 * InteractionGroups form tree structures which are checked along side InteractionFilters
 * when deciding if two Shapes should interact.
 * <br/><br/>
 * InteractionGroups are assigned to any Interactor (not just Shapes), and two Shapes will
 * interact only if the most recent common ancestor in the InteractionGroup tree permits it.
 * <br/><br/>
 * For the purposes of the search, if any Interactor has no InteractionGroup assigned, we
 * search up the Compound tree first.
 * <pre>
 *            _Group1
 *           /   |
 *          /  Group2      Group3
 *         /    |    \       |                 Group1
 *     Body1   /      Cmp1   |                 /   \           Group3
 *    /    \  /      /    \  |      ==>    Shp1    Group2        |
 * Shp1   Shp2   Body2     Cmp2                    /    \      Shp4
 *                 |         |                  Shp2    Shp3
 *                Shp3     Body3
 *                           |
 *                         Shp4
 * </pre>
 * If we look at which InteractionGroup is used for which Shape following this rule, then
 * the left graph can be transformed into an InteractionGroup tree on the right and we get
 * that the MRCA (Most recent common ancestors) are such that:
 * <pre>
 * MRCA(Shp1, Shp3) == Group1;
 * MRCA(Shp2, Shp3) == Group2;
 * MRCA(Shp4,   # ) == null;
 * </pre>
 * If we were to set up the groups such that <code>Group1.ignore = false</code> and
 * <code>Group2.ignore = true</code>; then shapes 1 and 3 would not be permitted to
 * interact, whilst shapes 2 and 3 would be permitted.
 * <br/>
 * As the MRCA for shape 4 with any other is null, then the value of Group3's ignore field
 * is irrelevant, but the existance of Group3 is not as it serves to otherwise prevent Shape 4
 * from being permitted to interact with shapes 2 and 3.
 * <br/><br/>
 * InteractionGroup's can be fairly expressive, but they are strictly less powerful than
 * InteractionFilters. InteractionGroup's have their place however as there is no limit
 * to the number of Groups you can use.
 */
@:final#if nape_swc@:keep #end
class InteractionGroup{
    /**
     * @private
     */
    public var zpp_inner:ZPP_InteractionGroup=null;
    /**
     * Parent group in InteractionGroup tree.
     * @default null
     */
    #if nape_swc@:isVar #end
    public var group(get_group,set_group):Null<InteractionGroup>;
    inline function get_group():Null<InteractionGroup>{
        return if(zpp_inner.group==null)null else zpp_inner.group.outer;
    }
    inline function set_group(group:Null<InteractionGroup>):Null<InteractionGroup>{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(group==this)throw "Error: Cannot assign InteractionGroup to itself";
            #end
            zpp_inner.setGroup(group==null?null:group.zpp_inner);
        }
        return get_group();
    }
    /**
     * Ignore property, set to true so that objects will not interact.
     * @default false
     */
    #if nape_swc@:isVar #end
    public var ignore(get_ignore,set_ignore):Bool;
    inline function get_ignore():Bool{
        return zpp_inner.ignore;
    }
    inline function set_ignore(ignore:Bool):Bool{
        {
            if(this.ignore!=ignore){
                zpp_inner.invalidate(true);
                zpp_inner.ignore=ignore;
            }
        }
        return get_ignore();
    }
    /**
     * Set of active interactors using this group.
     * <br/><br/>
     * Active interactors meaning those that are part of a Space.
     * <br/><br/>
     * This list is immutable.
     */
    #if nape_swc@:isVar #end
    public var interactors(get_interactors,never):InteractorList;
    inline function get_interactors():InteractorList{
        if(zpp_inner.wrap_interactors==null)zpp_inner.wrap_interactors=ZPP_InteractorList.get(zpp_inner.interactors,true);
        return zpp_inner.wrap_interactors;
    }
    /**
     * Immutable set of children of Interaction groups.
     * <br/><br/>
     * You cannot assign or remove children in this manner, you must do it via setting
     * the childs parent group to this/null.
     * <br/><br/>
     * This list is immutable.
     */
    #if nape_swc@:isVar #end
    public var groups(get_groups,never):InteractionGroupList;
    inline function get_groups():InteractionGroupList{
        if(zpp_inner.wrap_groups==null)zpp_inner.wrap_groups=ZPP_InteractionGroupList.get(zpp_inner.groups,true);
        return zpp_inner.wrap_groups;
    }
    /**
     * Construct a new InteractionGroup.
     *
     * @param ignore Whether interactors should be ignored. (default false)
     */
    public function new(ignore=false){
        zpp_inner=new ZPP_InteractionGroup();
        zpp_inner.outer=this;
        this.ignore=ignore;
    }
    /**
     * @private
     */
    public function toString(){
        var ret="InteractionGroup";
        if(ignore)ret+=":ignore";
        return ret;
    }
}

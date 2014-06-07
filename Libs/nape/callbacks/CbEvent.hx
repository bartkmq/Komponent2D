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
import nape.callbacks.CbType;
import nape.callbacks.ListenerType;
import nape.callbacks.PreFlag;
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
 * Enumeration of possible callback event types.
 */
@:final#if nape_swc@:keep #end
class CbEvent{
    /**
     * @private
     */
    public function new(){
        #if(!NAPE_RELEASE_BUILD)
        if(!ZPP_Flags.internal)throw "Error: Cannot instantiate "+"CbEvent"+" derp!";
        #end
    }
    /**
     * @private
     */
    @:keep public function toString(){
        if(false)return "";
        
        else if(this==PRE)return"PRE";
        else if(this==BEGIN)return"BEGIN";
        else if(this==ONGOING)return"ONGOING";
        else if(this==END)return"END";
        else if(this==WAKE)return"WAKE";
        else if(this==SLEEP)return"SLEEP";
        else if(this==BREAK)return"BREAK";
        else return "";
    }
    /**
     * BEGIN event corresponds to the start of an interaction
     */
    #if nape_swc@:isVar #end
    public static var BEGIN(get_BEGIN,never):CbEvent;
    inline static function get_BEGIN(){
        if(ZPP_Flags.CbEvent_BEGIN==null){
            ZPP_Flags.internal=true;
            ZPP_Flags.CbEvent_BEGIN=new CbEvent();
            ZPP_Flags.internal=false;
        }
        return ZPP_Flags.CbEvent_BEGIN;
    }
    /**
     * ONGOING event corresponds to any step in which an interaction is occuring
     * overlapping with the BEGIN event.
     */
    #if nape_swc@:isVar #end
    public static var ONGOING(get_ONGOING,never):CbEvent;
    inline static function get_ONGOING(){
        if(ZPP_Flags.CbEvent_ONGOING==null){
            ZPP_Flags.internal=true;
            ZPP_Flags.CbEvent_ONGOING=new CbEvent();
            ZPP_Flags.internal=false;
        }
        return ZPP_Flags.CbEvent_ONGOING;
    }
    /**
     * END event corresponds to the end of an interaction.
     */
    #if nape_swc@:isVar #end
    public static var END(get_END,never):CbEvent;
    inline static function get_END(){
        if(ZPP_Flags.CbEvent_END==null){
            ZPP_Flags.internal=true;
            ZPP_Flags.CbEvent_END=new CbEvent();
            ZPP_Flags.internal=false;
        }
        return ZPP_Flags.CbEvent_END;
    }
    /**
     * WAKE event corresponds to the waking of a Body or Constraint in the space.
     */
    #if nape_swc@:isVar #end
    public static var WAKE(get_WAKE,never):CbEvent;
    inline static function get_WAKE(){
        if(ZPP_Flags.CbEvent_WAKE==null){
            ZPP_Flags.internal=true;
            ZPP_Flags.CbEvent_WAKE=new CbEvent();
            ZPP_Flags.internal=false;
        }
        return ZPP_Flags.CbEvent_WAKE;
    }
    /**
     * SLEEP event corresponds to the sleeping of a Body or Constraint in the space.
     */
    #if nape_swc@:isVar #end
    public static var SLEEP(get_SLEEP,never):CbEvent;
    inline static function get_SLEEP(){
        if(ZPP_Flags.CbEvent_SLEEP==null){
            ZPP_Flags.internal=true;
            ZPP_Flags.CbEvent_SLEEP=new CbEvent();
            ZPP_Flags.internal=false;
        }
        return ZPP_Flags.CbEvent_SLEEP;
    }
    /**
     * BREAK event corresponds to the breaking of a defined limit on a Constraint.
     */
    #if nape_swc@:isVar #end
    public static var BREAK(get_BREAK,never):CbEvent;
    inline static function get_BREAK(){
        if(ZPP_Flags.CbEvent_BREAK==null){
            ZPP_Flags.internal=true;
            ZPP_Flags.CbEvent_BREAK=new CbEvent();
            ZPP_Flags.internal=false;
        }
        return ZPP_Flags.CbEvent_BREAK;
    }
    /**
     * PRE event corresponds to a special mid-step event that occurs after it is determined
     * that two objects 'will' begin to interact, but before any interaction commences.
     */
    #if nape_swc@:isVar #end
    public static var PRE(get_PRE,never):CbEvent;
    inline static function get_PRE(){
        if(ZPP_Flags.CbEvent_PRE==null){
            ZPP_Flags.internal=true;
            ZPP_Flags.CbEvent_PRE=new CbEvent();
            ZPP_Flags.internal=false;
        }
        return ZPP_Flags.CbEvent_PRE;
    }
}

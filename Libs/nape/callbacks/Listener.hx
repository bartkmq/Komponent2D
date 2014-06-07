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
 * Base type for all Nape callback listeners.
 */
#if nape_swc@:keep #end
class Listener{
    /**
     * @private
     */
    public var zpp_inner:ZPP_Listener=null;
    /**
     * The sub-type of this listener.
     */
    #if nape_swc@:isVar #end
    public var type(get_type,never):ListenerType;
    inline function get_type():ListenerType{
        return ZPP_Listener.types[zpp_inner.type];
    }
    /**
     * The CbEvent this listener responds to.
     */
    #if nape_swc@:isVar #end
    public var event(get_event,set_event):CbEvent;
    inline function get_event():CbEvent{
        return ZPP_Listener.events[zpp_inner.event];
    }
    inline function set_event(event:CbEvent):CbEvent{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(event==null){
                throw "Error: Cannot set listener event type to null";
            }
            #end
            if(this.event!=event){
                var xevent=if(event==CbEvent.BEGIN)ZPP_Flags.id_CbEvent_BEGIN else if(event==CbEvent.ONGOING)ZPP_Flags.id_CbEvent_ONGOING else if(event==CbEvent.END)ZPP_Flags.id_CbEvent_END else if(event==CbEvent.SLEEP)ZPP_Flags.id_CbEvent_SLEEP else if(event==CbEvent.WAKE)ZPP_Flags.id_CbEvent_WAKE else if(event==CbEvent.PRE)ZPP_Flags.id_CbEvent_PRE else ZPP_Flags.id_CbEvent_BREAK;
                zpp_inner.swapEvent(xevent);
            }
        }
        return get_event();
    }
    /**
     * The precedence of this listener.
     * <br/><br/>
     * In any case that there is more than one suitable listener for a situation,
     * the listeners will be ordered by their precedence.
     *
     * @default 0
     */
    #if nape_swc@:isVar #end
    public var precedence(get_precedence,set_precedence):Int;
    inline function get_precedence():Int{
        return zpp_inner.precedence;
    }
    inline function set_precedence(precedence:Int):Int{
        {
            if(this.precedence!=precedence){
                zpp_inner.precedence=precedence;
                zpp_inner.invalidate_precedence();
            }
        }
        return get_precedence();
    }
    /**
     * @private
     */
    public function new(){
        #if(!NAPE_RELEASE_BUILD)
        if(!ZPP_Listener.internal){
            throw "Error: Cannot instantiate Listener derp!";
        }
        #end
    }
    /**
     * The Space this listener is assigned to.
     * <br/><br/>
     * This value can be set, with setting to null being equivalent to removing
     * the listener from whichever Space it is presently assigned to.
     *
     * @default null
     */
    #if nape_swc@:isVar #end
    public var space(get_space,set_space):Null<Space>;
    inline function get_space():Null<Space>{
        return if(zpp_inner.space==null)null else zpp_inner.space.outer;
    }
    inline function set_space(space:Null<Space>):Null<Space>{
        {
            if(this.space!=space){
                if(zpp_inner.space!=null){
                    zpp_inner.space.outer.listeners.remove(this);
                }
                if(space!=null){
                    space.listeners.add(this);
                }
                else{
                    zpp_inner.space=null;
                }
            }
        }
        return get_space();
    }
    /**
     * @private
     */
    @:keep public function toString(){
        var event=["BEGIN","END","WAKE","SLEEP","BREAK","PRE","ONGOING"][zpp_inner.event];
        if(zpp_inner.type==ZPP_Flags.id_ListenerType_BODY){
            var body=zpp_inner.body;
            return "BodyListener{"+event+"::"+body.outer_zn.options+"}";
        }
        else if(zpp_inner.type==ZPP_Flags.id_ListenerType_CONSTRAINT){
            var con=zpp_inner.constraint;
            return "ConstraintListener{"+event+"::"+con.outer_zn.options+"}";
        }
        else{
            var con=zpp_inner.interaction;
            var itype=switch(con.itype){
                case ZPP_Flags.id_InteractionType_COLLISION:"COLLISION";
                case ZPP_Flags.id_InteractionType_SENSOR:"SENSOR";
                case ZPP_Flags.id_InteractionType_FLUID:"FLUID";
                default:"ALL";
            }
            return(if(zpp_inner.type==ZPP_Flags.id_ListenerType_INTERACTION)"InteractionListener{"+event+"#"+itype+"::"+con.outer_zni.options1+":"+con.outer_zni.options2+"}" else "PreListener{"+itype+"::"+con.outer_znp.options1+":"+con.outer_znp.options2+"}")+" precedence="+zpp_inner.precedence;
        }
    }
}

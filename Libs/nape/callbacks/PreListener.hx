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
 * Event listener for Pre-Interaction type events.
 * <br/><br/>
 * Pre-Interaction type events can occur between any two Interactors (whether they
 * be Shapes, Bodys, Compounds or a mix thereof).
 */
@:final#if nape_swc@:keep #end
class PreListener extends Listener{
    /**
     * @private
     */
    public var zpp_inner_zn:ZPP_InteractionListener=null;
    /**
     * The OptionType used to match against Interactors for the first object.
     */
    #if nape_swc@:isVar #end
    public var options1(get_options1,set_options1):OptionType;
    inline function get_options1():OptionType{
        return zpp_inner_zn.options1.outer;
    }
    inline function set_options1(options1:OptionType):OptionType{
        {
            zpp_inner_zn.options1.set(options1.zpp_inner);
        }
        return get_options1();
    }
    /**
     * The OptionType used to match against Interactors for the second object.
     */
    #if nape_swc@:isVar #end
    public var options2(get_options2,set_options2):OptionType;
    inline function get_options2():OptionType{
        return zpp_inner_zn.options2.outer;
    }
    inline function set_options2(options2:OptionType):OptionType{
        {
            zpp_inner_zn.options2.set(options2.zpp_inner);
        }
        return get_options2();
    }
    /**
     * Callback handler for this listener.
     * <br/><br/>
     * This callback handler returns a possibly null PreFlag object.
     * <br/>
     * Passing null is equivalent to telling nape 'ignore me' so that whatever existing
     * decision has been made regarding the interaction is not modified.
     * Otherwise returning a non-null PreFlag will change the current decision about what
     * to do with the interaction.
     * <br/><br/>
     * Returning ACCEPT/IGNORE inform nape to take control over all subsequent interaction
     * between the two objects until they seperate. Returning these will mean that the pre
     * listener will not be invoked again until the objects seperate, and then begin to interact
     * afresh.
     * <br/><br/>
     * Returning #_ONCE, the objects will only be effected for a single step, and the following
     * step should they still be candidates for interaction, this handler will be invoked again.
     * <br/>
     * In the case of a #_ONCE, PreFlag; Nape will 'not' permit the two objects to go to sleep
     * as Nape cannot know if this callback handler will suddenly changes its mind.
     * <br/>
     * If this handler is a 'pure' function, then you may mark it as such and Nape will keep you
     * to your word and permit the objects to sleep.
     */
    #if nape_swc@:isVar #end
    public var handler(get_handler,set_handler):PreCallback->Null<PreFlag>;
    inline function get_handler():PreCallback->Null<PreFlag>{
        return zpp_inner_zn.handlerp;
    }
    inline function set_handler(handler:PreCallback->Null<PreFlag>):PreCallback->Null<PreFlag>{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(handler==null){
                throw "Error: PreListener must take a non-null handler!";
            }
            #end
            zpp_inner_zn.handlerp=handler;
            zpp_inner_zn.wake();
        }
        return get_handler();
    }
    /**
     * Mark this listener as having a pure callback handler.
     * <br/><br/>
     * A pure callback handler is one which under no circumstances will change its behaviour.
     * In such a (favourable) instance, marking the callback as pure will allow Nape to permit
     * objects in interaction to go to sleep even if the handler returns an IGNORE_ONCE/ACCEPT_ONCE
     * PreFlag.
     *
     * @default false
     */
    #if nape_swc@:isVar #end
    public var pure(get_pure,set_pure):Bool;
    inline function get_pure():Bool{
        return zpp_inner_zn.pure;
    }
    inline function set_pure(pure:Bool):Bool{
        {
            if(!pure){
                zpp_inner_zn.wake();
            }
            zpp_inner_zn.pure=pure;
        }
        return get_pure();
    }
    /**
     * The specific type of interaction that is to be listened for.
     * <br/><br/>
     * If we specify that we only want to listen for a fluid type interaction, then
     * this listener will operate so that any other interactions for the same pair
     * of objects is ignored.
     */
    #if nape_swc@:isVar #end
    public var interactionType(get_interactionType,set_interactionType):InteractionType;
    inline function get_interactionType():InteractionType{
        var ret=zpp_inner_zn.itype;
        return if(ret==ZPP_Flags.id_InteractionType_COLLISION)InteractionType.COLLISION;
        else if(ret==ZPP_Flags.id_InteractionType_SENSOR)InteractionType.SENSOR;
        else if(ret==ZPP_Flags.id_InteractionType_FLUID)InteractionType.FLUID;
        else if(ret==ZPP_Flags.id_InteractionType_ANY)InteractionType.ANY;
        else null;
    }
    inline function set_interactionType(interactionType:InteractionType):InteractionType{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(interactionType==null){
                throw "Error: Cannot set listener interaction type to null";
            }
            #end
            if(this.interactionType!=interactionType){
                var xtype=if(interactionType==InteractionType.COLLISION)ZPP_Flags.id_InteractionType_COLLISION else if(interactionType==InteractionType.SENSOR)ZPP_Flags.id_InteractionType_SENSOR else if(interactionType==InteractionType.FLUID)ZPP_Flags.id_InteractionType_FLUID else ZPP_Flags.id_InteractionType_ANY;
                zpp_inner_zn.setInteractionType(xtype);
            }
        }
        return get_interactionType();
    }
    /**
     * Construct a new PreListener.
     * <br/><br/>
     * The options arguments are typed Dynamic, and are permitted to be either an
     * <code>OptionType</code> or one of:
     * <code>CbType, CbTypeList, Array&lt;CbType&gt;, flash.Vector&lt;CbType&gt;</code>
     * In which case the input CbType's will be used to construct an OptionType
     * whose included types will be the set of CbTypes supplied.
     *
     * @param interactionType The interaction type to listen for.
     * @param options1 The OptionType to match first Interactor against, passing null
     *                will equate to an empty OptionType.
     * @param options2 The OptionType to match second Interactor against, passing null
     *                will equate to an empty OptionType.
     * @param handler The callback handler for this listener.
     * @param precedence The precedence of this listener used to sort
     *                   the order of callbacks in the case of more than
     *                   one suitable BodyListener existing for the same
     *                   event on the same Body. (default 0)
     * @param pure If true, then the listener will be marked as having a pure handler.
     *             (default false)
     * @return The newly constructed InteractionListener
     * @throws # If handler is null.
     * @throws # If either option is not of the expected Type.
     */
    #if flib@:keep function flibopts_2(){}
    #end
    public function new(interactionType:InteractionType,options1:Null<Dynamic>,options2:Null<Dynamic>,handler:PreCallback->Null<PreFlag>,precedence=0,pure=false){
        #if(!NAPE_RELEASE_BUILD)
        ZPP_Listener.internal=true;
        #end
        super();
        #if(!NAPE_RELEASE_BUILD)
        ZPP_Listener.internal=false;
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(handler==null){
            throw "Error: PreListener must take a handler!";
        }
        #end
        zpp_inner_zn=new ZPP_InteractionListener(ZPP_OptionType.argument(options1),ZPP_OptionType.argument(options2),ZPP_Flags.id_CbEvent_PRE,ZPP_Flags.id_ListenerType_PRE);
        zpp_inner=zpp_inner_zn;
        zpp_inner.outer=this;
        zpp_inner_zn.outer_znp=this;
        zpp_inner.precedence=precedence;
        zpp_inner_zn.pure=pure;
        zpp_inner_zn.handlerp=handler;
        this.interactionType=interactionType;
    }
}

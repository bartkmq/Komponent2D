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
import nape.dynamics.FluidArbiter;
import nape.dynamics.ContactList;
import nape.dynamics.ArbiterType;
import nape.dynamics.CollisionArbiter;
import nape.util.Debug;
import nape.util.BitmapDebug;
import nape.util.ShapeDebug;
/**
 * Event listener for Interaction type events.
 * <br/><br/>
 * Interaction type events can occur between any two Interactors (whether they
 * be Shapes, Bodys, Compounds or a mix thereof).
 * <br/><br/>
 * The events that can be caught are BEGIN, ONGOING, and END type events.
 * Theses listeners will operate between pairs of Interactors.
 * <pre>
 *          _Space
 *         /      \
 *     Cmp1        Cmp3
 *    /    \         |
 * Body1  Cmp2     Body3
 *   |      |        |
 * Shp1   Body2    Shp3
 *          |
 *        Shp2
 * </pre>
 * The possible interactor pairs for callbacks are formed by finding the most
 * recent common ancestor in the world for the given pair of shapes and taking all
 * possible pairings. In the above situation we have:
 * <pre>
 * MRCA(Shp1, Shp2) = Cmp1  --> Possible pairings = [Shp1, Body1] x [Shp2, Body2, Cmp2]
 * MRCA(Shp1, Shp3) = Space --> Possible pairings = [Shp1, Body1, Cmp1] x [Shp3, Body3, Cmp3]
 * MRCA(Shp2, Shp3) = Space --> Possible pairings = [Shp2, Body2, Cmp2, Cmp1] x [Shp3, Body3, Cmp3]
 * </pre>
 * Of course, not all of these pairings will generate callbacks, only those for which
 * a valid listener exists for the event type, and for the cbtypes of each interactor.
 * <br/><br/>
 * Furthermore, the listener specifies an interaction type which works even in mixed
 * cases where many types of interaction between two objects is happening at once.
 */
@:final#if nape_swc@:keep #end
class InteractionListener extends Listener{
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
     * The callback handler for this listener.
     */
    #if nape_swc@:isVar #end
    public var handler(get_handler,set_handler):InteractionCallback->Void;
    inline function get_handler():InteractionCallback->Void{
        return zpp_inner_zn.handleri;
    }
    inline function set_handler(handler:InteractionCallback->Void):InteractionCallback->Void{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(handler==null){
                throw "Error: InteractionListener::handler cannot be null";
            }
            #end
            zpp_inner_zn.handleri=handler;
        }
        return get_handler();
    }
    /**
     * For ONGOING listeners only, permit ONGOING callbacks whilst sleeping.
     * <br/><br/>
     * This property determines whether we will still receive
     * ONGOING callbacks between two sleeping Interactors. The default action is to
     * inhibit callbacks between sleeping objects for performance. Setting this field to true
     * will permit Nape to always generate callbacks.
     */
    #if nape_swc@:isVar #end
    public var allowSleepingCallbacks(get_allowSleepingCallbacks,set_allowSleepingCallbacks):Bool;
    inline function get_allowSleepingCallbacks():Bool{
        return zpp_inner_zn.allowSleepingCallbacks;
    }
    inline function set_allowSleepingCallbacks(allowSleepingCallbacks:Bool):Bool{
        {
            zpp_inner_zn.allowSleepingCallbacks=allowSleepingCallbacks;
        }
        return get_allowSleepingCallbacks();
    }
    /**
     * Construct a new InteractionListener.
     * <br/><br/>
     * The possible event types are BEGIN, ONGOING and END.
     * <br/><br/>
     * The options arguments are typed Dynamic, and are permitted to be either an
     * <code>OptionType</code> or one of:
     * <code>CbType, CbTypeList, Array&lt;CbType&gt;, flash.Vector&lt;CbType&gt;</code>
     * In which case the input CbType's will be used to construct an OptionType
     * whose included types will be the set of CbTypes supplied.
     *
     * @param event The event type to listen for.
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
     * @return The newly constructed InteractionListener
     * @throws # If handler is null.
     * @throws # If the event type is not permitted for this listener.
     * @throws # If either option is not of the expected Type.
     */
    #if flib@:keep function flibopts_1(){}
    #end
    public function new(event:CbEvent,interactionType:InteractionType,options1:Null<Dynamic>,options2:Null<Dynamic>,handler:InteractionCallback->Void,precedence:Int=0){
        #if(!NAPE_RELEASE_BUILD)
        ZPP_Listener.internal=true;
        #end
        super();
        #if(!NAPE_RELEASE_BUILD)
        ZPP_Listener.internal=false;
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(handler==null){
            throw "Error: InteractionListener::handler cannot be null";
        }
        if(event==null){
            throw "Error: CbEvent cannot be null for InteractionListener";
        }
        #end
        var xevent=-1;
        if(event==CbEvent.BEGIN)xevent=ZPP_Flags.id_CbEvent_BEGIN;
        else if(event==CbEvent.END)xevent=ZPP_Flags.id_CbEvent_END;
        else if(event==CbEvent.ONGOING)xevent=ZPP_Flags.id_CbEvent_ONGOING;
        else{
            #if(!NAPE_RELEASE_BUILD)
            throw "Error: CbEvent '"+event.toString()+"' is not a valid event type for InteractionListener";
            #end
        }
        zpp_inner_zn=new ZPP_InteractionListener(ZPP_OptionType.argument(options1),ZPP_OptionType.argument(options2),xevent,ZPP_Flags.id_ListenerType_INTERACTION);
        zpp_inner=zpp_inner_zn;
        zpp_inner.outer=this;
        zpp_inner_zn.outer_zni=this;
        zpp_inner.precedence=precedence;
        zpp_inner_zn.handleri=handler;
        this.interactionType=interactionType;
    }
}

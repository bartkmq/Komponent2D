package zpp_nape.callbacks;
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
#if nape_swc@:keep #end
class ZPP_OptionType{
    public var outer:OptionType=null;
    public var handler:Null<ZPP_CbType->Bool->Bool->Void>=null;
    public var includes:ZNPList_ZPP_CbType=null;
    public var excludes:ZNPList_ZPP_CbType=null;
    public function new(){
        includes=new ZNPList_ZPP_CbType();
        excludes=new ZNPList_ZPP_CbType();
    }
    public var wrap_includes:CbTypeList=null;
    public var wrap_excludes:CbTypeList=null;
    public function setup_includes():Void{
        wrap_includes=ZPP_CbTypeList.get(includes,true);
    }
    public function setup_excludes():Void{
        wrap_excludes=ZPP_CbTypeList.get(excludes,true);
    }
    public#if NAPE_NO_INLINE#else inline #end
    function excluded(xs:ZNPList_ZPP_CbType):Bool{
        return nonemptyintersection(xs,excludes);
    }
    public#if NAPE_NO_INLINE#else inline #end
    function included(xs:ZNPList_ZPP_CbType):Bool{
        return nonemptyintersection(xs,includes);
    }
    public#if NAPE_NO_INLINE#else inline #end
    function compatible(xs:ZNPList_ZPP_CbType):Bool{
        return included(xs)&&!excluded(xs);
    }
    public function nonemptyintersection(xs:ZNPList_ZPP_CbType,ys:ZNPList_ZPP_CbType):Bool{
        var ret=false;
        var xite=xs.begin();
        var eite=ys.begin();
        while(eite!=null&&xite!=null){
            var ex=eite.elem();
            var xi=xite.elem();
            if(ex==xi){
                ret=true;
                break;
            }
            else if(ZPP_CbType.setlt(ex,xi)){
                eite=eite.next;
            }
            else{
                xite=xite.next;
            }
        }
        return ret;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function effect_change(val:ZPP_CbType,included:Bool,added:Bool):Void{
        if(included){
            if(added){
                var pre=null;
                {
                    var cx_ite=includes.begin();
                    while(cx_ite!=null){
                        var j=cx_ite.elem();
                        {
                            if(ZPP_CbType.setlt(val,j))break;
                            pre=cx_ite;
                        };
                        cx_ite=cx_ite.next;
                    }
                };
                includes.inlined_insert(pre,val);
            };
            else includes.remove(val);
        }
        else{
            if(added){
                var pre=null;
                {
                    var cx_ite=excludes.begin();
                    while(cx_ite!=null){
                        var j=cx_ite.elem();
                        {
                            if(ZPP_CbType.setlt(val,j))break;
                            pre=cx_ite;
                        };
                        cx_ite=cx_ite.next;
                    }
                };
                excludes.inlined_insert(pre,val);
            };
            else excludes.remove(val);
        }
    }
    function append_type(list:ZNPList_ZPP_CbType,val:ZPP_CbType){
        if(list==includes){
            if(!includes.has(val)){
                if(!excludes.has(val)){
                    if(handler!=null)handler(val,true,true);
                    else effect_change(val,true,true);
                }
                else{
                    if(handler!=null)handler(val,false,false);
                    else effect_change(val,false,false);
                }
            }
        }
        else{
            if(!excludes.has(val)){
                if(!includes.has(val)){
                    if(handler!=null)handler(val,false,true);
                    else effect_change(val,false,true);
                }
                else{
                    if(handler!=null)handler(val,true,false);
                    else effect_change(val,true,false);
                }
            }
        }
    }
    public function set(options:ZPP_OptionType){
        if(options!=this){
            while(!includes.empty())append_type(excludes,includes.front());
            while(!excludes.empty())append_type(includes,excludes.front());
            {
                var cx_ite=options.excludes.begin();
                while(cx_ite!=null){
                    var i=cx_ite.elem();
                    append_type(excludes,i);
                    cx_ite=cx_ite.next;
                }
            };
            {
                var cx_ite=options.includes.begin();
                while(cx_ite!=null){
                    var i=cx_ite.elem();
                    append_type(includes,i);
                    cx_ite=cx_ite.next;
                }
            };
        }
        return this;
    }
    public function append(list:ZNPList_ZPP_CbType,val:Dynamic){
        #if(!NAPE_RELEASE_BUILD)
        if(val==null){
            throw "Error: Cannot append null, only CbType and CbType list values";
        }
        #end
        if(#if flash untyped __is__(val,CbType)#else Std.is(val,CbType)#end){
            var cb:CbType=val;
            append_type(list,cb.zpp_inner);
        }
        else if(#if flash untyped __is__(val,CbTypeList)#else Std.is(val,CbTypeList)#end){
            var cbs:CbTypeList=val;
            for(cb in cbs)append_type(list,cb.zpp_inner);
        }
        else if(#if flash10#if flash untyped __is__(val,ZPP_Const.cbtypevector)#else Std.is(val,ZPP_Const.cbtypevector)#end
        #else false #end){
            #if flash10 var cbs:flash.Vector<CbType>=val;
            for(cb in cbs)append_type(list,cb.zpp_inner);
            #end
        }
        else if(#if flash untyped __is__(val,Array)#else Std.is(val,Array)#end){
            var cbs:Array<Dynamic>=val;
            for(cb in cbs){
                #if(!NAPE_RELEASE_BUILD)
                if(!#if flash untyped __is__(cb,CbType)#else Std.is(cb,CbType)#end){
                    throw "Error: Cannot append non-CbType or CbType list value";
                }
                #end
                var cbx:CbType=cb;
                append_type(list,cbx.zpp_inner);
            }
        }
        else{
            #if(!NAPE_RELEASE_BUILD)
            throw "Error: Cannot append non-CbType or CbType list value";
            #end
        }
    }
    public static function argument(val:Dynamic):OptionType{
        return if(val==null)new OptionType();
        else if(#if flash untyped __is__(val,OptionType)#else Std.is(val,OptionType)#end)val;
        else new OptionType().including(val);
    }
}

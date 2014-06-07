package zpp_nape.constraint;
import zpp_nape.Const;
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
class ZPP_PivotJoint extends ZPP_Constraint{
    public var outer_zn:PivotJoint=null;
    public function bodyImpulse(b:ZPP_Body){
        if(stepped){
            if(b==b1)return Vec3.get(-jAccx,-jAccy,-(jAccy*a1relx-jAccx*a1rely));
            else return Vec3.get(jAccx,jAccy,(jAccy*a2relx-jAccx*a2rely));
        }
        else return Vec3.get(0,0,0);
    }
    public override function activeBodies(){
        {
            if(b1!=null)b1.constraints.add(this);
        };
        if(b2!=b1){
            if(b2!=null)b2.constraints.add(this);
        };
    }
    public override function inactiveBodies(){
        {
            if(b1!=null)b1.constraints.remove(this);
        };
        if(b2!=b1){
            if(b2!=null)b2.constraints.remove(this);
        };
    }
    public var b1:ZPP_Body=null;
    public var a1localx:Float=0.0;
    public var a1localy:Float=0.0;
    public var a1relx:Float=0.0;
    public var a1rely:Float=0.0;
    private function validate_a1(){
        {
            wrap_a1.zpp_inner.x=a1localx;
            wrap_a1.zpp_inner.y=a1localy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_a1.zpp_inner.x!=wrap_a1.zpp_inner.x));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_a1.zpp_inner.x)"+") :: "+("vec_set(in n: "+"wrap_a1.zpp_inner."+",in x: "+"a1localx"+",in y: "+"a1localy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_a1.zpp_inner.y!=wrap_a1.zpp_inner.y));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_a1.zpp_inner.y)"+") :: "+("vec_set(in n: "+"wrap_a1.zpp_inner."+",in x: "+"a1localx"+",in y: "+"a1localy"+")");
                #end
            };
        };
    }
    private function invalidate_a1(x:ZPP_Vec2){
        immutable_midstep("Constraint::"+"a1");
        {
            a1localx=x.x;
            a1localy=x.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((a1localx!=a1localx));
                };
                if(!res)throw "assert("+"!assert_isNaN(a1localx)"+") :: "+("vec_set(in n: "+"a1local"+",in x: "+"x.x"+",in y: "+"x.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((a1localy!=a1localy));
                };
                if(!res)throw "assert("+"!assert_isNaN(a1localy)"+") :: "+("vec_set(in n: "+"a1local"+",in x: "+"x.x"+",in y: "+"x.y"+")");
                #end
            };
        };
         wake();
    }
    public function setup_a1(){
        wrap_a1=Vec2.get(a1localx,a1localy);
        wrap_a1.zpp_inner._inuse=true;
        wrap_a1.zpp_inner._validate=validate_a1;
        wrap_a1.zpp_inner._invalidate=invalidate_a1;
    }
    public var wrap_a1:Vec2=null;
    public var b2:ZPP_Body=null;
    public var a2localx:Float=0.0;
    public var a2localy:Float=0.0;
    public var a2relx:Float=0.0;
    public var a2rely:Float=0.0;
    private function validate_a2(){
        {
            wrap_a2.zpp_inner.x=a2localx;
            wrap_a2.zpp_inner.y=a2localy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_a2.zpp_inner.x!=wrap_a2.zpp_inner.x));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_a2.zpp_inner.x)"+") :: "+("vec_set(in n: "+"wrap_a2.zpp_inner."+",in x: "+"a2localx"+",in y: "+"a2localy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_a2.zpp_inner.y!=wrap_a2.zpp_inner.y));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_a2.zpp_inner.y)"+") :: "+("vec_set(in n: "+"wrap_a2.zpp_inner."+",in x: "+"a2localx"+",in y: "+"a2localy"+")");
                #end
            };
        };
    }
    private function invalidate_a2(x:ZPP_Vec2){
        immutable_midstep("Constraint::"+"a2");
        {
            a2localx=x.x;
            a2localy=x.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((a2localx!=a2localx));
                };
                if(!res)throw "assert("+"!assert_isNaN(a2localx)"+") :: "+("vec_set(in n: "+"a2local"+",in x: "+"x.x"+",in y: "+"x.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((a2localy!=a2localy));
                };
                if(!res)throw "assert("+"!assert_isNaN(a2localy)"+") :: "+("vec_set(in n: "+"a2local"+",in x: "+"x.x"+",in y: "+"x.y"+")");
                #end
            };
        };
         wake();
    }
    public function setup_a2(){
        wrap_a2=Vec2.get(a2localx,a2localy);
        wrap_a2.zpp_inner._inuse=true;
        wrap_a2.zpp_inner._validate=validate_a2;
        wrap_a2.zpp_inner._invalidate=invalidate_a2;
    }
    public var wrap_a2:Vec2=null;
    public var kMassa:Float=0.0;
    public var kMassb:Float=0.0;
    public var kMassc:Float=0.0;
    public var jAccx:Float=0.0;
    public var jAccy:Float=0.0;
    public var jMax:Float=0.0;
    public var gamma:Float=0.0;
    public var biasx:Float=0.0;
    public var biasy:Float=0.0;
    public var stepped:Bool=false;
    public override function copy(dict:Array<ZPP_CopyHelper>=null,todo:Array<ZPP_CopyHelper>=null):Constraint{
        var ret=new PivotJoint(null,null,outer_zn.anchor1,outer_zn.anchor2);
        copyto(ret);
        {
            if(dict!=null&&b1!=null){
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        todo!=null;
                    };
                    if(!res)throw "assert("+"todo!=null"+") :: "+("dict non-null,but todo is in constraint copy?");
                    #end
                };
                var b:Body=null;
                for(idc in dict){
                    if(idc.id==b1.id){
                        b=idc.bc;
                        break;
                    }
                }
                if(b!=null)ret.zpp_inner_zn.b1=b.zpp_inner;
                else todo.push(ZPP_CopyHelper.todo(b1.id,function(b:Body)ret.zpp_inner_zn.b1=b.zpp_inner));
            }
        };
        {
            if(dict!=null&&b2!=null){
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        todo!=null;
                    };
                    if(!res)throw "assert("+"todo!=null"+") :: "+("dict non-null,but todo is in constraint copy?");
                    #end
                };
                var b:Body=null;
                for(idc in dict){
                    if(idc.id==b2.id){
                        b=idc.bc;
                        break;
                    }
                }
                if(b!=null)ret.zpp_inner_zn.b2=b.zpp_inner;
                else todo.push(ZPP_CopyHelper.todo(b2.id,function(b:Body)ret.zpp_inner_zn.b2=b.zpp_inner));
            }
        };
        return ret;
    }
    public function new(){
        super();
        stepped=false;
        {
            jAccx=0;
            jAccy=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((jAccx!=jAccx));
                };
                if(!res)throw "assert("+"!assert_isNaN(jAccx)"+") :: "+("vec_set(in n: "+"jAcc"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((jAccy!=jAccy));
                };
                if(!res)throw "assert("+"!assert_isNaN(jAccy)"+") :: "+("vec_set(in n: "+"jAcc"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        jMax=ZPP_Const.POSINF();
        {
            {
                a1localx=0;
                a1localy=0;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a1localx!=a1localx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a1localx)"+") :: "+("vec_set(in n: "+"a1local"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a1localy!=a1localy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a1localy)"+") :: "+("vec_set(in n: "+"a1local"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
            };
            {
                a1relx=0;
                a1rely=0;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a1relx!=a1relx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a1relx)"+") :: "+("vec_set(in n: "+"a1rel"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a1rely!=a1rely));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a1rely)"+") :: "+("vec_set(in n: "+"a1rel"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
            };
        };
        {
            {
                a2localx=0;
                a2localy=0;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a2localx!=a2localx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a2localx)"+") :: "+("vec_set(in n: "+"a2local"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a2localy!=a2localy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a2localy)"+") :: "+("vec_set(in n: "+"a2local"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
            };
            {
                a2relx=0;
                a2rely=0;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a2relx!=a2relx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a2relx)"+") :: "+("vec_set(in n: "+"a2rel"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a2rely!=a2rely));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a2rely)"+") :: "+("vec_set(in n: "+"a2rel"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
            };
        };
    }
    public override function validate(){
        if(b1==null||b2==null)throw "Error: PivotJoint cannot be simulated null bodies";
        if(b1==b2)throw "Error: PivotJoint cannot be simulated with body1 == body2 (body1=body2="+b1.outer.toString()+")";
        if(b1.space!=space||b2.space!=space)throw "Error: Constraints must have each body within the same space to which the constraint has been assigned (body1="+b1.outer.toString()+", body2="+b2.outer.toString()+")";
        if(!b1.isDynamic()&&!b2.isDynamic())throw "Error: Constraints cannot have both bodies non-dynamic (body1="+b1.outer.toString()+", body2="+b2.outer.toString()+")";
    }
    public override function wake_connected(){
        if(b1!=null&&b1.isDynamic())b1.wake();
        if(b2!=null&&b2.isDynamic())b2.wake();
    }
    public override function forest(){
        if(b1.isDynamic()){
            var xr=({
                if(b1.component==b1.component.parent)b1.component;
                else{
                    var obj=b1.component;
                    var stack=null;
                    while(obj!=obj.parent){
                        var nxt=obj.parent;
                        obj.parent=stack;
                        stack=obj;
                        obj=nxt;
                    }
                    while(stack!=null){
                        var nxt=stack.parent;
                        stack.parent=obj;
                        stack=nxt;
                    }
                    obj;
                }
            });
            var yr=({
                if(component==component.parent)component;
                else{
                    var obj=component;
                    var stack=null;
                    while(obj!=obj.parent){
                        var nxt=obj.parent;
                        obj.parent=stack;
                        stack=obj;
                        obj=nxt;
                    }
                    while(stack!=null){
                        var nxt=stack.parent;
                        stack.parent=obj;
                        stack=nxt;
                    }
                    obj;
                }
            });
            if(xr!=yr){
                if(xr.rank<yr.rank)xr.parent=yr;
                else if(xr.rank>yr.rank)yr.parent=xr;
                else{
                    yr.parent=xr;
                    xr.rank++;
                }
            }
        };
        if(b2.isDynamic()){
            var xr=({
                if(b2.component==b2.component.parent)b2.component;
                else{
                    var obj=b2.component;
                    var stack=null;
                    while(obj!=obj.parent){
                        var nxt=obj.parent;
                        obj.parent=stack;
                        stack=obj;
                        obj=nxt;
                    }
                    while(stack!=null){
                        var nxt=stack.parent;
                        stack.parent=obj;
                        stack=nxt;
                    }
                    obj;
                }
            });
            var yr=({
                if(component==component.parent)component;
                else{
                    var obj=component;
                    var stack=null;
                    while(obj!=obj.parent){
                        var nxt=obj.parent;
                        obj.parent=stack;
                        stack=obj;
                        obj=nxt;
                    }
                    while(stack!=null){
                        var nxt=stack.parent;
                        stack.parent=obj;
                        stack=nxt;
                    }
                    obj;
                }
            });
            if(xr!=yr){
                if(xr.rank<yr.rank)xr.parent=yr;
                else if(xr.rank>yr.rank)yr.parent=xr;
                else{
                    yr.parent=xr;
                    xr.rank++;
                }
            }
        };
    }
    public override function pair_exists(id:Int,di:Int){
        return(b1.id==id&&b2.id==di)||(b1.id==di&&b2.id==id);
    }
    public override function clearcache(){
        {
            jAccx=0;
            jAccy=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((jAccx!=jAccx));
                };
                if(!res)throw "assert("+"!assert_isNaN(jAccx)"+") :: "+("vec_set(in n: "+"jAcc"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((jAccy!=jAccy));
                };
                if(!res)throw "assert("+"!assert_isNaN(jAccy)"+") :: "+("vec_set(in n: "+"jAcc"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        pre_dt=-1.0;
    }
    public override function preStep(dt:Float){
        if(pre_dt==-1.0)pre_dt=dt;
        var dtratio=dt/pre_dt;
        pre_dt=dt;
        stepped=true;
        {
            a1relx=(b1.axisy*a1localx-b1.axisx*a1localy);
            a1rely=(a1localx*b1.axisx+a1localy*b1.axisy);
        };
        {
            a2relx=(b2.axisy*a2localx-b2.axisx*a2localy);
            a2rely=(a2localx*b2.axisx+a2localy*b2.axisy);
        };
        {
            var m=b1.smass+b2.smass;
            {
                kMassa=m;
                kMassb=0;
                kMassc=m;
            };
            if(b1.sinertia!=0){
                var X=a1relx*b1.sinertia;
                var Y=a1rely*b1.sinertia;
                {
                    kMassa+=Y*a1rely;
                    kMassb+=-Y*a1relx;
                    kMassc+=X*a1relx;
                };
            };
            if(b2.sinertia!=0){
                var X=a2relx*b2.sinertia;
                var Y=a2rely*b2.sinertia;
                {
                    kMassa+=Y*a2rely;
                    kMassb+=-Y*a2relx;
                    kMassc+=X*a2relx;
                };
            };
        };
        var flag={
            var det=(kMassa*kMassc-kMassb*kMassb);
            if((det!=det)){
                kMassa=kMassb=kMassc=0;
                3;
            }
            else if(det==0){
                var flag=0;
                if(kMassa!=0)kMassa=1/kMassa;
                else{
                    kMassa=0;
                    flag|=1;
                }
                if(kMassc!=0)kMassc=1/kMassc;
                else{
                    kMassc=0;
                    flag|=2;
                }
                kMassb=0;
                flag;
            }
            else{
                det=1/det;
                var t=kMassc*det;
                kMassc=kMassa*det;
                kMassa=t;
                kMassb*=-det;
                0;
            }
        };
        if((flag&1)!=0)jAccx=0;
        if((flag&2)!=0)jAccy=0;
        if(!stiff){
            var biasCoef;
            {
                var X=({
                    var omega=2*Math.PI*frequency;
                    gamma=1/(dt*omega*(2*damping+omega*dt));
                    var ig=1/(1+gamma);
                    biasCoef=dt*omega*omega*gamma;
                    gamma*=ig;
                    ig;
                });
                kMassa*=X;
                kMassb*=X;
                kMassc*=X;
            };
            {
                biasx=(b2.posx+a2relx)-(b1.posx+a1relx);
                biasy=(b2.posy+a2rely)-(b1.posy+a1rely);
            };
            if(breakUnderError&&(biasx*biasx+biasy*biasy)>maxError*maxError)return true;
            {
                var t=(-biasCoef);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"bias"+",in s: "+"-biasCoef"+")");
                    #end
                };
                biasx*=t;
                biasy*=t;
            };
            {
                var t=(maxError);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_clamp(in n: "+"bias"+", in m: "+"maxError"+")");
                    #end
                };
                var ls=(biasx*biasx+biasy*biasy);
                if(ls>t*t){
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            ls!=0.0;
                        };
                        if(!res)throw "assert("+"ls!=0.0"+") :: "+("vec_clamp(in n: "+"bias"+", in m: "+"maxError"+")");
                        #end
                    };
                    {
                        var t=(t*ZPP_Math.invsqrt(ls));
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"bias"+",in s: "+"t*ZPP_Math.invsqrt(ls)"+")");
                            #end
                        };
                        biasx*=t;
                        biasy*=t;
                    };
                }
            };
        }
        else{
            {
                biasx=0;
                biasy=0;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((biasx!=biasx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(biasx)"+") :: "+("vec_set(in n: "+"bias"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((biasy!=biasy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(biasy)"+") :: "+("vec_set(in n: "+"bias"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
            };
            gamma=0;
        }
        {
            var t=(dtratio);
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((t!=t));
                };
                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"jAcc"+",in s: "+"dtratio"+")");
                #end
            };
            jAccx*=t;
            jAccy*=t;
        };
        jMax=maxForce*dt;
        return false;
    }
    public override function warmStart(){
        {
            if(false){
                {
                    var t=(b1.imass);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.pos"+",in b: "+"jAcc"+",in s: "+"b1.imass"+")");
                        #end
                    };
                    b1.posx-=jAccx*t;
                    b1.posy-=jAccy*t;
                };
                {
                    var t=(b2.imass);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.pos"+",in b: "+"jAcc"+",in s: "+"b2.imass"+")");
                        #end
                    };
                    b2.posx+=jAccx*t;
                    b2.posy+=jAccy*t;
                };
                b1.delta_rot(-(jAccy*a1relx-jAccx*a1rely)*b1.iinertia);
                b2.delta_rot((jAccy*a2relx-jAccx*a2rely)*b2.iinertia);
            }
            else{
                {
                    var t=(b1.imass);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.vel"+",in b: "+"jAcc"+",in s: "+"b1.imass"+")");
                        #end
                    };
                    b1.velx-=jAccx*t;
                    b1.vely-=jAccy*t;
                };
                {
                    var t=(b2.imass);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.vel"+",in b: "+"jAcc"+",in s: "+"b2.imass"+")");
                        #end
                    };
                    b2.velx+=jAccx*t;
                    b2.vely+=jAccy*t;
                };
                b1.angvel-=(jAccy*a1relx-jAccx*a1rely)*b1.iinertia;
                b2.angvel+=(jAccy*a2relx-jAccx*a2rely)*b2.iinertia;
            }
        };
    }
    public override function applyImpulseVel(){
        var Ex:Float=0.0;
        var Ey:Float=0.0;
        {
            Ex=(b2.velx+b2.kinvelx-a2rely*(b2.angvel+b2.kinangvel))-(b1.velx+b1.kinvelx-a1rely*(b1.angvel+b1.kinangvel));
            Ey=(b2.vely+b2.kinvely+a2relx*(b2.angvel+b2.kinangvel))-(b1.vely+b1.kinvely+a1relx*(b1.angvel+b1.kinangvel));
        };
        var Jx:Float=0.0;
        var Jy:Float=0.0;
        {
            Jx=biasx-Ex;
            Jy=biasy-Ey;
        };
        {
            var t=kMassa*Jx+kMassb*Jy;
            Jy=kMassb*Jx+kMassc*Jy;
            Jx=t;
        };
        {
            var t=(gamma);
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((t!=t));
                };
                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"J"+",in b: "+"jAcc"+",in s: "+"gamma"+")");
                #end
            };
            Jx-=jAccx*t;
            Jy-=jAccy*t;
        };
        {
            var jOldx:Float=0.0;
            var jOldy:Float=0.0;
            {
                jOldx=jAccx;
                jOldy=jAccy;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((jOldx!=jOldx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(jOldx)"+") :: "+("vec_set(in n: "+"jOld"+",in x: "+"jAccx"+",in y: "+"jAccy"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((jOldy!=jOldy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(jOldy)"+") :: "+("vec_set(in n: "+"jOld"+",in x: "+"jAccx"+",in y: "+"jAccy"+")");
                    #end
                };
            };
            {
                var t=(1.0);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"jAcc"+",in b: "+"J"+",in s: "+"1.0"+")");
                    #end
                };
                jAccx+=Jx*t;
                jAccy+=Jy*t;
            };
            {
                if(breakUnderForce){
                    if((jAccx*jAccx+jAccy*jAccy)>jMax*jMax)return true;
                }
                else if(!stiff){
                    var t=(jMax);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_clamp(in n: "+"jAcc"+", in m: "+"jMax"+")");
                        #end
                    };
                    var ls=(jAccx*jAccx+jAccy*jAccy);
                    if(ls>t*t){
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                ls!=0.0;
                            };
                            if(!res)throw "assert("+"ls!=0.0"+") :: "+("vec_clamp(in n: "+"jAcc"+", in m: "+"jMax"+")");
                            #end
                        };
                        {
                            var t=(t*ZPP_Math.invsqrt(ls));
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"jAcc"+",in s: "+"t*ZPP_Math.invsqrt(ls)"+")");
                                #end
                            };
                            jAccx*=t;
                            jAccy*=t;
                        };
                    }
                };
            };
            {
                Jx=jAccx-jOldx;
                Jy=jAccy-jOldy;
            };
        };
        {
            if(false){
                {
                    var t=(b1.imass);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.pos"+",in b: "+"J"+",in s: "+"b1.imass"+")");
                        #end
                    };
                    b1.posx-=Jx*t;
                    b1.posy-=Jy*t;
                };
                {
                    var t=(b2.imass);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.pos"+",in b: "+"J"+",in s: "+"b2.imass"+")");
                        #end
                    };
                    b2.posx+=Jx*t;
                    b2.posy+=Jy*t;
                };
                b1.delta_rot(-(Jy*a1relx-Jx*a1rely)*b1.iinertia);
                b2.delta_rot((Jy*a2relx-Jx*a2rely)*b2.iinertia);
            }
            else{
                {
                    var t=(b1.imass);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.vel"+",in b: "+"J"+",in s: "+"b1.imass"+")");
                        #end
                    };
                    b1.velx-=Jx*t;
                    b1.vely-=Jy*t;
                };
                {
                    var t=(b2.imass);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.vel"+",in b: "+"J"+",in s: "+"b2.imass"+")");
                        #end
                    };
                    b2.velx+=Jx*t;
                    b2.vely+=Jy*t;
                };
                b1.angvel-=(Jy*a1relx-Jx*a1rely)*b1.iinertia;
                b2.angvel+=(Jy*a2relx-Jx*a2rely)*b2.iinertia;
            }
        };
        return false;
    }
    public override function applyImpulsePos(){
        var r1x:Float=0.0;
        var r1y:Float=0.0;
        {
            {
                r1x=(b1.axisy*a1localx-b1.axisx*a1localy);
                r1y=(a1localx*b1.axisx+a1localy*b1.axisy);
            };
        };
        var r2x:Float=0.0;
        var r2y:Float=0.0;
        {
            {
                r2x=(b2.axisy*a2localx-b2.axisx*a2localy);
                r2y=(a2localx*b2.axisx+a2localy*b2.axisy);
            };
        };
        var Ex:Float=0.0;
        var Ey:Float=0.0;
        {
            Ex=(b2.posx+r2x)-(b1.posx+r1x);
            Ey=(b2.posy+r2y)-(b1.posy+r1y);
        };
        if(breakUnderError&&(Ex*Ex+Ey*Ey)>maxError*maxError)return true;
        if((Ex*Ex+Ey*Ey)<Config.constraintLinearSlop*Config.constraintLinearSlop)return false;
        {
            var t=(0.5);
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((t!=t));
                };
                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"E"+",in s: "+"0.5"+")");
                #end
            };
            Ex*=t;
            Ey*=t;
        };
        var Jx:Float=0.0;
        var Jy:Float=0.0;
        if((Ex*Ex+Ey*Ey)>6){
            var k=b1.smass+b2.smass;
            if(k>Config.epsilon){
                k=0.75/k;
                {
                    Jx=-Ex*k;
                    Jy=-Ey*k;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((Jx!=Jx));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(Jx)"+") :: "+("vec_set(in n: "+"J"+",in x: "+"-Ex*k"+",in y: "+"-Ey*k"+")");
                        #end
                    };
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((Jy!=Jy));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(Jy)"+") :: "+("vec_set(in n: "+"J"+",in x: "+"-Ex*k"+",in y: "+"-Ey*k"+")");
                        #end
                    };
                };
                {
                    var t=(20);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_clamp(in n: "+"J"+", in m: "+"20"+")");
                        #end
                    };
                    var ls=(Jx*Jx+Jy*Jy);
                    if(ls>t*t){
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                ls!=0.0;
                            };
                            if(!res)throw "assert("+"ls!=0.0"+") :: "+("vec_clamp(in n: "+"J"+", in m: "+"20"+")");
                            #end
                        };
                        {
                            var t=(t*ZPP_Math.invsqrt(ls));
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"J"+",in s: "+"t*ZPP_Math.invsqrt(ls)"+")");
                                #end
                            };
                            Jx*=t;
                            Jy*=t;
                        };
                    }
                };
                {
                    var t=(b1.imass);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.pos"+",in b: "+"J"+",in s: "+"b1.imass"+")");
                        #end
                    };
                    b1.posx-=Jx*t;
                    b1.posy-=Jy*t;
                };
                {
                    var t=(b2.imass);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.pos"+",in b: "+"J"+",in s: "+"b2.imass"+")");
                        #end
                    };
                    b2.posx+=Jx*t;
                    b2.posy+=Jy*t;
                };
                {
                    Ex=(b2.posx+r2x)-(b1.posx+r1x);
                    Ey=(b2.posy+r2y)-(b1.posy+r1y);
                };
                {
                    var t=(0.5);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"E"+",in s: "+"0.5"+")");
                        #end
                    };
                    Ex*=t;
                    Ey*=t;
                };
            }
        }
        var Ka:Float=0.0;
        var Kb:Float=0.0;
        var Kc:Float=0.0;
        {
            var m=b1.smass+b2.smass;
            {
                Ka=m;
                Kb=0;
                Kc=m;
            };
            if(b1.sinertia!=0){
                var X=r1x*b1.sinertia;
                var Y=r1y*b1.sinertia;
                {
                    Ka+=Y*r1y;
                    Kb+=-Y*r1x;
                    Kc+=X*r1x;
                };
            };
            if(b2.sinertia!=0){
                var X=r2x*b2.sinertia;
                var Y=r2y*b2.sinertia;
                {
                    Ka+=Y*r2y;
                    Kb+=-Y*r2x;
                    Kc+=X*r2x;
                };
            };
        };
        {
            Jx=-Ex;
            Jy=-Ey;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((Jx!=Jx));
                };
                if(!res)throw "assert("+"!assert_isNaN(Jx)"+") :: "+("vec_set(in n: "+"J"+",in x: "+"-Ex"+",in y: "+"-Ey"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((Jy!=Jy));
                };
                if(!res)throw "assert("+"!assert_isNaN(Jy)"+") :: "+("vec_set(in n: "+"J"+",in x: "+"-Ex"+",in y: "+"-Ey"+")");
                #end
            };
        };
        {
            var t=(6);
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((t!=t));
                };
                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_clamp(in n: "+"J"+", in m: "+"6"+")");
                #end
            };
            var ls=(Jx*Jx+Jy*Jy);
            if(ls>t*t){
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        ls!=0.0;
                    };
                    if(!res)throw "assert("+"ls!=0.0"+") :: "+("vec_clamp(in n: "+"J"+", in m: "+"6"+")");
                    #end
                };
                {
                    var t=(t*ZPP_Math.invsqrt(ls));
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"J"+",in s: "+"t*ZPP_Math.invsqrt(ls)"+")");
                        #end
                    };
                    Jx*=t;
                    Jy*=t;
                };
            }
        };
        {
            var det=(Ka*Kc-Kb*Kb);
            if((det!=det))Jx=Jy=0;
            else if(det==0){
                if(Ka!=0)Jx/=Ka;
                else Jx=0;
                if(Kc!=0)Jy/=Kc;
                else Jy=0;
            }
            else{
                det=1/det;
                var t=det*(Kc*Jx-Kb*Jy);
                Jy=det*(Ka*Jy-Kb*Jx);
                Jx=t;
            }
        };
        {
            if(true){
                {
                    var t=(b1.imass);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.pos"+",in b: "+"J"+",in s: "+"b1.imass"+")");
                        #end
                    };
                    b1.posx-=Jx*t;
                    b1.posy-=Jy*t;
                };
                {
                    var t=(b2.imass);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.pos"+",in b: "+"J"+",in s: "+"b2.imass"+")");
                        #end
                    };
                    b2.posx+=Jx*t;
                    b2.posy+=Jy*t;
                };
                b1.delta_rot(-(Jy*r1x-Jx*r1y)*b1.iinertia);
                b2.delta_rot((Jy*r2x-Jx*r2y)*b2.iinertia);
            }
            else{
                {
                    var t=(b1.imass);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.vel"+",in b: "+"J"+",in s: "+"b1.imass"+")");
                        #end
                    };
                    b1.velx-=Jx*t;
                    b1.vely-=Jy*t;
                };
                {
                    var t=(b2.imass);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.vel"+",in b: "+"J"+",in s: "+"b2.imass"+")");
                        #end
                    };
                    b2.velx+=Jx*t;
                    b2.vely+=Jy*t;
                };
                b1.angvel-=(Jy*r1x-Jx*r1y)*b1.iinertia;
                b2.angvel+=(Jy*r2x-Jx*r2y)*b2.iinertia;
            }
        };
        return false;
    }
    public override function draw(g:Debug){
        #if(flash9||openfl||nme)var me=outer_zn;
        var a1=me.body1.localPointToWorld(me.anchor1);
        var a2=me.body2.localPointToWorld(me.anchor2);
        if(!stiff){
            var n=a2.sub(a1);
            var nl=n.length;
            if(nl!=0)g.drawSpring(a1,a2,0xff00ff);
            n.dispose();
        }
        g.drawFilledCircle(a1,2,0xff);
        g.drawFilledCircle(a2,2,0xff0000);
        a1.dispose();
        a2.dispose();
        #end
    }
}

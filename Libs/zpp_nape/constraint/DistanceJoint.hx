package zpp_nape.constraint;
import zpp_nape.Const;
import zpp_nape.constraint.PivotJoint;
import zpp_nape.ID;
import zpp_nape.constraint.Constraint;
import zpp_nape.constraint.WeldJoint;
import zpp_nape.constraint.UserConstraint;
import zpp_nape.constraint.LineJoint;
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
class ZPP_DistanceJoint extends ZPP_Constraint{
    public var outer_zn:DistanceJoint=null;
    public var jointMin:Float=0.0;
    public var jointMax:Float=0.0;
    public var slack:Bool=false;
    public var equal:Bool=false;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function is_slack(){
        var slack;
        {
            a1relx=(b1.axisy*a1localx-b1.axisx*a1localy);
            a1rely=(a1localx*b1.axisx+a1localy*b1.axisy);
        };
        {
            a2relx=(b2.axisy*a2localx-b2.axisx*a2localy);
            a2rely=(a2localx*b2.axisx+a2localy*b2.axisy);
        };
        var nx:Float=0.0;
        var ny:Float=0.0;
        {
            nx=(b2.posx+a2relx)-(b1.posx+a1relx);
            ny=(b2.posy+a2rely)-(b1.posy+a1rely);
            var C=(nx*nx+ny*ny);
            if(C<Config.epsilon){
                {
                    nx=0;
                    ny=0;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((nx!=nx));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"0"+",in y: "+"0"+")");
                        #end
                    };
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((ny!=ny));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"0"+",in y: "+"0"+")");
                        #end
                    };
                };
                C=0;
                slack=true;
            }
            else{
                C=ZPP_Math.sqrt(C);
                {
                    var t=(1.0/(C));
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"n"+",in s: "+"1.0/(C)"+")");
                        #end
                    };
                    nx*=t;
                    ny*=t;
                };
                if(equal){
                    C-=jointMax;
                    slack=false;
                }
                else{
                    if(C<jointMin){
                        C=jointMin-C;
                        {
                            nx=-nx;
                            ny=-ny;
                        };
                        slack=false;
                    }
                    else if(C>jointMax){
                        C-=jointMax;
                        slack=false;
                    }
                    else{
                        {
                            nx=0;
                            ny=0;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((nx!=nx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"0"+",in y: "+"0"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((ny!=ny));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"0"+",in y: "+"0"+")");
                                #end
                            };
                        };
                        C=0;
                        slack=true;
                    }
                }
            }
            C;
        };
        return slack;
    }
    public var nx:Float=0.0;
    public var ny:Float=0.0;
    public var cx1:Float=0.0;
    public var cx2:Float=0.0;
    public function bodyImpulse(b:ZPP_Body){
        if(stepped){
            if(b==b1)return Vec3.get(-jAcc*nx,-jAcc*ny,-cx1*jAcc);
            else return Vec3.get(jAcc*nx,jAcc*ny,cx2*jAcc);
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
    public var kMass:Float=0.0;
    public var jAcc:Float=0.0;
    public var jMax:Float=0.0;
    public var gamma:Float=0.0;
    public var bias:Float=0.0;
    public var stepped:Bool=false;
    public override function copy(dict:Array<ZPP_CopyHelper>=null,todo:Array<ZPP_CopyHelper>=null):Constraint{
        var ret=new DistanceJoint(null,null,outer_zn.anchor1,outer_zn.anchor2,jointMin,jointMax);
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
        jAcc=0;
        jMax=ZPP_Const.POSINF();
        stepped=false;
        cx1=cx2=0;
    }
    public override function validate(){
        if(b1==null||b2==null)throw "Error: DistanceJoint cannot be simulated null bodies";
        if(b1==b2)throw "Error: DistanceJoint cannot be simulated with body1 == body2";
        if(b1.space!=space||b2.space!=space)throw "Error: Constraints must have each body within the same space to which the constraint has been assigned";
        if(jointMin>jointMax)throw "Error: DistanceJoint must have jointMin <= jointMax";
        if(!b1.isDynamic()&&!b2.isDynamic())throw "Error: Constraints cannot have both bodies non-dynamic";
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
        jAcc=0;
        pre_dt=-1.0;
    }
    public override function preStep(dt:Float){
        if(pre_dt==-1.0)pre_dt=dt;
        var dtratio=dt/pre_dt;
        pre_dt=dt;
        stepped=true;
        equal=jointMin==jointMax;
        {
            a1relx=(b1.axisy*a1localx-b1.axisx*a1localy);
            a1rely=(a1localx*b1.axisx+a1localy*b1.axisy);
        };
        {
            a2relx=(b2.axisy*a2localx-b2.axisx*a2localy);
            a2rely=(a2localx*b2.axisx+a2localy*b2.axisy);
        };
        var C={
            nx=(b2.posx+a2relx)-(b1.posx+a1relx);
            ny=(b2.posy+a2rely)-(b1.posy+a1rely);
            var C=(nx*nx+ny*ny);
            if(C<Config.epsilon){
                {
                    nx=0;
                    ny=0;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((nx!=nx));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"0"+",in y: "+"0"+")");
                        #end
                    };
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((ny!=ny));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"0"+",in y: "+"0"+")");
                        #end
                    };
                };
                C=0;
                slack=true;
            }
            else{
                C=ZPP_Math.sqrt(C);
                {
                    var t=(1.0/(C));
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"n"+",in s: "+"1.0/(C)"+")");
                        #end
                    };
                    nx*=t;
                    ny*=t;
                };
                if(equal){
                    C-=jointMax;
                    slack=false;
                }
                else{
                    if(C<jointMin){
                        C=jointMin-C;
                        {
                            nx=-nx;
                            ny=-ny;
                        };
                        slack=false;
                    }
                    else if(C>jointMax){
                        C-=jointMax;
                        slack=false;
                    }
                    else{
                        {
                            nx=0;
                            ny=0;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((nx!=nx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"0"+",in y: "+"0"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((ny!=ny));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"0"+",in y: "+"0"+")");
                                #end
                            };
                        };
                        C=0;
                        slack=true;
                    }
                }
            }
            C;
        };
        if(!slack){
            kMass={
                cx1=(ny*a1relx-nx*a1rely);
                cx2=(ny*a2relx-nx*a2rely);
                b1.smass+b2.smass+cx1*cx1*b1.sinertia+cx2*cx2*b2.sinertia;
            };
            if(kMass!=0)kMass=1/kMass;
            else jAcc=0;
            if(!stiff){
                if(breakUnderError&&C*C>maxError*maxError)return true;
                var biasCoef;
                kMass*={
                    var omega=2*Math.PI*frequency;
                    gamma=1/(dt*omega*(2*damping+omega*dt));
                    var ig=1/(1+gamma);
                    biasCoef=dt*omega*omega*gamma;
                    gamma*=ig;
                    ig;
                };
                bias=-C*biasCoef;
                {
                    if(bias<-maxError)bias=-maxError;
                    else if(bias>maxError)bias=maxError;
                };
            }
            else{
                bias=0;
                gamma=0;
            }
            jAcc*=dtratio;
            jMax=maxForce*dt;
        }
        return false;
    }
    public override function warmStart(){
        if(!slack){
            if(false){
                {
                    var t=(b1.imass*jAcc);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.pos"+",in b: "+"n"+",in s: "+"b1.imass*jAcc"+")");
                        #end
                    };
                    b1.posx-=nx*t;
                    b1.posy-=ny*t;
                };
                {
                    var t=(b2.imass*jAcc);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.pos"+",in b: "+"n"+",in s: "+"b2.imass*jAcc"+")");
                        #end
                    };
                    b2.posx+=nx*t;
                    b2.posy+=ny*t;
                };
                b1.delta_rot(-cx1*b1.iinertia*jAcc);
                b2.delta_rot(cx2*b2.iinertia*jAcc);
            }
            else{
                {
                    var t=(b1.imass*jAcc);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.vel"+",in b: "+"n"+",in s: "+"b1.imass*jAcc"+")");
                        #end
                    };
                    b1.velx-=nx*t;
                    b1.vely-=ny*t;
                };
                {
                    var t=(b2.imass*jAcc);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.vel"+",in b: "+"n"+",in s: "+"b2.imass*jAcc"+")");
                        #end
                    };
                    b2.velx+=nx*t;
                    b2.vely+=ny*t;
                };
                b1.angvel-=cx1*b1.iinertia*jAcc;
                b2.angvel+=cx2*b2.iinertia*jAcc;
            }
        };
    }
    public override function applyImpulseVel(){
        if(slack)return false;
        var E={
            nx*(b2.velx+b2.kinvelx-b1.velx-b1.kinvelx)+ny*(b2.vely+b2.kinvely-b1.vely-b1.kinvely)+(b2.angvel+b2.kinangvel)*cx2-(b1.angvel+b1.kinangvel)*cx1;
        };
        var j=kMass*(bias-E)-jAcc*gamma;
        {
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((j!=j));
                };
                if(!res)throw "assert("+"!assert_isNaN(j)"+") :: "+("accum nan");
                #end
            };
            var jOld=jAcc;
            jAcc+=j;
            {
                if(!equal&&jAcc>0)jAcc=0;
                if(breakUnderForce&&jAcc<-jMax)return true;
                if(!stiff){
                    if(jAcc<-jMax)jAcc=-jMax;
                }
            };
            j=jAcc-jOld;
        };
        {
            if(false){
                {
                    var t=(b1.imass*j);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.pos"+",in b: "+"n"+",in s: "+"b1.imass*j"+")");
                        #end
                    };
                    b1.posx-=nx*t;
                    b1.posy-=ny*t;
                };
                {
                    var t=(b2.imass*j);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.pos"+",in b: "+"n"+",in s: "+"b2.imass*j"+")");
                        #end
                    };
                    b2.posx+=nx*t;
                    b2.posy+=ny*t;
                };
                b1.delta_rot(-cx1*b1.iinertia*j);
                b2.delta_rot(cx2*b2.iinertia*j);
            }
            else{
                {
                    var t=(b1.imass*j);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.vel"+",in b: "+"n"+",in s: "+"b1.imass*j"+")");
                        #end
                    };
                    b1.velx-=nx*t;
                    b1.vely-=ny*t;
                };
                {
                    var t=(b2.imass*j);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.vel"+",in b: "+"n"+",in s: "+"b2.imass*j"+")");
                        #end
                    };
                    b2.velx+=nx*t;
                    b2.vely+=ny*t;
                };
                b1.angvel-=cx1*b1.iinertia*j;
                b2.angvel+=cx2*b2.iinertia*j;
            }
        };
        return false;
    }
    public override function applyImpulsePos(){
        var E;
        var j;
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
        var slack;
        var nx:Float=0.0;
        var ny:Float=0.0;
        E={
            nx=(b2.posx+r2x)-(b1.posx+r1x);
            ny=(b2.posy+r2y)-(b1.posy+r1y);
            var C=(nx*nx+ny*ny);
            if(C<Config.epsilon){
                {
                    nx=0;
                    ny=0;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((nx!=nx));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"0"+",in y: "+"0"+")");
                        #end
                    };
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((ny!=ny));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"0"+",in y: "+"0"+")");
                        #end
                    };
                };
                C=0;
                slack=true;
            }
            else{
                C=ZPP_Math.sqrt(C);
                {
                    var t=(1.0/(C));
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"n"+",in s: "+"1.0/(C)"+")");
                        #end
                    };
                    nx*=t;
                    ny*=t;
                };
                if(equal){
                    C-=jointMax;
                    slack=false;
                }
                else{
                    if(C<jointMin){
                        C=jointMin-C;
                        {
                            nx=-nx;
                            ny=-ny;
                        };
                        slack=false;
                    }
                    else if(C>jointMax){
                        C-=jointMax;
                        slack=false;
                    }
                    else{
                        {
                            nx=0;
                            ny=0;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((nx!=nx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"0"+",in y: "+"0"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((ny!=ny));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"0"+",in y: "+"0"+")");
                                #end
                            };
                        };
                        C=0;
                        slack=true;
                    }
                }
            }
            C;
        };
        if(!slack){
            if(breakUnderError&&E*E>maxError*maxError)return true;
            if(E*E<Config.constraintLinearSlop*Config.constraintLinearSlop)return false;
            E*=0.5;
            if(E*E>6){
                var k=b1.smass+b2.smass;
                if(k>Config.epsilon){
                    k=0.75/k;
                    j=-E*k;
                    if(equal||j<0){
                        {
                            var t=(j*b1.imass);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.pos"+",in b: "+"n"+",in s: "+"j*b1.imass"+")");
                                #end
                            };
                            b1.posx-=nx*t;
                            b1.posy-=ny*t;
                        };
                        {
                            var t=(j*b2.imass);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.pos"+",in b: "+"n"+",in s: "+"j*b2.imass"+")");
                                #end
                            };
                            b2.posx+=nx*t;
                            b2.posy+=ny*t;
                        };
                        E={
                            nx=(b2.posx+r2x)-(b1.posx+r1x);
                            ny=(b2.posy+r2y)-(b1.posy+r1y);
                            var C=(nx*nx+ny*ny);
                            if(C<Config.epsilon){
                                {
                                    nx=0;
                                    ny=0;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((nx!=nx));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"0"+",in y: "+"0"+")");
                                        #end
                                    };
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((ny!=ny));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"0"+",in y: "+"0"+")");
                                        #end
                                    };
                                };
                                C=0;
                                slack=true;
                            }
                            else{
                                C=ZPP_Math.sqrt(C);
                                {
                                    var t=(1.0/(C));
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((t!=t));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"n"+",in s: "+"1.0/(C)"+")");
                                        #end
                                    };
                                    nx*=t;
                                    ny*=t;
                                };
                                if(equal){
                                    C-=jointMax;
                                    slack=false;
                                }
                                else{
                                    if(C<jointMin){
                                        C=jointMin-C;
                                        {
                                            nx=-nx;
                                            ny=-ny;
                                        };
                                        slack=false;
                                    }
                                    else if(C>jointMax){
                                        C-=jointMax;
                                        slack=false;
                                    }
                                    else{
                                        {
                                            nx=0;
                                            ny=0;
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    !((nx!=nx));
                                                };
                                                if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"0"+",in y: "+"0"+")");
                                                #end
                                            };
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    !((ny!=ny));
                                                };
                                                if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"0"+",in y: "+"0"+")");
                                                #end
                                            };
                                        };
                                        C=0;
                                        slack=true;
                                    }
                                }
                            }
                            C;
                        };
                        E*=0.5;
                    }
                }
            }
            var cx1,cx2;
            var k={
                cx1=(ny*r1x-nx*r1y);
                cx2=(ny*r2x-nx*r2y);
                b1.smass+b2.smass+cx1*cx1*b1.sinertia+cx2*cx2*b2.sinertia;
            };
            if(k!=0)k=1/k;
            j=-E*k;
            if(equal||j<0){
                if(true){
                    {
                        var t=(b1.imass*j);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.pos"+",in b: "+"n"+",in s: "+"b1.imass*j"+")");
                            #end
                        };
                        b1.posx-=nx*t;
                        b1.posy-=ny*t;
                    };
                    {
                        var t=(b2.imass*j);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.pos"+",in b: "+"n"+",in s: "+"b2.imass*j"+")");
                            #end
                        };
                        b2.posx+=nx*t;
                        b2.posy+=ny*t;
                    };
                    b1.delta_rot(-cx1*b1.iinertia*j);
                    b2.delta_rot(cx2*b2.iinertia*j);
                }
                else{
                    {
                        var t=(b1.imass*j);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.vel"+",in b: "+"n"+",in s: "+"b1.imass*j"+")");
                            #end
                        };
                        b1.velx-=nx*t;
                        b1.vely-=ny*t;
                    };
                    {
                        var t=(b2.imass*j);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.vel"+",in b: "+"n"+",in s: "+"b2.imass*j"+")");
                            #end
                        };
                        b2.velx+=nx*t;
                        b2.vely+=ny*t;
                    };
                    b1.angvel-=cx1*b1.iinertia*j;
                    b2.angvel+=cx2*b2.iinertia*j;
                }
            };
        }
        return false;
    }
    public override function draw(g:Debug){
        #if(flash9||openfl||nme)var me=outer_zn;
        var a1=me.body1.localPointToWorld(me.anchor1);
        var a2=me.body2.localPointToWorld(me.anchor2);
        var n=a2.sub(a1);
        var nl=n.length;
        if(nl!=0){
            n.muleq(1/nl);
            var mid=a1.add(a2).muleq(0.5);
            var min1=mid.sub(n.mul(jointMin*0.5,true));
            var min2=mid.add(n.mul(jointMin*0.5,true));
            var max1=mid.sub(n.mul(jointMax*0.5,true));
            var max2=mid.add(n.mul(jointMax*0.5,true));
            g.drawLine(min1,min2,0xffff00);
            g.drawLine(max1,min1,0x00ffff);
            g.drawLine(max2,min2,0x00ffff);
            if(!stiff){
                if(nl>jointMax){
                    g.drawSpring(max1,a1,0x00ffff);
                    g.drawSpring(max2,a2,0x00ffff);
                }
                else if(nl<jointMin){
                    g.drawSpring(min1,a1,0xffff00);
                    g.drawSpring(min2,a2,0xffff00);
                }
            }
            mid.dispose();
            min1.dispose();
            min2.dispose();
            max1.dispose();
            max2.dispose();
        }
        g.drawFilledCircle(a1,2,0xff);
        g.drawFilledCircle(a2,2,0xff0000);
        a1.dispose();
        a2.dispose();
        n.dispose();
        #end
    }
}

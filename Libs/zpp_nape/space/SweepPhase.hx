package zpp_nape.space;
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
class ZPP_SweepData{
    public var next:ZPP_SweepData=null;
    static public var zpp_pool:ZPP_SweepData=null;
    #if NAPE_POOL_STATS 
    /**
     * @private
     */
    static public var POOL_CNT:Int=0;
    /**
     * @private
     */
    static public var POOL_TOT:Int=0;
    /**
     * @private
     */
    static public var POOL_ADD:Int=0;
    /**
     * @private
     */
    static public var POOL_ADDNEW:Int=0;
    /**
     * @private
     */
    static public var POOL_SUB:Int=0;
    #end
    
    public var prev:ZPP_SweepData=null;
    public var shape:ZPP_Shape=null;
    public var aabb:ZPP_AABB=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free(){
        prev=null;
        shape=null;
        aabb=null;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc(){}
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function gt(x:ZPP_SweepData){
        return aabb.minx>x.aabb.minx;
    }
}
#if nape_swc@:keep #end
class ZPP_SweepPhase extends ZPP_Broadphase{
    public var list:ZPP_SweepData=null;
    public function new(space:ZPP_Space){
        this.space=space;
        is_sweep=true;
        sweep=this;
    }
    public function __insert(shape:ZPP_Shape){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                shape.sweep==null;
            };
            if(!res)throw "assert("+"shape.sweep==null"+") :: "+("SweepPhase::insert");
            #end
        };
        var dat;
        {
            if(ZPP_SweepData.zpp_pool==null){
                dat=new ZPP_SweepData();
                #if NAPE_POOL_STATS ZPP_SweepData.POOL_TOT++;
                ZPP_SweepData.POOL_ADDNEW++;
                #end
            }
            else{
                dat=ZPP_SweepData.zpp_pool;
                ZPP_SweepData.zpp_pool=dat.next;
                dat.next=null;
                #if NAPE_POOL_STATS ZPP_SweepData.POOL_CNT--;
                ZPP_SweepData.POOL_ADD++;
                #end
            }
            dat.alloc();
        };
        shape.sweep=dat;
        dat.shape=shape;
        dat.aabb=shape.aabb;
        dat.next=list;
        if(list!=null)list.prev=dat;
        list=dat;
    }
    public function __remove(shape:ZPP_Shape){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                shape.sweep!=null;
            };
            if(!res)throw "assert("+"shape.sweep!=null"+") :: "+("SweepPhase::remove");
            #end
        };
        var dat=shape.sweep;
        if(dat.prev==null)list=dat.next;
        else dat.prev.next=dat.next;
        if(dat.next!=null)dat.next.prev=dat.prev;
        shape.sweep=null;
        {
            var o=dat;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_SweepData"+", in obj: "+"dat"+")");
                #end
            };
            o.free();
            o.next=ZPP_SweepData.zpp_pool;
            ZPP_SweepData.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_SweepData.POOL_CNT++;
            ZPP_SweepData.POOL_SUB++;
            #end
        };
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function __sync(shape:ZPP_Shape){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !shape.body.isStatic();
            };
            if(!res)throw "assert("+"!shape.body.isStatic()"+") :: "+("static shape being synced?");
            #end
        };
        if(!space.continuous)shape.validate_aabb();
    }
    public function sync_broadphase(){
        space.validation();
        if(list!=null)sync_broadphase_fast();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function sync_broadphase_fast(){
        var a=list.next;
        while(a!=null){
            #if NAPE_TIMES Debug.BROADTOTAL++;
            #end
            var n=a.next;
            var b=a.prev;
            if(a.gt(b)){
                a=n;
                continue;
            }
            #if NAPE_TIMES Debug.BROADCLASH++;
            #end
            while(b.prev!=null&&b.prev.gt(a))b=b.prev;
            var prev=a.prev;
            prev.next=a.next;
            if(a.next!=null)a.next.prev=prev;
            if(b.prev==null){
                a.prev=null;
                list=a;
                a.next=b;
                b.prev=a;
            }
            else{
                a.prev=b.prev;
                b.prev=a;
                a.prev.next=a;
                a.next=b;
            }
            a=n;
        }
    }
    public override function broadphase(space:ZPP_Space,discrete:Bool){
        if(list!=null){
            sync_broadphase_fast();
            var d1=list;
            while(d1!=null){
                var d2=d1.next;
                var s1=d1.shape;
                var b1=s1.body;
                var bottom=d1.aabb.maxx;
                while(d2!=null){
                    if(d2.aabb.minx>bottom)break;
                    var s2=d2.shape;
                    var b2=s2.body;
                    if(b2==b1){
                        d2=d2.next;
                        continue;
                    }
                    if(b1.isStatic()&&b2.isStatic()){
                        d2=d2.next;
                        continue;
                    }
                    if(b1.component.sleeping&&b2.component.sleeping){
                        d2=d2.next;
                        continue;
                    }
                    if(s1.aabb.intersectY(s2.aabb)){
                        if(discrete){
                            space.narrowPhase(s1,s2,!b1.isDynamic()||!b2.isDynamic(),null,false);
                        }
                        else{
                            space.continuousEvent(s1,s2,!b1.isDynamic()||!b2.isDynamic(),null,false);
                        }
                    }
                    d2=d2.next;
                }
                d1=d1.next;
            }
        }
    }
    public override function clear(){
        while(list!=null){
            list.shape.removedFromSpace();
            __remove(list.shape);
        }
    }
    public override function shapesUnderPoint(x:Float,y:Float,filter:ZPP_InteractionFilter,output:ShapeList){
        sync_broadphase();
        var v=ZPP_Vec2.get(x,y);
        var ret=(output==null?new ShapeList():output);
        var a=list;
        while(a!=null&&a.aabb.minx>x)a=a.next;
        while(a!=null&&a.aabb.minx<=x){
            if(a.aabb.maxx>=x&&a.aabb.miny<=y&&a.aabb.maxy>=y){
                var shape=a.shape;
                if(filter==null||shape.filter.shouldCollide(filter)){
                    if(shape.isCircle()){
                        if(ZPP_Collide.circleContains(shape.circle,v))ret.push(shape.outer);
                    }
                    else{
                        if(ZPP_Collide.polyContains(shape.polygon,v))ret.push(shape.outer);
                    }
                }
            }
            a=a.next;
        }
        {
            var o=v;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Vec2"+", in obj: "+"v"+")");
                #end
            };
            o.free();
            o.next=ZPP_Vec2.zpp_pool;
            ZPP_Vec2.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_Vec2.POOL_CNT++;
            ZPP_Vec2.POOL_SUB++;
            #end
        };
        return ret;
    }
    public override function bodiesUnderPoint(x:Float,y:Float,filter:ZPP_InteractionFilter,output:BodyList){
        sync_broadphase();
        var v=ZPP_Vec2.get(x,y);
        var ret=(output==null?new BodyList():output);
        var a=list;
        while(a!=null&&a.aabb.minx>x)a=a.next;
        while(a!=null&&a.aabb.minx<=x){
            if(a.aabb.maxx>=x&&a.aabb.miny<=y&&a.aabb.maxy>=y){
                var shape=a.shape;
                var body=shape.body.outer;
                if(!ret.has(body)){
                    if(filter==null||shape.filter.shouldCollide(filter)){
                        if(shape.isCircle()){
                            if(ZPP_Collide.circleContains(shape.circle,v))ret.push(body);
                        }
                        else{
                            if(ZPP_Collide.polyContains(shape.polygon,v))ret.push(body);
                        }
                    }
                }
            }
            a=a.next;
        }
        {
            var o=v;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Vec2"+", in obj: "+"v"+")");
                #end
            };
            o.free();
            o.next=ZPP_Vec2.zpp_pool;
            ZPP_Vec2.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_Vec2.POOL_CNT++;
            ZPP_Vec2.POOL_SUB++;
            #end
        };
        return ret;
    }
    public override function shapesInAABB(aabb:ZPP_AABB,strict:Bool,containment:Bool,filter:ZPP_InteractionFilter,output:ShapeList){
        sync_broadphase();
        updateAABBShape(aabb);
        var ab=aabbShape.zpp_inner.aabb;
        var ret=(output==null?new ShapeList():output);
        var a=list;
        while(a!=null&&a.aabb.maxx<ab.minx)a=a.next;
        while(a!=null&&a.aabb.minx<=ab.maxx){
            var shape=a.shape;
            if(filter==null||shape.filter.shouldCollide(filter)){
                if(strict){
                    if(containment){
                        if(ZPP_Collide.containTest(aabbShape.zpp_inner,shape))ret.push(shape.outer);
                    }
                    else{
                        if(ab.contains(a.aabb))ret.push(shape.outer);
                        else if(a.aabb.intersect(ab)){
                            if(ZPP_Collide.testCollide_safe(shape,aabbShape.zpp_inner))ret.push(shape.outer);
                        }
                    }
                }
                else if(containment?ab.contains(a.aabb):a.aabb.intersect(ab))ret.push(shape.outer);
            }
            a=a.next;
        }
        return ret;
    }
    public var failed:BodyList=null;
    public override function bodiesInAABB(aabb:ZPP_AABB,strict:Bool,containment:Bool,filter:ZPP_InteractionFilter,output:BodyList){
        sync_broadphase();
        updateAABBShape(aabb);
        var ab=aabbShape.zpp_inner.aabb;
        var ret=(output==null?new BodyList():output);
        if(failed==null)failed=new BodyList();
        var a=list;
        while(a!=null&&a.aabb.maxx<ab.minx)a=a.next;
        while(a!=null&&a.aabb.minx<=ab.maxx){
            var shape=a.shape;
            var body=shape.body.outer;
            if(a.aabb.intersect(ab)){
                if(filter==null||shape.filter.shouldCollide(filter)){
                    if(strict){
                        if(containment){
                            if(!failed.has(body)){
                                var col=ZPP_Collide.containTest(aabbShape.zpp_inner,shape);
                                if(!ret.has(body)&&col)ret.push(body);
                                else if(!col){
                                    ret.remove(body);
                                    failed.push(body);
                                }
                            }
                        }
                        else if(!ret.has(body)&&ZPP_Collide.testCollide_safe(shape,aabbShape.zpp_inner)){
                            ret.push(body);
                        }
                    }
                    else{
                        if(containment){
                            if(!failed.has(body)){
                                var col=ab.contains(shape.aabb);
                                if(!ret.has(body)&&col)ret.push(body);
                                else if(!col){
                                    ret.remove(body);
                                    failed.push(body);
                                }
                            }
                        }
                        else if(!ret.has(body)&&ab.contains(shape.aabb)){
                            ret.push(body);
                        }
                    }
                }
            }
            a=a.next;
        }
        failed.clear();
        return ret;
    }
    public override function shapesInCircle(x:Float,y:Float,r:Float,containment:Bool,filter:ZPP_InteractionFilter,output:ShapeList){
        sync_broadphase();
        updateCircShape(x,y,r);
        var ab=circShape.zpp_inner.aabb;
        var ret=(output==null?new ShapeList():output);
        var a=list;
        while(a!=null&&a.aabb.maxx<ab.minx)a=a.next;
        while(a!=null&&a.aabb.minx<=ab.maxx){
            if(a.aabb.intersect(ab)){
                var shape=a.shape;
                if(filter==null||shape.filter.shouldCollide(filter)){
                    if(containment){
                        if(ZPP_Collide.containTest(circShape.zpp_inner,shape))ret.push(shape.outer);
                    }
                    else if(ZPP_Collide.testCollide_safe(shape,circShape.zpp_inner))ret.push(shape.outer);
                }
            }
            a=a.next;
        }
        return ret;
    }
    public override function bodiesInCircle(x:Float,y:Float,r:Float,containment:Bool,filter:ZPP_InteractionFilter,output:BodyList){
        sync_broadphase();
        updateCircShape(x,y,r);
        var ab=circShape.zpp_inner.aabb;
        var ret=(output==null?new BodyList():output);
        if(failed==null)failed=new BodyList();
        var a=list;
        while(a!=null&&a.aabb.maxx<ab.minx)a=a.next;
        while(a!=null&&a.aabb.minx<=ab.maxx){
            if(a.aabb.intersect(ab)){
                var shape=a.shape;
                var body=shape.body.outer;
                if(filter==null||shape.filter.shouldCollide(filter)){
                    if(containment){
                        if(!failed.has(body)){
                            var col=ZPP_Collide.containTest(circShape.zpp_inner,shape);
                            if(!ret.has(body)&&col)ret.push(body);
                            else if(!col){
                                ret.remove(body);
                                failed.push(body);
                            }
                        }
                    }
                    else if(!ret.has(body)&&ZPP_Collide.testCollide_safe(shape,circShape.zpp_inner)){
                        ret.push(body);
                    }
                }
            }
            a=a.next;
        }
        failed.clear();
        return ret;
    }
    public override function shapesInShape(shape:ZPP_Shape,containment:Bool,filter:ZPP_InteractionFilter,output:ShapeList){
        sync_broadphase();
        validateShape(shape);
        var ab=shape.aabb;
        var ret=(output==null?new ShapeList():output);
        var a=list;
        while(a!=null&&a.aabb.maxx<ab.minx)a=a.next;
        while(a!=null&&a.aabb.minx<=ab.maxx){
            if(a.aabb.intersect(ab)){
                var shape2=a.shape;
                if(filter==null||shape2.filter.shouldCollide(filter)){
                    if(containment){
                        if(ZPP_Collide.containTest(shape,shape2))ret.push(shape2.outer);
                    }
                    else if(ZPP_Collide.testCollide_safe(shape2,shape))ret.push(shape2.outer);
                }
            }
            a=a.next;
        }
        return ret;
    }
    public override function bodiesInShape(shape:ZPP_Shape,containment:Bool,filter:ZPP_InteractionFilter,output:BodyList){
        sync_broadphase();
        validateShape(shape);
        var ab=shape.aabb;
        var ret=(output==null?new BodyList():output);
        if(failed==null)failed=new BodyList();
        var a=list;
        while(a!=null&&a.aabb.maxx<ab.minx)a=a.next;
        while(a!=null&&a.aabb.minx<=ab.maxx){
            if(a.aabb.intersect(ab)){
                var shape2=a.shape;
                var body=shape2.body.outer;
                if(filter==null||shape2.filter.shouldCollide(filter)){
                    if(containment){
                        if(!failed.has(body)){
                            var col=ZPP_Collide.containTest(shape,shape2);
                            if(!ret.has(body)&&col)ret.push(body);
                            else if(!col){
                                ret.remove(body);
                                failed.push(body);
                            }
                        }
                    }
                    else if(!ret.has(body)&&ZPP_Collide.testCollide_safe(shape,shape2)){
                        ret.push(body);
                    }
                }
            }
            a=a.next;
        }
        failed.clear();
        return ret;
    }
    public override function rayCast(ray:ZPP_Ray,inner:Bool,filter:ZPP_InteractionFilter){
        sync_broadphase();
        ray.validate_dir();
        var rayab=ray.rayAABB();
        var mint=ray.maxdist;
        var minres:RayResult=null;
        if(ray.dirx==0){
            var a=list;
            while(a!=null&&a.aabb.minx<=rayab.minx){
                if(a.aabb.intersect(rayab)&&(filter==null||a.shape.filter.shouldCollide(filter))){
                    var t=ray.aabbsect(a.aabb);
                    if(t>=0&&t<mint){
                        var result=if(a.shape.isCircle())ray.circlesect(a.shape.circle,inner,mint)else ray.polysect(a.shape.polygon,inner,mint);
                        if(result!=null){
                            mint=result.distance;
                            if(minres!=null){
                                minres.dispose();
                            }
                            minres=result;
                        }
                    }
                }
                a=a.next;
            }
        }
        else if(ray.dirx<0){
            var a=list;
            var b=null;
            while(a!=null&&a.aabb.minx<=rayab.maxx){
                b=a;
                a=a.next;
            }
            a=b;
            while(a!=null){
                if(a.aabb.intersect(rayab)&&(filter==null||a.shape.filter.shouldCollide(filter))){
                    var t=ray.aabbsect(a.aabb);
                    if(t>=0&&t<mint){
                        var result=if(a.shape.isCircle())ray.circlesect(a.shape.circle,inner,mint)else ray.polysect(a.shape.polygon,inner,mint);
                        if(result!=null){
                            mint=result.distance;
                            if(minres!=null){
                                minres.dispose();
                            }
                            minres=result;
                        }
                    }
                }
                a=a.prev;
            }
        }
        else{
            var a=list;
            while(a!=null&&a.aabb.minx<=rayab.maxx&&a.aabb.minx<ray.originx+ray.dirx*mint){
                if(a.aabb.intersect(rayab)&&(filter==null||a.shape.filter.shouldCollide(filter))){
                    var t=ray.aabbsect(a.aabb);
                    if(t>=0&&t<mint){
                        var result=if(a.shape.isCircle())ray.circlesect(a.shape.circle,inner,mint)else ray.polysect(a.shape.polygon,inner,mint);
                        if(result!=null){
                            mint=result.distance;
                            if(minres!=null){
                                minres.dispose();
                            }
                            minres=result;
                        }
                    }
                }
                a=a.next;
            }
        }
        {
            var o=rayab;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_AABB"+", in obj: "+"rayab"+")");
                #end
            };
            o.free();
            o.next=ZPP_AABB.zpp_pool;
            ZPP_AABB.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_AABB.POOL_CNT++;
            ZPP_AABB.POOL_SUB++;
            #end
        };
        return minres;
    }
    public override function rayMultiCast(ray:ZPP_Ray,inner:Bool,filter:ZPP_InteractionFilter,output:RayResultList){
        sync_broadphase();
        ray.validate_dir();
        var rayab=ray.rayAABB();
        var ret=(output==null?new RayResultList():output);
        if(ray.dirx==0){
            var a=list;
            while(a!=null&&a.aabb.minx<=rayab.minx){
                if(a.aabb.intersect(rayab)&&(filter==null||a.shape.filter.shouldCollide(filter))){
                    var t=ray.aabbsect(a.aabb);
                    if(t>=0){
                        if(a.shape.isCircle())ray.circlesect2(a.shape.circle,inner,ret);
                        else ray.polysect2(a.shape.polygon,inner,ret);
                    }
                }
                a=a.next;
            }
        }
        else if(ray.dirx<0){
            var a=list;
            var b=null;
            while(a!=null&&a.aabb.minx<=rayab.maxx){
                b=a;
                a=a.next;
            }
            a=b;
            while(a!=null){
                if(a.aabb.intersect(rayab)&&(filter==null||a.shape.filter.shouldCollide(filter))){
                    var t=ray.aabbsect(a.aabb);
                    if(t>=0){
                        if(a.shape.isCircle())ray.circlesect2(a.shape.circle,inner,ret);
                        else ray.polysect2(a.shape.polygon,inner,ret);
                    }
                }
                a=a.prev;
            }
        }
        else{
            var a=list;
            while(a!=null&&a.aabb.minx<=rayab.maxx){
                if(a.aabb.intersect(rayab)&&(filter==null||a.shape.filter.shouldCollide(filter))){
                    var t=ray.aabbsect(a.aabb);
                    if(t>=0){
                        if(a.shape.isCircle())ray.circlesect2(a.shape.circle,inner,ret);
                        else ray.polysect2(a.shape.polygon,inner,ret);
                    }
                }
                a=a.next;
            }
        }
        {
            var o=rayab;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_AABB"+", in obj: "+"rayab"+")");
                #end
            };
            o.free();
            o.next=ZPP_AABB.zpp_pool;
            ZPP_AABB.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_AABB.POOL_CNT++;
            ZPP_AABB.POOL_SUB++;
            #end
        };
        return ret;
    }
}

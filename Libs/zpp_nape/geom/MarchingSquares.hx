package zpp_nape.geom;
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
class ZPP_MarchSpan{
    public var parent:ZPP_MarchSpan=null;
    public var rank:Int=0;
    public var out:Bool=false;
    public var next:ZPP_MarchSpan=null;
    static public var zpp_pool:ZPP_MarchSpan=null;
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
    
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free(){
        parent=this;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc(){
        out=false;
        rank=0;
    }
    public function new(){
        parent=this;
    }
}
#if nape_swc@:keep #end
class ZPP_MarchPair{
    public var p1:ZPP_GeomVert=null;
    public var key1:Int=0;
    public var okey1:Int=0;
    public var p2:ZPP_GeomVert=null;
    public var key2:Int=0;
    public var okey2:Int=0;
    public var pr:ZPP_GeomVert=null;
    public var keyr:Int=0;
    public var okeyr:Int=0;
    public var pd:ZPP_GeomVert=null;
    public var span1:ZPP_MarchSpan=null;
    public var span2:ZPP_MarchSpan=null;
    public var spanr:ZPP_MarchSpan=null;
    public var next:ZPP_MarchPair=null;
    static public var zpp_pool:ZPP_MarchPair=null;
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
    
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free(){
        p1=p2=pr=pd=null;
        span1=span2=spanr=null;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc(){}
    public function new(){}
}
#if nape_swc@:keep #end
class ZPP_MarchingSquares{
    static var me:ZPP_MarchingSquares=new ZPP_MarchingSquares();
    function new(){}
    static var isos:ZNPArray2_Float;
    static var ints:ZNPArray2_ZPP_GeomVert;
    static var map:ZNPArray2_ZPP_MarchPair;
    public static function run(iso:IsoFunctionDef,bx0:Float,by0:Float,bx1:Float,by1:Float,cell:Vec2,quality:Int,combine:Bool,ret:GeomPolyList){
        var xp=(bx1-bx0)/cell.x;
        var xn:Int=(#if flash9 untyped __int__(xp)#else Std.int(xp)#end);
        var yp=(by1-by0)/cell.y;
        var yn:Int=(#if flash9 untyped __int__(yp)#else Std.int(yp)#end);
        if(xp!=xn)xn++;
        if(yp!=yn)yn++;
        if(combine){
            if(map==null){
                map=new ZNPArray2_ZPP_MarchPair(xn,yn);
            }
            else{
                map.resize(xn,yn,null);
            }
        }
        if(isos==null){
            isos=new ZNPArray2_Float(xn+1,yn+1);
        }
        else{
            isos.resize(xn+1,yn+1,0);
        }
        for(y in 0...yn+1){
            var yc=if(y==0)by0 else if(y<=yn)by0+cell.y*y else by1;
            for(x in 0...xn+1){
                var xc=if(x==0)bx0 else if(x<=xn)bx0+cell.x*x else bx1;
                isos.set(x,y,ISO(iso,xc,yc));
            }
        }
        if(ints==null){
            ints=new ZNPArray2_ZPP_GeomVert(xn+1,(yn<<1)+1);
        }
        else{
            ints.resize(xn+1,(yn<<1)+1,null);
        }
        var spans:ZPP_MarchSpan=null;
        if(combine){
            {
                if(ZPP_MarchSpan.zpp_pool==null){
                    spans=new ZPP_MarchSpan();
                    #if NAPE_POOL_STATS ZPP_MarchSpan.POOL_TOT++;
                    ZPP_MarchSpan.POOL_ADDNEW++;
                    #end
                }
                else{
                    spans=ZPP_MarchSpan.zpp_pool;
                    ZPP_MarchSpan.zpp_pool=spans.next;
                    spans.next=null;
                    #if NAPE_POOL_STATS ZPP_MarchSpan.POOL_CNT--;
                    ZPP_MarchSpan.POOL_ADD++;
                    #end
                }
                spans.alloc();
            };
        }
        var py=by0;
        for(y in 0...yn){
            var y0=py;
            var y1=if(y==yn-1)by1 else by0+cell.y*(y+1);
            py=y1;
            var px=bx0;
            var pre:ZPP_MarchPair=null;
            for(x in 0...xn){
                var x0=px;
                var x1=if(x==xn-1)bx1 else bx0+cell.x*(x+1);
                px=x1;
                var fstx=x==0||!combine;
                var fsty=y==0||!combine;
                var sndx=x==xn-1||!combine;
                var sndy=y==yn-1||!combine;
                var pp=me.marchSquare(iso,isos,ints,x0,y0,x1,y1,x,y,fstx,fsty,sndx,sndy,quality);
                if(pp==null){
                    pre=null;
                    continue;
                }
                if(combine){
                    var pd=if(pp.p2!=null&&pp.okey2!=14)pp.p2 else pp.p1;
                    pp.pd=me.linkdown(pd,pd==pp.p2?pp.okey2:pp.okey1);
                    map.set(x,y,pp);
                    if(pre!=null&&me.combLeft(pp.key1)){
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                me.combRight(pre.keyr);
                            };
                            if(!res)throw "assert("+"me.combRight(pre.keyr)"+") :: "+("left poly cannot comb right?");
                            #end
                        };
                        me.combLR(pre,pp);
                        pp.span1=pre.spanr;
                    }
                    else{
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                pp.span1==null;
                            };
                            if(!res)throw "assert("+"pp.span1==null"+") :: "+("oops");
                            #end
                        };
                        {
                            if(ZPP_MarchSpan.zpp_pool==null){
                                pp.span1=new ZPP_MarchSpan();
                                #if NAPE_POOL_STATS ZPP_MarchSpan.POOL_TOT++;
                                ZPP_MarchSpan.POOL_ADDNEW++;
                                #end
                            }
                            else{
                                pp.span1=ZPP_MarchSpan.zpp_pool;
                                ZPP_MarchSpan.zpp_pool=pp.span1.next;
                                pp.span1.next=null;
                                #if NAPE_POOL_STATS ZPP_MarchSpan.POOL_CNT--;
                                ZPP_MarchSpan.POOL_ADD++;
                                #end
                            }
                            pp.span1.alloc();
                        };
                        pp.span1.next=spans;
                        spans=pp.span1;
                    }
                    if(pp.p2!=null){
                        {
                            if(ZPP_MarchSpan.zpp_pool==null){
                                pp.span2=new ZPP_MarchSpan();
                                #if NAPE_POOL_STATS ZPP_MarchSpan.POOL_TOT++;
                                ZPP_MarchSpan.POOL_ADDNEW++;
                                #end
                            }
                            else{
                                pp.span2=ZPP_MarchSpan.zpp_pool;
                                ZPP_MarchSpan.zpp_pool=pp.span2.next;
                                pp.span2.next=null;
                                #if NAPE_POOL_STATS ZPP_MarchSpan.POOL_CNT--;
                                ZPP_MarchSpan.POOL_ADD++;
                                #end
                            }
                            pp.span2.alloc();
                        };
                        pp.span2.next=spans;
                        spans=pp.span2;
                        pp.spanr=pp.span2;
                    }
                    else pp.spanr=pp.span1;
                    if(me.combRight(pp.keyr))pre=pp;
                    else pre=null;
                }
                else{
                    me.output(ret,pp.p1);
                    if(pp.p2!=null)me.output(ret,pp.p2);
                    {
                        var o=pp;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                o!=null;
                            };
                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_MarchPair"+", in obj: "+"pp"+")");
                            #end
                        };
                        o.free();
                        o.next=ZPP_MarchPair.zpp_pool;
                        ZPP_MarchPair.zpp_pool=o;
                        #if NAPE_POOL_STATS ZPP_MarchPair.POOL_CNT++;
                        ZPP_MarchPair.POOL_SUB++;
                        #end
                    };
                }
            }
        }
        if(!combine){
            return;
        }
        for(y in 1...yn){
            var pre:ZPP_MarchSpan=null;
            for(x in 0...xn){
                var b=map.get(x,y);
                if(b==null){
                    pre=null;
                    continue;
                }
                var bkey=if(b.p2!=null&&b.okey2==0x0e)b.okey2 else b.okey1;
                if(!me.combUp(bkey)){
                    pre=null;
                    continue;
                }
                var a=map.get(x,y-1);
                if(a==null){
                    pre=null;
                    continue;
                }
                var akey=if(a.p2!=null&&a.okey2==0x38)a.okey2 else a.okey1;
                if(!me.combDown(akey)){
                    pre=null;
                    continue;
                }
                var ad=if(a.p2!=null&&a.okey2==0x38)a.span2 else a.span1;
                var bu=if(b.p2!=null&&b.okey2==0x0e)b.span2 else b.span1;
                if(({
                    if(ad==ad.parent)ad;
                    else{
                        var obj=ad;
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
                })==({
                    if(bu==bu.parent)bu;
                    else{
                        var obj=bu;
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
                })){
                    if(pre!=bu){
                        me.combUD_virtual(a,b);
                    }
                }
                else{
                    {
                        var xr=({
                            if(ad==ad.parent)ad;
                            else{
                                var obj=ad;
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
                            if(bu==bu.parent)bu;
                            else{
                                var obj=bu;
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
                    me.combUD(a,b);
                }
                var key=if(bu==b.span2)b.okey2 else b.okey1;
                pre=if((key&4)!=0)bu else null;
            }
        }
        for(y in 0...yn){
            for(x in 0...xn){
                var p=map.get(x,y);
                if(p==null)continue;
                var root=({
                    if(p.span1==p.span1.parent)p.span1;
                    else{
                        var obj=p.span1;
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
                if(!root.out){
                    root.out=true;
                    me.output(ret,p.p1);
                }
                if(p.p2!=null){
                    root=({
                        if(p.span2==p.span2.parent)p.span2;
                        else{
                            var obj=p.span2;
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
                    if(!root.out){
                        root.out=true;
                        me.output(ret,p.p2);
                    }
                }
                {
                    var o=p;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            o!=null;
                        };
                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_MarchPair"+", in obj: "+"p"+")");
                        #end
                    };
                    o.free();
                    o.next=ZPP_MarchPair.zpp_pool;
                    ZPP_MarchPair.zpp_pool=o;
                    #if NAPE_POOL_STATS ZPP_MarchPair.POOL_CNT++;
                    ZPP_MarchPair.POOL_SUB++;
                    #end
                };
                map.set(x,y,null);
            }
        }
        while(spans!=null){
            var t=spans;
            spans=t.next;
            {
                var o=t;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o!=null;
                    };
                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_MarchSpan"+", in obj: "+"t"+")");
                    #end
                };
                o.free();
                o.next=ZPP_MarchSpan.zpp_pool;
                ZPP_MarchSpan.zpp_pool=o;
                #if NAPE_POOL_STATS ZPP_MarchSpan.POOL_CNT++;
                ZPP_MarchSpan.POOL_SUB++;
                #end
            };
        }
    }
    function output(ret:GeomPolyList,poly:ZPP_GeomVert){
        if(({
            if(poly==null||poly.next==poly||poly.next==poly.prev){
                true;
            }
            else{
                var a=({
                    {
                        #if NAPE_ASSERT if(({
                            var ret=0;
                            {
                                var F=poly;
                                var L=poly;
                                if(F!=null){
                                    var nite=F;
                                    do{
                                        var i=nite;
                                        {
                                            ret++;
                                        }
                                        nite=nite.next;
                                    }
                                    while(nite!=L);
                                }
                            };
                            ret;
                        })<3){
                            throw "Error: Method requires that polygon has atleast 3 vertices";
                        }
                        #end
                    };
                    var area=0.0;
                    {
                        var F=poly;
                        var L=poly;
                        if(F!=null){
                            var nite=F;
                            do{
                                var v=nite;
                                {
                                    {
                                        area+=v.x*(v.next.y-v.prev.y);
                                    };
                                }
                                nite=nite.next;
                            }
                            while(nite!=L);
                        }
                    };
                    area*0.5;
                });
                (a*a)<(Config.epsilon*Config.epsilon);
            }
        })){
            {
                while(!(poly==null))poly={
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !(poly==null);
                        };
                        if(!res)throw "assert("+"!(poly==null)"+") :: "+("can't pop from empty list herpaderp");
                        #end
                    };
                    if((poly!=null&&poly.prev==poly)){
                        poly.next=poly.prev=null;
                        {};
                        poly=null;
                    }
                    else{
                        var retnodes=poly.next;
                        poly.prev.next=poly.next;
                        poly.next.prev=poly.prev;
                        poly.next=poly.prev=null;
                        {};
                        poly=null;
                        retnodes;
                    }
                }
            };
            return;
        }
        var gp=GeomPoly.get();
        gp.zpp_inner.vertices=poly;
        ret.add(gp);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    #if NAPE_NO_INLINE#else inline #end
    function linkright(poly:ZPP_GeomVert,key:Int){
        var kind=key&7;
        if(kind==0)return poly;
        else if(kind==3)return poly.next.next;
        else return poly.next;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    #if NAPE_NO_INLINE#else inline #end
    function linkleft(poly:ZPP_GeomVert,key:Int){
        return if((key&1)==0)poly.prev else poly;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    #if NAPE_NO_INLINE#else inline #end
    function linkdown(poly:ZPP_GeomVert,key:Int){
        return if((key&0x80)==0)poly.prev else poly.prev.prev;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    #if NAPE_NO_INLINE#else inline #end
    function linkup(poly:ZPP_GeomVert,key:Int){
        return poly;
    }
    function combLR(a:ZPP_MarchPair,b:ZPP_MarchPair){
        var ap:ZPP_GeomVert=linkright(a.pr,a.okeyr);
        var bp:ZPP_GeomVert=linkleft(b.p1,b.okey1);
        var ap2=ap.next;
        var bp2=bp.prev;
        if((a.keyr&0x04)!=0){
            if(b.pr==b.p1)b.pr=ap.prev;
            b.p1=ap.prev;
            ap.prev.next=bp.next;
            bp.next.prev=ap.prev;
            {
                var o=ap;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o!=null;
                    };
                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_GeomVert"+", in obj: "+"ap"+")");
                    #end
                };
                o.free();
                o.next=ZPP_GeomVert.zpp_pool;
                ZPP_GeomVert.zpp_pool=o;
                #if NAPE_POOL_STATS ZPP_GeomVert.POOL_CNT++;
                ZPP_GeomVert.POOL_SUB++;
                #end
            };
        }
        else{
            ap.next=bp.next;
            bp.next.prev=ap;
        }
        {
            var o=bp;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_GeomVert"+", in obj: "+"bp"+")");
                #end
            };
            o.free();
            o.next=ZPP_GeomVert.zpp_pool;
            ZPP_GeomVert.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_GeomVert.POOL_CNT++;
            ZPP_GeomVert.POOL_SUB++;
            #end
        };
        if((a.keyr&0x10)!=0){
            b.pd=ap2.next;
            ap2.next.prev=bp2.prev;
            bp2.prev.next=ap2.next;
            {
                var o=ap2;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o!=null;
                    };
                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_GeomVert"+", in obj: "+"ap2"+")");
                    #end
                };
                o.free();
                o.next=ZPP_GeomVert.zpp_pool;
                ZPP_GeomVert.zpp_pool=o;
                #if NAPE_POOL_STATS ZPP_GeomVert.POOL_CNT++;
                ZPP_GeomVert.POOL_SUB++;
                #end
            };
        }
        else{
            ap2.prev=bp2.prev;
            bp2.prev.next=ap2;
        }
        {
            var o=bp2;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_GeomVert"+", in obj: "+"bp2"+")");
                #end
            };
            o.free();
            o.next=ZPP_GeomVert.zpp_pool;
            ZPP_GeomVert.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_GeomVert.POOL_CNT++;
            ZPP_GeomVert.POOL_SUB++;
            #end
        };
    }
    function combUD(a:ZPP_MarchPair,b:ZPP_MarchPair){
        var ad=if(a.p2!=null&&a.key2==0x38)a.p2 else a.p1;
        var bu=if(b.p2!=null&&b.key2==0x0e)b.p2 else b.p1;
        var ap:ZPP_GeomVert=a.pd;
        var bp:ZPP_GeomVert=linkup(bu,bu==b.p2?b.okey2:b.okey1);
        var ap2=ap.prev;
        var bp2=bp.next;
        bp.next=ap.next;
        ap.next.prev=bp;
        {
            var o=ap;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_GeomVert"+", in obj: "+"ap"+")");
                #end
            };
            o.free();
            o.next=ZPP_GeomVert.zpp_pool;
            ZPP_GeomVert.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_GeomVert.POOL_CNT++;
            ZPP_GeomVert.POOL_SUB++;
            #end
        };
        bp2.prev=ap2.prev;
        ap2.prev.next=bp2;
        if(ap2==a.p1)a.p1=bp2;
        {
            var o=ap2;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_GeomVert"+", in obj: "+"ap2"+")");
                #end
            };
            o.free();
            o.next=ZPP_GeomVert.zpp_pool;
            ZPP_GeomVert.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_GeomVert.POOL_CNT++;
            ZPP_GeomVert.POOL_SUB++;
            #end
        };
    }
    function combUD_virtual(a:ZPP_MarchPair,b:ZPP_MarchPair){
        var ad=if(a.p2!=null&&a.key2==0x38)a.p2 else a.p1;
        var bu=if(b.p2!=null&&b.key2==0x0e)b.p2 else b.p1;
        var ap:ZPP_GeomVert=a.pd;
        var bp:ZPP_GeomVert=linkup(bu,bu==b.p2?b.key2:b.key1);
        var ap2=ap.prev;
        var bp2=bp.next;
        ap.forced=bp.forced=ap2.forced=bp2.forced=true;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    #if NAPE_NO_INLINE#else inline #end
    function combLeft(key:Int){
        var flag=(key&1)|((key&0xC0)>>5);
        return comb(flag);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    #if NAPE_NO_INLINE#else inline #end
    function combRight(key:Int){
        var flag=(key&0x1C)>>2;
        return comb(flag);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    #if NAPE_NO_INLINE#else inline #end
    function combUp(key:Int){
        var flag=(key&0x07);
        return comb(flag);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    #if NAPE_NO_INLINE#else inline #end
    function combDown(key:Int){
        var flag=(key&0x70)>>4;
        return comb(flag);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    #if NAPE_NO_INLINE#else inline #end
    function comb(flag:Int){
        var cnt=0;
        if((flag&1)!=0)cnt++;
        if((flag&2)!=0)cnt++;
        if((flag&4)!=0)cnt++;
        return cnt>=2;
    }
    public static var look_march=[-1,0xE0,0x38,0xD8,0x0E,-1,0x36,0xD6,0x83,0x63,-1,0x5B,0x8D,0x6D,0xB5,0x55];
    function marchSquare(iso:IsoFunctionDef,isos:ZNPArray2_Float,ints:ZNPArray2_ZPP_GeomVert,x0:Float,y0:Float,x1:Float,y1:Float,xn:Int,yn:Int,fstx:Bool,fsty:Bool,sndx:Bool,sndy:Bool,quality:Int){
        var key=0;
        var v0=isos.get(xn,yn);
        if(v0<0)key|=8;
        var v1=isos.get(xn+1,yn);
        if(v1<0)key|=4;
        var v2=isos.get(xn+1,yn+1);
        if(v2<0)key|=2;
        var v3=isos.get(xn,yn+1);
        if(v3<0)key|=1;
        if(key==0)return null;
        else{
            var ret;
            {
                if(ZPP_MarchPair.zpp_pool==null){
                    ret=new ZPP_MarchPair();
                    #if NAPE_POOL_STATS ZPP_MarchPair.POOL_TOT++;
                    ZPP_MarchPair.POOL_ADDNEW++;
                    #end
                }
                else{
                    ret=ZPP_MarchPair.zpp_pool;
                    ZPP_MarchPair.zpp_pool=ret.next;
                    ret.next=null;
                    #if NAPE_POOL_STATS ZPP_MarchPair.POOL_CNT--;
                    ZPP_MarchPair.POOL_ADD++;
                    #end
                }
                ret.alloc();
            };
            if(key!=10&&key!=5){
                var val=look_march[key];
                {
                    ret.okey1=val;
                    for(i in 0...8){
                        if((val&(1<<i))!=0){
                            var p:ZPP_GeomVert=null;
                            if(i==0){
                                p=ZPP_GeomVert.get(x0,y0);
                                if(fstx||fsty)p.forced=true;
                            }
                            else if(i==2){
                                p=ZPP_GeomVert.get(x1,y0);
                                if(sndx||fsty)p.forced=true;
                            }
                            else if(i==4){
                                p=ZPP_GeomVert.get(x1,y1);
                                if(sndx||sndy)p.forced=true;
                            }
                            else if(i==6){
                                p=ZPP_GeomVert.get(x0,y1);
                                if(fstx||sndy)p.forced=true;
                            }
                            else if(i==1){
                                p=ints.get(xn,yn<<1);
                                if(p==null){
                                    p=ZPP_GeomVert.get(xlerp(x0,x1,y0,v0,v1,iso,quality),y0);
                                    ints.set(xn,yn<<1,p);
                                }
                                else p=ZPP_GeomVert.get(p.x,p.y);
                                if(fsty)p.forced=true;
                                if(p.x==x0||p.x==x1){
                                    if((p.x==x0&&(val&1)!=0)||(p.x==x1&&(val&4)!=0))val^=0x02;
                                }
                            }
                            else if(i==5){
                                p=ints.get(xn,(yn<<1)+2);
                                if(p==null){
                                    p=ZPP_GeomVert.get(xlerp(x0,x1,y1,v3,v2,iso,quality),y1);
                                    ints.set(xn,(yn<<1)+2,p);
                                }
                                else p=ZPP_GeomVert.get(p.x,p.y);
                                if(sndy)p.forced=true;
                                if(p.x==x0||p.x==x1){
                                    if((p.x==x0&&(val&0x40)!=0)||(p.x==x1&&(val&0x10)!=0))val^=0x20;
                                }
                            }
                            else if(i==3){
                                p=ints.get(xn+1,(yn<<1)+1);
                                if(p==null){
                                    p=ZPP_GeomVert.get(x1,ylerp(y0,y1,x1,v1,v2,iso,quality));
                                    ints.set(xn+1,(yn<<1)+1,p);
                                }
                                else p=ZPP_GeomVert.get(p.x,p.y);
                                if(sndx)p.forced=true;
                                if(p.y==y0||p.y==y1){
                                    if((p.y==y0&&(val&4)!=0)||(p.y==y1&&(val&0x10)!=0))val^=0x08;
                                }
                            }
                            else{
                                p=ints.get(xn,(yn<<1)+1);
                                if(p==null){
                                    p=ZPP_GeomVert.get(x0,ylerp(y0,y1,x0,v0,v3,iso,quality));
                                    ints.set(xn,(yn<<1)+1,p);
                                }
                                else p=ZPP_GeomVert.get(p.x,p.y);
                                if(fstx)p.forced=true;
                                if(p.y==y0||p.y==y1){
                                    if((p.y==y0&&(val&1)!=0)||(p.y==y1&&(val&0x40)!=0))val^=0x80;
                                }
                            };
                            ret.p1={
                                var obj=p;
                                if(ret.p1==null)ret.p1=obj.prev=obj.next=obj;
                                else{
                                    obj.prev=ret.p1;
                                    obj.next=ret.p1.next;
                                    ret.p1.next.prev=obj;
                                    ret.p1.next=obj;
                                }
                                obj;
                            };
                        }
                    }
                    ret.p1=ret.p1.next;
                    ret.key1=val;
                    if(val==0x01||val==0x04||val==0x10||val==0x40||val==0x03||val==0x0c||val==0x30||val==0xc0||val==0x81||val==0x06||val==0x18||val==0x60||val==0x05||val==0x14||val==0x50||val==0x41||val==0x11||val==0x44){
                        val=0x00;
                        ret.key1=0;
                        ret.p1=null;
                    }
                };
                if(val==0)ret=null;
                else{
                    ret.pr=ret.p1;
                    ret.okeyr=ret.okey1;
                    ret.keyr=ret.key1;
                };
            }
            else{
                var mid=ISO(iso,0.5*(x0+x1),0.5*(y0+y1))<0;
                if(key==10){
                    if(mid){
                        var val=0xBB;
                        {
                            ret.okey1=val;
                            for(i in 0...8){
                                if((val&(1<<i))!=0){
                                    var p:ZPP_GeomVert=null;
                                    if(i==0){
                                        p=ZPP_GeomVert.get(x0,y0);
                                        if(fstx||fsty)p.forced=true;
                                    }
                                    else if(i==2){
                                        p=ZPP_GeomVert.get(x1,y0);
                                        if(sndx||fsty)p.forced=true;
                                    }
                                    else if(i==4){
                                        p=ZPP_GeomVert.get(x1,y1);
                                        if(sndx||sndy)p.forced=true;
                                    }
                                    else if(i==6){
                                        p=ZPP_GeomVert.get(x0,y1);
                                        if(fstx||sndy)p.forced=true;
                                    }
                                    else if(i==1){
                                        p=ints.get(xn,yn<<1);
                                        if(p==null){
                                            p=ZPP_GeomVert.get(xlerp(x0,x1,y0,v0,v1,iso,quality),y0);
                                            ints.set(xn,yn<<1,p);
                                        }
                                        else p=ZPP_GeomVert.get(p.x,p.y);
                                        if(fsty)p.forced=true;
                                        if(p.x==x0||p.x==x1){
                                            if((p.x==x0&&(val&1)!=0)||(p.x==x1&&(val&4)!=0))val^=0x02;
                                        }
                                    }
                                    else if(i==5){
                                        p=ints.get(xn,(yn<<1)+2);
                                        if(p==null){
                                            p=ZPP_GeomVert.get(xlerp(x0,x1,y1,v3,v2,iso,quality),y1);
                                            ints.set(xn,(yn<<1)+2,p);
                                        }
                                        else p=ZPP_GeomVert.get(p.x,p.y);
                                        if(sndy)p.forced=true;
                                        if(p.x==x0||p.x==x1){
                                            if((p.x==x0&&(val&0x40)!=0)||(p.x==x1&&(val&0x10)!=0))val^=0x20;
                                        }
                                    }
                                    else if(i==3){
                                        p=ints.get(xn+1,(yn<<1)+1);
                                        if(p==null){
                                            p=ZPP_GeomVert.get(x1,ylerp(y0,y1,x1,v1,v2,iso,quality));
                                            ints.set(xn+1,(yn<<1)+1,p);
                                        }
                                        else p=ZPP_GeomVert.get(p.x,p.y);
                                        if(sndx)p.forced=true;
                                        if(p.y==y0||p.y==y1){
                                            if((p.y==y0&&(val&4)!=0)||(p.y==y1&&(val&0x10)!=0))val^=0x08;
                                        }
                                    }
                                    else{
                                        p=ints.get(xn,(yn<<1)+1);
                                        if(p==null){
                                            p=ZPP_GeomVert.get(x0,ylerp(y0,y1,x0,v0,v3,iso,quality));
                                            ints.set(xn,(yn<<1)+1,p);
                                        }
                                        else p=ZPP_GeomVert.get(p.x,p.y);
                                        if(fstx)p.forced=true;
                                        if(p.y==y0||p.y==y1){
                                            if((p.y==y0&&(val&1)!=0)||(p.y==y1&&(val&0x40)!=0))val^=0x80;
                                        }
                                    };
                                    ret.p1={
                                        var obj=p;
                                        if(ret.p1==null)ret.p1=obj.prev=obj.next=obj;
                                        else{
                                            obj.prev=ret.p1;
                                            obj.next=ret.p1.next;
                                            ret.p1.next.prev=obj;
                                            ret.p1.next=obj;
                                        }
                                        obj;
                                    };
                                }
                            }
                            ret.p1=ret.p1.next;
                            ret.key1=val;
                            if(val==0x01||val==0x04||val==0x10||val==0x40||val==0x03||val==0x0c||val==0x30||val==0xc0||val==0x81||val==0x06||val==0x18||val==0x60||val==0x05||val==0x14||val==0x50||val==0x41||val==0x11||val==0x44){
                                val=0x00;
                                ret.key1=0;
                                ret.p1=null;
                            }
                        };
                        if(val==0)ret=null;
                        else{
                            ret.pr=ret.p1;
                            ret.okeyr=ret.okey1;
                            ret.keyr=ret.key1;
                        };
                    }
                    else{
                        var val=0x83;
                        {
                            ret.okey1=val;
                            for(i in 0...8){
                                if((val&(1<<i))!=0){
                                    var p:ZPP_GeomVert=null;
                                    if(i==0){
                                        p=ZPP_GeomVert.get(x0,y0);
                                        if(fstx||fsty)p.forced=true;
                                    }
                                    else if(i==2){
                                        p=ZPP_GeomVert.get(x1,y0);
                                        if(sndx||fsty)p.forced=true;
                                    }
                                    else if(i==4){
                                        p=ZPP_GeomVert.get(x1,y1);
                                        if(sndx||sndy)p.forced=true;
                                    }
                                    else if(i==6){
                                        p=ZPP_GeomVert.get(x0,y1);
                                        if(fstx||sndy)p.forced=true;
                                    }
                                    else if(i==1){
                                        p=ints.get(xn,yn<<1);
                                        if(p==null){
                                            p=ZPP_GeomVert.get(xlerp(x0,x1,y0,v0,v1,iso,quality),y0);
                                            ints.set(xn,yn<<1,p);
                                        }
                                        else p=ZPP_GeomVert.get(p.x,p.y);
                                        if(fsty)p.forced=true;
                                        if(p.x==x0||p.x==x1){
                                            if((p.x==x0&&(val&1)!=0)||(p.x==x1&&(val&4)!=0))val^=0x02;
                                        }
                                    }
                                    else if(i==5){
                                        p=ints.get(xn,(yn<<1)+2);
                                        if(p==null){
                                            p=ZPP_GeomVert.get(xlerp(x0,x1,y1,v3,v2,iso,quality),y1);
                                            ints.set(xn,(yn<<1)+2,p);
                                        }
                                        else p=ZPP_GeomVert.get(p.x,p.y);
                                        if(sndy)p.forced=true;
                                        if(p.x==x0||p.x==x1){
                                            if((p.x==x0&&(val&0x40)!=0)||(p.x==x1&&(val&0x10)!=0))val^=0x20;
                                        }
                                    }
                                    else if(i==3){
                                        p=ints.get(xn+1,(yn<<1)+1);
                                        if(p==null){
                                            p=ZPP_GeomVert.get(x1,ylerp(y0,y1,x1,v1,v2,iso,quality));
                                            ints.set(xn+1,(yn<<1)+1,p);
                                        }
                                        else p=ZPP_GeomVert.get(p.x,p.y);
                                        if(sndx)p.forced=true;
                                        if(p.y==y0||p.y==y1){
                                            if((p.y==y0&&(val&4)!=0)||(p.y==y1&&(val&0x10)!=0))val^=0x08;
                                        }
                                    }
                                    else{
                                        p=ints.get(xn,(yn<<1)+1);
                                        if(p==null){
                                            p=ZPP_GeomVert.get(x0,ylerp(y0,y1,x0,v0,v3,iso,quality));
                                            ints.set(xn,(yn<<1)+1,p);
                                        }
                                        else p=ZPP_GeomVert.get(p.x,p.y);
                                        if(fstx)p.forced=true;
                                        if(p.y==y0||p.y==y1){
                                            if((p.y==y0&&(val&1)!=0)||(p.y==y1&&(val&0x40)!=0))val^=0x80;
                                        }
                                    };
                                    ret.p1={
                                        var obj=p;
                                        if(ret.p1==null)ret.p1=obj.prev=obj.next=obj;
                                        else{
                                            obj.prev=ret.p1;
                                            obj.next=ret.p1.next;
                                            ret.p1.next.prev=obj;
                                            ret.p1.next=obj;
                                        }
                                        obj;
                                    };
                                }
                            }
                            ret.p1=ret.p1.next;
                            ret.key1=val;
                            if(val==0x01||val==0x04||val==0x10||val==0x40||val==0x03||val==0x0c||val==0x30||val==0xc0||val==0x81||val==0x06||val==0x18||val==0x60||val==0x05||val==0x14||val==0x50||val==0x41||val==0x11||val==0x44){
                                val=0x00;
                                ret.key1=0;
                                ret.p1=null;
                            }
                        };
                        if(val!=0){
                            var val=0x38;
                            {
                                ret.okey2=val;
                                for(i in 0...8){
                                    if((val&(1<<i))!=0){
                                        var p:ZPP_GeomVert=null;
                                        if(i==0){
                                            p=ZPP_GeomVert.get(x0,y0);
                                            if(fstx||fsty)p.forced=true;
                                        }
                                        else if(i==2){
                                            p=ZPP_GeomVert.get(x1,y0);
                                            if(sndx||fsty)p.forced=true;
                                        }
                                        else if(i==4){
                                            p=ZPP_GeomVert.get(x1,y1);
                                            if(sndx||sndy)p.forced=true;
                                        }
                                        else if(i==6){
                                            p=ZPP_GeomVert.get(x0,y1);
                                            if(fstx||sndy)p.forced=true;
                                        }
                                        else if(i==1){
                                            p=ints.get(xn,yn<<1);
                                            if(p==null){
                                                p=ZPP_GeomVert.get(xlerp(x0,x1,y0,v0,v1,iso,quality),y0);
                                                ints.set(xn,yn<<1,p);
                                            }
                                            else p=ZPP_GeomVert.get(p.x,p.y);
                                            if(fsty)p.forced=true;
                                            if(p.x==x0||p.x==x1){
                                                if((p.x==x0&&(val&1)!=0)||(p.x==x1&&(val&4)!=0))val^=0x02;
                                            }
                                        }
                                        else if(i==5){
                                            p=ints.get(xn,(yn<<1)+2);
                                            if(p==null){
                                                p=ZPP_GeomVert.get(xlerp(x0,x1,y1,v3,v2,iso,quality),y1);
                                                ints.set(xn,(yn<<1)+2,p);
                                            }
                                            else p=ZPP_GeomVert.get(p.x,p.y);
                                            if(sndy)p.forced=true;
                                            if(p.x==x0||p.x==x1){
                                                if((p.x==x0&&(val&0x40)!=0)||(p.x==x1&&(val&0x10)!=0))val^=0x20;
                                            }
                                        }
                                        else if(i==3){
                                            p=ints.get(xn+1,(yn<<1)+1);
                                            if(p==null){
                                                p=ZPP_GeomVert.get(x1,ylerp(y0,y1,x1,v1,v2,iso,quality));
                                                ints.set(xn+1,(yn<<1)+1,p);
                                            }
                                            else p=ZPP_GeomVert.get(p.x,p.y);
                                            if(sndx)p.forced=true;
                                            if(p.y==y0||p.y==y1){
                                                if((p.y==y0&&(val&4)!=0)||(p.y==y1&&(val&0x10)!=0))val^=0x08;
                                            }
                                        }
                                        else{
                                            p=ints.get(xn,(yn<<1)+1);
                                            if(p==null){
                                                p=ZPP_GeomVert.get(x0,ylerp(y0,y1,x0,v0,v3,iso,quality));
                                                ints.set(xn,(yn<<1)+1,p);
                                            }
                                            else p=ZPP_GeomVert.get(p.x,p.y);
                                            if(fstx)p.forced=true;
                                            if(p.y==y0||p.y==y1){
                                                if((p.y==y0&&(val&1)!=0)||(p.y==y1&&(val&0x40)!=0))val^=0x80;
                                            }
                                        };
                                        ret.p2={
                                            var obj=p;
                                            if(ret.p2==null)ret.p2=obj.prev=obj.next=obj;
                                            else{
                                                obj.prev=ret.p2;
                                                obj.next=ret.p2.next;
                                                ret.p2.next.prev=obj;
                                                ret.p2.next=obj;
                                            }
                                            obj;
                                        };
                                    }
                                }
                                ret.p2=ret.p2.next;
                                ret.key2=val;
                                if(val==0x01||val==0x04||val==0x10||val==0x40||val==0x03||val==0x0c||val==0x30||val==0xc0||val==0x81||val==0x06||val==0x18||val==0x60||val==0x05||val==0x14||val==0x50||val==0x41||val==0x11||val==0x44){
                                    val=0x00;
                                    ret.key2=0;
                                    ret.p2=null;
                                }
                            };
                            if(val==0){
                                ret.pr=ret.p1;
                                ret.okeyr=ret.okey1;
                                ret.keyr=ret.key1;
                            };
                            else{
                                ret.pr=ret.p2;
                                ret.okeyr=ret.okey2;
                                ret.keyr=ret.key2;
                            };
                        }
                        else{
                            var val=0x38;
                            {
                                ret.okey1=val;
                                for(i in 0...8){
                                    if((val&(1<<i))!=0){
                                        var p:ZPP_GeomVert=null;
                                        if(i==0){
                                            p=ZPP_GeomVert.get(x0,y0);
                                            if(fstx||fsty)p.forced=true;
                                        }
                                        else if(i==2){
                                            p=ZPP_GeomVert.get(x1,y0);
                                            if(sndx||fsty)p.forced=true;
                                        }
                                        else if(i==4){
                                            p=ZPP_GeomVert.get(x1,y1);
                                            if(sndx||sndy)p.forced=true;
                                        }
                                        else if(i==6){
                                            p=ZPP_GeomVert.get(x0,y1);
                                            if(fstx||sndy)p.forced=true;
                                        }
                                        else if(i==1){
                                            p=ints.get(xn,yn<<1);
                                            if(p==null){
                                                p=ZPP_GeomVert.get(xlerp(x0,x1,y0,v0,v1,iso,quality),y0);
                                                ints.set(xn,yn<<1,p);
                                            }
                                            else p=ZPP_GeomVert.get(p.x,p.y);
                                            if(fsty)p.forced=true;
                                            if(p.x==x0||p.x==x1){
                                                if((p.x==x0&&(val&1)!=0)||(p.x==x1&&(val&4)!=0))val^=0x02;
                                            }
                                        }
                                        else if(i==5){
                                            p=ints.get(xn,(yn<<1)+2);
                                            if(p==null){
                                                p=ZPP_GeomVert.get(xlerp(x0,x1,y1,v3,v2,iso,quality),y1);
                                                ints.set(xn,(yn<<1)+2,p);
                                            }
                                            else p=ZPP_GeomVert.get(p.x,p.y);
                                            if(sndy)p.forced=true;
                                            if(p.x==x0||p.x==x1){
                                                if((p.x==x0&&(val&0x40)!=0)||(p.x==x1&&(val&0x10)!=0))val^=0x20;
                                            }
                                        }
                                        else if(i==3){
                                            p=ints.get(xn+1,(yn<<1)+1);
                                            if(p==null){
                                                p=ZPP_GeomVert.get(x1,ylerp(y0,y1,x1,v1,v2,iso,quality));
                                                ints.set(xn+1,(yn<<1)+1,p);
                                            }
                                            else p=ZPP_GeomVert.get(p.x,p.y);
                                            if(sndx)p.forced=true;
                                            if(p.y==y0||p.y==y1){
                                                if((p.y==y0&&(val&4)!=0)||(p.y==y1&&(val&0x10)!=0))val^=0x08;
                                            }
                                        }
                                        else{
                                            p=ints.get(xn,(yn<<1)+1);
                                            if(p==null){
                                                p=ZPP_GeomVert.get(x0,ylerp(y0,y1,x0,v0,v3,iso,quality));
                                                ints.set(xn,(yn<<1)+1,p);
                                            }
                                            else p=ZPP_GeomVert.get(p.x,p.y);
                                            if(fstx)p.forced=true;
                                            if(p.y==y0||p.y==y1){
                                                if((p.y==y0&&(val&1)!=0)||(p.y==y1&&(val&0x40)!=0))val^=0x80;
                                            }
                                        };
                                        ret.p1={
                                            var obj=p;
                                            if(ret.p1==null)ret.p1=obj.prev=obj.next=obj;
                                            else{
                                                obj.prev=ret.p1;
                                                obj.next=ret.p1.next;
                                                ret.p1.next.prev=obj;
                                                ret.p1.next=obj;
                                            }
                                            obj;
                                        };
                                    }
                                }
                                ret.p1=ret.p1.next;
                                ret.key1=val;
                                if(val==0x01||val==0x04||val==0x10||val==0x40||val==0x03||val==0x0c||val==0x30||val==0xc0||val==0x81||val==0x06||val==0x18||val==0x60||val==0x05||val==0x14||val==0x50||val==0x41||val==0x11||val==0x44){
                                    val=0x00;
                                    ret.key1=0;
                                    ret.p1=null;
                                }
                            };
                            if(val==0)ret=null;
                            else{
                                ret.pr=ret.p1;
                                ret.okeyr=ret.okey1;
                                ret.keyr=ret.key1;
                            };
                        }
                    }
                }
                else{
                    if(mid){
                        var val=0xEE;
                        {
                            ret.okey1=val;
                            for(i in 0...8){
                                if((val&(1<<i))!=0){
                                    var p:ZPP_GeomVert=null;
                                    if(i==0){
                                        p=ZPP_GeomVert.get(x0,y0);
                                        if(fstx||fsty)p.forced=true;
                                    }
                                    else if(i==2){
                                        p=ZPP_GeomVert.get(x1,y0);
                                        if(sndx||fsty)p.forced=true;
                                    }
                                    else if(i==4){
                                        p=ZPP_GeomVert.get(x1,y1);
                                        if(sndx||sndy)p.forced=true;
                                    }
                                    else if(i==6){
                                        p=ZPP_GeomVert.get(x0,y1);
                                        if(fstx||sndy)p.forced=true;
                                    }
                                    else if(i==1){
                                        p=ints.get(xn,yn<<1);
                                        if(p==null){
                                            p=ZPP_GeomVert.get(xlerp(x0,x1,y0,v0,v1,iso,quality),y0);
                                            ints.set(xn,yn<<1,p);
                                        }
                                        else p=ZPP_GeomVert.get(p.x,p.y);
                                        if(fsty)p.forced=true;
                                        if(p.x==x0||p.x==x1){
                                            if((p.x==x0&&(val&1)!=0)||(p.x==x1&&(val&4)!=0))val^=0x02;
                                        }
                                    }
                                    else if(i==5){
                                        p=ints.get(xn,(yn<<1)+2);
                                        if(p==null){
                                            p=ZPP_GeomVert.get(xlerp(x0,x1,y1,v3,v2,iso,quality),y1);
                                            ints.set(xn,(yn<<1)+2,p);
                                        }
                                        else p=ZPP_GeomVert.get(p.x,p.y);
                                        if(sndy)p.forced=true;
                                        if(p.x==x0||p.x==x1){
                                            if((p.x==x0&&(val&0x40)!=0)||(p.x==x1&&(val&0x10)!=0))val^=0x20;
                                        }
                                    }
                                    else if(i==3){
                                        p=ints.get(xn+1,(yn<<1)+1);
                                        if(p==null){
                                            p=ZPP_GeomVert.get(x1,ylerp(y0,y1,x1,v1,v2,iso,quality));
                                            ints.set(xn+1,(yn<<1)+1,p);
                                        }
                                        else p=ZPP_GeomVert.get(p.x,p.y);
                                        if(sndx)p.forced=true;
                                        if(p.y==y0||p.y==y1){
                                            if((p.y==y0&&(val&4)!=0)||(p.y==y1&&(val&0x10)!=0))val^=0x08;
                                        }
                                    }
                                    else{
                                        p=ints.get(xn,(yn<<1)+1);
                                        if(p==null){
                                            p=ZPP_GeomVert.get(x0,ylerp(y0,y1,x0,v0,v3,iso,quality));
                                            ints.set(xn,(yn<<1)+1,p);
                                        }
                                        else p=ZPP_GeomVert.get(p.x,p.y);
                                        if(fstx)p.forced=true;
                                        if(p.y==y0||p.y==y1){
                                            if((p.y==y0&&(val&1)!=0)||(p.y==y1&&(val&0x40)!=0))val^=0x80;
                                        }
                                    };
                                    ret.p1={
                                        var obj=p;
                                        if(ret.p1==null)ret.p1=obj.prev=obj.next=obj;
                                        else{
                                            obj.prev=ret.p1;
                                            obj.next=ret.p1.next;
                                            ret.p1.next.prev=obj;
                                            ret.p1.next=obj;
                                        }
                                        obj;
                                    };
                                }
                            }
                            ret.p1=ret.p1.next;
                            ret.key1=val;
                            if(val==0x01||val==0x04||val==0x10||val==0x40||val==0x03||val==0x0c||val==0x30||val==0xc0||val==0x81||val==0x06||val==0x18||val==0x60||val==0x05||val==0x14||val==0x50||val==0x41||val==0x11||val==0x44){
                                val=0x00;
                                ret.key1=0;
                                ret.p1=null;
                            }
                        };
                        if(val==0)ret=null;
                        else{
                            ret.pr=ret.p1;
                            ret.okeyr=ret.okey1;
                            ret.keyr=ret.key1;
                        };
                    }
                    else{
                        var val=0xE0;
                        {
                            ret.okey1=val;
                            for(i in 0...8){
                                if((val&(1<<i))!=0){
                                    var p:ZPP_GeomVert=null;
                                    if(i==0){
                                        p=ZPP_GeomVert.get(x0,y0);
                                        if(fstx||fsty)p.forced=true;
                                    }
                                    else if(i==2){
                                        p=ZPP_GeomVert.get(x1,y0);
                                        if(sndx||fsty)p.forced=true;
                                    }
                                    else if(i==4){
                                        p=ZPP_GeomVert.get(x1,y1);
                                        if(sndx||sndy)p.forced=true;
                                    }
                                    else if(i==6){
                                        p=ZPP_GeomVert.get(x0,y1);
                                        if(fstx||sndy)p.forced=true;
                                    }
                                    else if(i==1){
                                        p=ints.get(xn,yn<<1);
                                        if(p==null){
                                            p=ZPP_GeomVert.get(xlerp(x0,x1,y0,v0,v1,iso,quality),y0);
                                            ints.set(xn,yn<<1,p);
                                        }
                                        else p=ZPP_GeomVert.get(p.x,p.y);
                                        if(fsty)p.forced=true;
                                        if(p.x==x0||p.x==x1){
                                            if((p.x==x0&&(val&1)!=0)||(p.x==x1&&(val&4)!=0))val^=0x02;
                                        }
                                    }
                                    else if(i==5){
                                        p=ints.get(xn,(yn<<1)+2);
                                        if(p==null){
                                            p=ZPP_GeomVert.get(xlerp(x0,x1,y1,v3,v2,iso,quality),y1);
                                            ints.set(xn,(yn<<1)+2,p);
                                        }
                                        else p=ZPP_GeomVert.get(p.x,p.y);
                                        if(sndy)p.forced=true;
                                        if(p.x==x0||p.x==x1){
                                            if((p.x==x0&&(val&0x40)!=0)||(p.x==x1&&(val&0x10)!=0))val^=0x20;
                                        }
                                    }
                                    else if(i==3){
                                        p=ints.get(xn+1,(yn<<1)+1);
                                        if(p==null){
                                            p=ZPP_GeomVert.get(x1,ylerp(y0,y1,x1,v1,v2,iso,quality));
                                            ints.set(xn+1,(yn<<1)+1,p);
                                        }
                                        else p=ZPP_GeomVert.get(p.x,p.y);
                                        if(sndx)p.forced=true;
                                        if(p.y==y0||p.y==y1){
                                            if((p.y==y0&&(val&4)!=0)||(p.y==y1&&(val&0x10)!=0))val^=0x08;
                                        }
                                    }
                                    else{
                                        p=ints.get(xn,(yn<<1)+1);
                                        if(p==null){
                                            p=ZPP_GeomVert.get(x0,ylerp(y0,y1,x0,v0,v3,iso,quality));
                                            ints.set(xn,(yn<<1)+1,p);
                                        }
                                        else p=ZPP_GeomVert.get(p.x,p.y);
                                        if(fstx)p.forced=true;
                                        if(p.y==y0||p.y==y1){
                                            if((p.y==y0&&(val&1)!=0)||(p.y==y1&&(val&0x40)!=0))val^=0x80;
                                        }
                                    };
                                    ret.p1={
                                        var obj=p;
                                        if(ret.p1==null)ret.p1=obj.prev=obj.next=obj;
                                        else{
                                            obj.prev=ret.p1;
                                            obj.next=ret.p1.next;
                                            ret.p1.next.prev=obj;
                                            ret.p1.next=obj;
                                        }
                                        obj;
                                    };
                                }
                            }
                            ret.p1=ret.p1.next;
                            ret.key1=val;
                            if(val==0x01||val==0x04||val==0x10||val==0x40||val==0x03||val==0x0c||val==0x30||val==0xc0||val==0x81||val==0x06||val==0x18||val==0x60||val==0x05||val==0x14||val==0x50||val==0x41||val==0x11||val==0x44){
                                val=0x00;
                                ret.key1=0;
                                ret.p1=null;
                            }
                        };
                        if(val!=0){
                            var val=0x0E;
                            {
                                ret.okey2=val;
                                for(i in 0...8){
                                    if((val&(1<<i))!=0){
                                        var p:ZPP_GeomVert=null;
                                        if(i==0){
                                            p=ZPP_GeomVert.get(x0,y0);
                                            if(fstx||fsty)p.forced=true;
                                        }
                                        else if(i==2){
                                            p=ZPP_GeomVert.get(x1,y0);
                                            if(sndx||fsty)p.forced=true;
                                        }
                                        else if(i==4){
                                            p=ZPP_GeomVert.get(x1,y1);
                                            if(sndx||sndy)p.forced=true;
                                        }
                                        else if(i==6){
                                            p=ZPP_GeomVert.get(x0,y1);
                                            if(fstx||sndy)p.forced=true;
                                        }
                                        else if(i==1){
                                            p=ints.get(xn,yn<<1);
                                            if(p==null){
                                                p=ZPP_GeomVert.get(xlerp(x0,x1,y0,v0,v1,iso,quality),y0);
                                                ints.set(xn,yn<<1,p);
                                            }
                                            else p=ZPP_GeomVert.get(p.x,p.y);
                                            if(fsty)p.forced=true;
                                            if(p.x==x0||p.x==x1){
                                                if((p.x==x0&&(val&1)!=0)||(p.x==x1&&(val&4)!=0))val^=0x02;
                                            }
                                        }
                                        else if(i==5){
                                            p=ints.get(xn,(yn<<1)+2);
                                            if(p==null){
                                                p=ZPP_GeomVert.get(xlerp(x0,x1,y1,v3,v2,iso,quality),y1);
                                                ints.set(xn,(yn<<1)+2,p);
                                            }
                                            else p=ZPP_GeomVert.get(p.x,p.y);
                                            if(sndy)p.forced=true;
                                            if(p.x==x0||p.x==x1){
                                                if((p.x==x0&&(val&0x40)!=0)||(p.x==x1&&(val&0x10)!=0))val^=0x20;
                                            }
                                        }
                                        else if(i==3){
                                            p=ints.get(xn+1,(yn<<1)+1);
                                            if(p==null){
                                                p=ZPP_GeomVert.get(x1,ylerp(y0,y1,x1,v1,v2,iso,quality));
                                                ints.set(xn+1,(yn<<1)+1,p);
                                            }
                                            else p=ZPP_GeomVert.get(p.x,p.y);
                                            if(sndx)p.forced=true;
                                            if(p.y==y0||p.y==y1){
                                                if((p.y==y0&&(val&4)!=0)||(p.y==y1&&(val&0x10)!=0))val^=0x08;
                                            }
                                        }
                                        else{
                                            p=ints.get(xn,(yn<<1)+1);
                                            if(p==null){
                                                p=ZPP_GeomVert.get(x0,ylerp(y0,y1,x0,v0,v3,iso,quality));
                                                ints.set(xn,(yn<<1)+1,p);
                                            }
                                            else p=ZPP_GeomVert.get(p.x,p.y);
                                            if(fstx)p.forced=true;
                                            if(p.y==y0||p.y==y1){
                                                if((p.y==y0&&(val&1)!=0)||(p.y==y1&&(val&0x40)!=0))val^=0x80;
                                            }
                                        };
                                        ret.p2={
                                            var obj=p;
                                            if(ret.p2==null)ret.p2=obj.prev=obj.next=obj;
                                            else{
                                                obj.prev=ret.p2;
                                                obj.next=ret.p2.next;
                                                ret.p2.next.prev=obj;
                                                ret.p2.next=obj;
                                            }
                                            obj;
                                        };
                                    }
                                }
                                ret.p2=ret.p2.next;
                                ret.key2=val;
                                if(val==0x01||val==0x04||val==0x10||val==0x40||val==0x03||val==0x0c||val==0x30||val==0xc0||val==0x81||val==0x06||val==0x18||val==0x60||val==0x05||val==0x14||val==0x50||val==0x41||val==0x11||val==0x44){
                                    val=0x00;
                                    ret.key2=0;
                                    ret.p2=null;
                                }
                            };
                            if(val==0){
                                ret.pr=ret.p1;
                                ret.okeyr=ret.okey1;
                                ret.keyr=ret.key1;
                            };
                            else{
                                ret.pr=ret.p2;
                                ret.okeyr=ret.okey2;
                                ret.keyr=ret.key2;
                            };
                        }
                        else{
                            var val=0x0E;
                            {
                                ret.okey1=val;
                                for(i in 0...8){
                                    if((val&(1<<i))!=0){
                                        var p:ZPP_GeomVert=null;
                                        if(i==0){
                                            p=ZPP_GeomVert.get(x0,y0);
                                            if(fstx||fsty)p.forced=true;
                                        }
                                        else if(i==2){
                                            p=ZPP_GeomVert.get(x1,y0);
                                            if(sndx||fsty)p.forced=true;
                                        }
                                        else if(i==4){
                                            p=ZPP_GeomVert.get(x1,y1);
                                            if(sndx||sndy)p.forced=true;
                                        }
                                        else if(i==6){
                                            p=ZPP_GeomVert.get(x0,y1);
                                            if(fstx||sndy)p.forced=true;
                                        }
                                        else if(i==1){
                                            p=ints.get(xn,yn<<1);
                                            if(p==null){
                                                p=ZPP_GeomVert.get(xlerp(x0,x1,y0,v0,v1,iso,quality),y0);
                                                ints.set(xn,yn<<1,p);
                                            }
                                            else p=ZPP_GeomVert.get(p.x,p.y);
                                            if(fsty)p.forced=true;
                                            if(p.x==x0||p.x==x1){
                                                if((p.x==x0&&(val&1)!=0)||(p.x==x1&&(val&4)!=0))val^=0x02;
                                            }
                                        }
                                        else if(i==5){
                                            p=ints.get(xn,(yn<<1)+2);
                                            if(p==null){
                                                p=ZPP_GeomVert.get(xlerp(x0,x1,y1,v3,v2,iso,quality),y1);
                                                ints.set(xn,(yn<<1)+2,p);
                                            }
                                            else p=ZPP_GeomVert.get(p.x,p.y);
                                            if(sndy)p.forced=true;
                                            if(p.x==x0||p.x==x1){
                                                if((p.x==x0&&(val&0x40)!=0)||(p.x==x1&&(val&0x10)!=0))val^=0x20;
                                            }
                                        }
                                        else if(i==3){
                                            p=ints.get(xn+1,(yn<<1)+1);
                                            if(p==null){
                                                p=ZPP_GeomVert.get(x1,ylerp(y0,y1,x1,v1,v2,iso,quality));
                                                ints.set(xn+1,(yn<<1)+1,p);
                                            }
                                            else p=ZPP_GeomVert.get(p.x,p.y);
                                            if(sndx)p.forced=true;
                                            if(p.y==y0||p.y==y1){
                                                if((p.y==y0&&(val&4)!=0)||(p.y==y1&&(val&0x10)!=0))val^=0x08;
                                            }
                                        }
                                        else{
                                            p=ints.get(xn,(yn<<1)+1);
                                            if(p==null){
                                                p=ZPP_GeomVert.get(x0,ylerp(y0,y1,x0,v0,v3,iso,quality));
                                                ints.set(xn,(yn<<1)+1,p);
                                            }
                                            else p=ZPP_GeomVert.get(p.x,p.y);
                                            if(fstx)p.forced=true;
                                            if(p.y==y0||p.y==y1){
                                                if((p.y==y0&&(val&1)!=0)||(p.y==y1&&(val&0x40)!=0))val^=0x80;
                                            }
                                        };
                                        ret.p1={
                                            var obj=p;
                                            if(ret.p1==null)ret.p1=obj.prev=obj.next=obj;
                                            else{
                                                obj.prev=ret.p1;
                                                obj.next=ret.p1.next;
                                                ret.p1.next.prev=obj;
                                                ret.p1.next=obj;
                                            }
                                            obj;
                                        };
                                    }
                                }
                                ret.p1=ret.p1.next;
                                ret.key1=val;
                                if(val==0x01||val==0x04||val==0x10||val==0x40||val==0x03||val==0x0c||val==0x30||val==0xc0||val==0x81||val==0x06||val==0x18||val==0x60||val==0x05||val==0x14||val==0x50||val==0x41||val==0x11||val==0x44){
                                    val=0x00;
                                    ret.key1=0;
                                    ret.p1=null;
                                }
                            };
                            if(val==0)ret=null;
                            else{
                                ret.pr=ret.p1;
                                ret.okeyr=ret.okey1;
                                ret.keyr=ret.key1;
                            };
                        }
                    }
                }
            }
            return ret;
        }
    }
    #if NAPE_NO_INLINE#else inline #end
    function lerp(x0:Float,x1:Float,v0:Float,v1:Float){
        if(v0==0)return x0;
        else if(v1==0)return x1;
        else{
            var dv=v0-v1;
            var t=if(dv*dv<(Config.epsilon*Config.epsilon))0.5 else v0/dv;
            if(t<0)t=0;
            else if(t>1)t=1;
            return x0+t*(x1-x0);
        }
    }
    function xlerp(x0:Float,x1:Float,y:Float,v0:Float,v1:Float,iso:IsoFunctionDef,quality:Int){
        var xm=lerp(x0,x1,v0,v1);
        while(quality--!=0&&x0<xm&&xm<x1){
            var vm:Float=ISO(iso,xm,y);
            if(vm==0)break;
            if(v0*vm<0){
                x1=xm;
                v1=vm;
            }
            else{
                x0=xm;
                v0=vm;
            }
            xm=lerp(x0,x1,v0,v1);
        }
        return xm;
    }
    function ylerp(y0:Float,y1:Float,x:Float,v0:Float,v1:Float,iso:IsoFunctionDef,quality:Int){
        var ym=lerp(y0,y1,v0,v1);
        while(quality--!=0&&y0<ym&&ym<y1){
            var vm:Float=ISO(iso,x,ym);
            if(vm==0)break;
            if(v0*vm<0){
                y1=ym;
                v1=vm;
            }
            else{
                y0=ym;
                v0=vm;
            }
            ym=lerp(y0,y1,v0,v1);
        }
        return ym;
    }
    #if NAPE_NO_INLINE#else inline #end
    static function ISO(iso:IsoFunctionDef,x:Float,y:Float){
        return#if flash iso.iso(x,y)#else iso(x,y)#end;
    }
}

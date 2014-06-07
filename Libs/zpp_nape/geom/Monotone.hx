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
class ZPP_Monotone{
    static function bisector(b:ZPP_PartitionVertex){
        var a=b.prev;
        var c=b.next;
        var ux:Float=0.0;
        var uy:Float=0.0;
        {
            ux=b.x-a.x;
            uy=b.y-a.y;
        };
        var vx:Float=0.0;
        var vy:Float=0.0;
        {
            vx=c.x-b.x;
            vy=c.y-b.y;
        };
        var ret=ZPP_Vec2.get(-uy-vy,ux+vx);
        {
            var d=(ret.x*ret.x+ret.y*ret.y);
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    d!=0.0;
                };
                if(!res)throw "assert("+"d!=0.0"+") :: "+("vec_normalise(in n: "+"ret."+")");
                #end
            };
            var imag=ZPP_Math.invsqrt(d);
            {
                var t=(imag);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"ret."+",in s: "+"imag"+")");
                    #end
                };
                ret.x*=t;
                ret.y*=t;
            };
        };
        if((vy*ux-vx*uy)<0){
            ret.x=-ret.x;
            ret.y=-ret.y;
        };
        return ret;
    }
    static function below(p:ZPP_PartitionVertex,q:ZPP_PartitionVertex){
        if(p.y<q.y)return true;
        else if(p.y>q.y)return false;
        else{
            if(p.x<q.x)return true;
            else if(p.x>q.x)return false;
            else{
                var po=bisector(p);
                var qo=bisector(q);
                {
                    var t=(1.0);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"po."+",in b: "+"p."+",in s: "+"1.0"+")");
                        #end
                    };
                    po.x+=p.x*t;
                    po.y+=p.y*t;
                };
                {
                    var t=(1.0);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"qo."+",in b: "+"q."+",in s: "+"1.0"+")");
                        #end
                    };
                    qo.x+=q.x*t;
                    qo.y+=q.y*t;
                };
                var ret=po.x<qo.x||(po.x==qo.x&&po.y<qo.y);
                {
                    var o=po;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            o!=null;
                        };
                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Vec2"+", in obj: "+"po"+")");
                        #end
                    };
                    o.free();
                    o.next=ZPP_Vec2.zpp_pool;
                    ZPP_Vec2.zpp_pool=o;
                    #if NAPE_POOL_STATS ZPP_Vec2.POOL_CNT++;
                    ZPP_Vec2.POOL_SUB++;
                    #end
                };
                {
                    var o=qo;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            o!=null;
                        };
                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Vec2"+", in obj: "+"qo"+")");
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
        }
    }
    static function above(p:ZPP_PartitionVertex,q:ZPP_PartitionVertex){
        return below(q,p);
    }
    static function left_vertex(p:ZPP_PartitionVertex){
        var pre=p.prev;
        return pre.y>p.y||(pre.y==p.y&&p.next.y<p.y);
    }
    public static function isMonotone(P:ZPP_GeomVert){
        var min=P;
        var max=P;
        {
            var F=P.next;
            var L=P;
            if(F!=null){
                var nite=F;
                do{
                    var p=nite;
                    {
                        {
                            if(p.y<min.y)min=p;
                            if(p.y>max.y)max=p;
                        };
                    }
                    nite=nite.next;
                }
                while(nite!=L);
            }
        };
        var ret=true;
        var pre=min;
        if(max!=min.next){
            {
                var F=min.next;
                var L=max;
                if(F!=null){
                    var nite=F;
                    do{
                        var p=nite;
                        {
                            {
                                if(p.y<pre.y){
                                    ret=false;
                                    break;
                                }
                                pre=p;
                            };
                        }
                        nite=nite.next;
                    }
                    while(nite!=L);
                }
            };
        }
        if(!ret)return false;
        pre=min;
        if(max!=min.prev){
            {
                var F=min.prev;
                var L=max;
                if(F!=null){
                    var nite=F;
                    do{
                        var p=nite;
                        {
                            {
                                if(p.y<pre.y){
                                    ret=false;
                                    break;
                                }
                                pre=p;
                            };
                        }
                        nite=nite.prev;
                    }
                    while(nite!=L);
                }
            };
        }
        return ret;
    }
    public static var sharedPPoly:ZPP_PartitionedPoly;
    public static#if NAPE_NO_INLINE#else inline #end
    function getShared(){
        if(sharedPPoly==null)sharedPPoly=new ZPP_PartitionedPoly();
        return sharedPPoly;
    }
    static var queue:ZNPList_ZPP_PartitionVertex=null;
    static var edges:ZPP_Set_ZPP_PartitionVertex=null;
    public static function decompose(P:ZPP_GeomVert,?poly:ZPP_PartitionedPoly){
        if(poly==null){
            poly=new ZPP_PartitionedPoly(P);
        }
        else{
            poly.init(P);
        }
        if(poly.vertices==null)return poly;
        if(queue==null)queue=new ZNPList_ZPP_PartitionVertex();
        {
            var F=poly.vertices;
            var L=poly.vertices;
            if(F!=null){
                var nite=F;
                do{
                    var p=nite;
                    {
                        {
                            queue.add(p);
                            var ux:Float=0.0;
                            var uy:Float=0.0;
                            {
                                ux=p.next.x-p.x;
                                uy=p.next.y-p.y;
                            };
                            var vx:Float=0.0;
                            var vy:Float=0.0;
                            {
                                vx=p.prev.x-p.x;
                                vy=p.prev.y-p.y;
                            };
                            var cx=(vy*ux-vx*uy)>0.0;
                            p.type=if(below(p.prev,p)){
                                if(below(p.next,p))cx?0:3 else 4;
                            }
                            else{
                                if(below(p,p.next))cx?1:2 else 4;
                            };
                        };
                    }
                    nite=nite.next;
                }
                while(nite!=L);
            }
        };
        {
            var xxlist=queue;
            if(!xxlist.empty()&&xxlist.begin().next!=null){
                var head:ZNPNode_ZPP_PartitionVertex=xxlist.begin();
                var tail:ZNPNode_ZPP_PartitionVertex=null;
                var left:ZNPNode_ZPP_PartitionVertex=null;
                var right:ZNPNode_ZPP_PartitionVertex=null;
                var nxt:ZNPNode_ZPP_PartitionVertex=null;
                var listSize=1;
                var numMerges:Int,leftSize:Int,rightSize:Int;
                do{
                    numMerges=0;
                    left=head;
                    tail=head=null;
                    while(left!=null){
                        numMerges++;
                        right=left;
                        leftSize=0;
                        rightSize=listSize;
                        while(right!=null&&leftSize<listSize){
                            leftSize++;
                            right=right.next;
                        }
                        while(leftSize>0||(rightSize>0&&right!=null)){
                            if(leftSize==0){
                                nxt=right;
                                right=right.next;
                                rightSize--;
                            }
                            else if(rightSize==0||right==null){
                                nxt=left;
                                left=left.next;
                                leftSize--;
                            }
                            else if(above(left.elem(),right.elem())){
                                nxt=left;
                                left=left.next;
                                leftSize--;
                            }
                            else{
                                nxt=right;
                                right=right.next;
                                rightSize--;
                            }
                            if(tail!=null)tail.next=nxt;
                            else head=nxt;
                            tail=nxt;
                        }
                        left=right;
                    }
                    tail.next=null;
                    listSize<<=1;
                }
                while(numMerges>1);
                xxlist.setbegin(head);
            }
        };
        if(edges==null){
            {
                if(ZPP_Set_ZPP_PartitionVertex.zpp_pool==null){
                    edges=new ZPP_Set_ZPP_PartitionVertex();
                    #if NAPE_POOL_STATS ZPP_Set_ZPP_PartitionVertex.POOL_TOT++;
                    ZPP_Set_ZPP_PartitionVertex.POOL_ADDNEW++;
                    #end
                }
                else{
                    edges=ZPP_Set_ZPP_PartitionVertex.zpp_pool;
                    ZPP_Set_ZPP_PartitionVertex.zpp_pool=edges.next;
                    edges.next=null;
                    #if NAPE_POOL_STATS ZPP_Set_ZPP_PartitionVertex.POOL_CNT--;
                    ZPP_Set_ZPP_PartitionVertex.POOL_ADD++;
                    #end
                }
                edges.alloc();
            };
            edges.lt=ZPP_PartitionVertex.edge_lt;
            edges.swapped=ZPP_PartitionVertex.edge_swap;
        }
        while(!queue.empty()){
            var v=queue.pop_unsafe();
            switch(v.type){
                case 0:v.helper=v;
                v.node=edges.insert(v);
                case 1:var e=v.prev;
                #if(!NAPE_RELEASE_BUILD)
                if(e.helper==null)throw "Fatal error (1): Polygon is not weakly-simple and clockwise";
                #end
                if(e.helper.type==2){
                    poly.add_diagonal(v,e.helper);
                }
                edges.remove_node(e.node);
                #if(!NAPE_RELEASE_BUILD)
                e.helper=null;
                #end
                case 3:var e={
                    var ret=null;
                    {
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                edges!=null;
                            };
                            if(!res)throw "assert("+"edges!=null"+") :: "+("Iterate  null set?");
                            #end
                        };
                        if(!edges.empty()){
                            var set_ite=edges.parent;
                            while(set_ite.prev!=null)set_ite=set_ite.prev;
                            while(set_ite!=null){
                                var elt=set_ite.data;
                                {
                                    if(!ZPP_PartitionVertex.vert_lt(elt,v)){
                                        ret=elt;
                                        break;
                                    }
                                };
                                if(set_ite.next!=null){
                                    set_ite=set_ite.next;
                                    while(set_ite.prev!=null)set_ite=set_ite.prev;
                                }
                                else{
                                    while(set_ite.parent!=null&&set_ite==set_ite.parent.next)set_ite=set_ite.parent;
                                    set_ite=set_ite.parent;
                                }
                            }
                        }
                    };
                    ret;
                };
                if(e!=null){
                    #if(!NAPE_RELEASE_BUILD)
                    if(e.helper==null)throw "Fatal error (2): Polygon is not weakly-simple and clockwise";
                    #end
                    poly.add_diagonal(v,e.helper);
                    e.helper=v;
                }
                v.node=edges.insert(v);
                v.helper=v;
                case 2:var e=v.prev;
                #if(!NAPE_RELEASE_BUILD)
                if(e.helper==null)throw "Fatal error (3): Polygon is not weakly-simple and clockwise";
                #end
                if(e.helper.type==2){
                    poly.add_diagonal(v,e.helper);
                }
                edges.remove_node(e.node);
                #if(!NAPE_RELEASE_BUILD)
                e.helper=null;
                #end
                var e={
                    var ret=null;
                    {
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                edges!=null;
                            };
                            if(!res)throw "assert("+"edges!=null"+") :: "+("Iterate  null set?");
                            #end
                        };
                        if(!edges.empty()){
                            var set_ite=edges.parent;
                            while(set_ite.prev!=null)set_ite=set_ite.prev;
                            while(set_ite!=null){
                                var elt=set_ite.data;
                                {
                                    if(!ZPP_PartitionVertex.vert_lt(elt,v)){
                                        ret=elt;
                                        break;
                                    }
                                };
                                if(set_ite.next!=null){
                                    set_ite=set_ite.next;
                                    while(set_ite.prev!=null)set_ite=set_ite.prev;
                                }
                                else{
                                    while(set_ite.parent!=null&&set_ite==set_ite.parent.next)set_ite=set_ite.parent;
                                    set_ite=set_ite.parent;
                                }
                            }
                        }
                    };
                    ret;
                };
                if(e!=null){
                    #if(!NAPE_RELEASE_BUILD)
                    if(e.helper==null)throw "Fatal error (4): Polygon is not weakly-simple and clockwise";
                    #end
                    if(e.helper.type==2){
                        poly.add_diagonal(v,e.helper);
                    }
                    e.helper=v;
                }
                case 4:var pre=v.prev;
                if(left_vertex(v)){
                    #if(!NAPE_RELEASE_BUILD)
                    if(pre.helper==null)throw "Fatal error (5): Polygon is not weakly-simple and clockwise";
                    #end
                    if(pre.helper.type==2)poly.add_diagonal(v,pre.helper);
                    edges.remove_node(pre.node);
                    #if(!NAPE_RELEASE_BUILD)
                    pre.helper=null;
                    #end
                    v.node=edges.insert(v);
                    v.helper=v;
                }
                else{
                    var e={
                        var ret=null;
                        {
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    edges!=null;
                                };
                                if(!res)throw "assert("+"edges!=null"+") :: "+("Iterate  null set?");
                                #end
                            };
                            if(!edges.empty()){
                                var set_ite=edges.parent;
                                while(set_ite.prev!=null)set_ite=set_ite.prev;
                                while(set_ite!=null){
                                    var elt=set_ite.data;
                                    {
                                        if(!ZPP_PartitionVertex.vert_lt(elt,v)){
                                            ret=elt;
                                            break;
                                        }
                                    };
                                    if(set_ite.next!=null){
                                        set_ite=set_ite.next;
                                        while(set_ite.prev!=null)set_ite=set_ite.prev;
                                    }
                                    else{
                                        while(set_ite.parent!=null&&set_ite==set_ite.parent.next)set_ite=set_ite.parent;
                                        set_ite=set_ite.parent;
                                    }
                                }
                            }
                        };
                        ret;
                    };
                    #if(!NAPE_RELEASE_BUILD)
                    if(e==null||e.helper==null)throw "Fatal error (6): Polygon is not weakly-simple and clockwise";
                    #end
                    if(e.helper.type==2){
                        poly.add_diagonal(v,e.helper);
                    }
                    e.helper=v;
                }
            }
        }
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                edges.empty();
            };
            if(!res)throw "assert("+"edges.empty()"+") :: "+("non-empty edges after monotone");
            #end
        };
        return poly;
    }
}

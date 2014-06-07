package zpp_nape.shape;
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
class ZPP_Polygon extends ZPP_Shape{
    public var outer_zn:Polygon=null;
    public var lverts:ZPP_Vec2=null;
    public var wrap_lverts:Vec2List=null;
    public var gverts:ZPP_Vec2=null;
    public var wrap_gverts:Vec2List=null;
    public var edges:ZNPList_ZPP_Edge=null;
    public var wrap_edges:EdgeList=null;
    public var edgeCnt:Int=0;
    public var reverse_flag:Bool=false;
    public function __clear(){}
    private function lverts_pa_invalidate(x:ZPP_Vec2){
        invalidate_lverts();
    }
    #if(!NAPE_RELEASE_BUILD)
    private function lverts_pa_immutable(){
        if(body!=null&&body.isStatic()&&body.space!=null)throw "Error: Cannot modify local vertex of Polygon added to a static body whilst within a Space";
    }
    #end
    private function gverts_pa_validate(){
        #if(!NAPE_RELEASE_BUILD)
        if(body==null)throw "Error: World vertex only makes sense when Polygon is contained in a rigid body";
        #end
        validate_gverts();
    }
    private function lverts_post_adder(x:Vec2){
        x.zpp_inner._invalidate=lverts_pa_invalidate;
        #if(!NAPE_RELEASE_BUILD)
        x.zpp_inner._isimmutable=lverts_pa_immutable;
        #end
        var ite:ZPP_Vec2=null;
        var ite2:ZNPNode_ZPP_Edge=null;
        {
            var cx_ite=lverts.begin();
            while(cx_ite!=null){
                var i=cx_ite.elem();
                {
                    if(i==x.zpp_inner)break;
                    else{
                        ite=ite==null?gverts.begin():ite.next;
                        ite2=ite2==null?edges.begin():ite2.next;
                    }
                };
                cx_ite=cx_ite.next;
            }
        };
        var vec=ZPP_Vec2.get(0,0,true);
        gverts.insert(ite,vec);
        if(lverts.begin().next!=null){
            if(lverts.begin().next.next==null){
                var ed:ZPP_Edge;
                {
                    if(ZPP_Edge.zpp_pool==null){
                        ed=new ZPP_Edge();
                        #if NAPE_POOL_STATS ZPP_Edge.POOL_TOT++;
                        ZPP_Edge.POOL_ADDNEW++;
                        #end
                    }
                    else{
                        ed=ZPP_Edge.zpp_pool;
                        ZPP_Edge.zpp_pool=ed.next;
                        ed.next=null;
                        #if NAPE_POOL_STATS ZPP_Edge.POOL_CNT--;
                        ZPP_Edge.POOL_ADD++;
                        #end
                    }
                    ed.alloc();
                };
                ed.polygon=this;
                edges.add(ed);
                var ed:ZPP_Edge;
                {
                    if(ZPP_Edge.zpp_pool==null){
                        ed=new ZPP_Edge();
                        #if NAPE_POOL_STATS ZPP_Edge.POOL_TOT++;
                        ZPP_Edge.POOL_ADDNEW++;
                        #end
                    }
                    else{
                        ed=ZPP_Edge.zpp_pool;
                        ZPP_Edge.zpp_pool=ed.next;
                        ed.next=null;
                        #if NAPE_POOL_STATS ZPP_Edge.POOL_CNT--;
                        ZPP_Edge.POOL_ADD++;
                        #end
                    }
                    ed.alloc();
                };
                ed.polygon=this;
                edges.add(ed);
                edgeCnt+=2;
            }
            else{
                var ed:ZPP_Edge;
                {
                    if(ZPP_Edge.zpp_pool==null){
                        ed=new ZPP_Edge();
                        #if NAPE_POOL_STATS ZPP_Edge.POOL_TOT++;
                        ZPP_Edge.POOL_ADDNEW++;
                        #end
                    }
                    else{
                        ed=ZPP_Edge.zpp_pool;
                        ZPP_Edge.zpp_pool=ed.next;
                        ed.next=null;
                        #if NAPE_POOL_STATS ZPP_Edge.POOL_CNT--;
                        ZPP_Edge.POOL_ADD++;
                        #end
                    }
                    ed.alloc();
                };
                ed.polygon=this;
                edges.insert(ite2,ed);
                edgeCnt++;
            }
        }
        vec._validate=gverts_pa_validate;
    }
    private function lverts_subber(x:Vec2){
        cleanup_lvert(x.zpp_inner);
    }
    private function lverts_invalidate(_){
        invalidate_lverts();
    }
    private function lverts_validate(){
        validate_lverts();
    }
    #if(!NAPE_RELEASE_BUILD)
    private function lverts_modifiable(){
        immutable_midstep("Polygon::localVerts");
        #if(!NAPE_RELEASE_BUILD)
        if(body!=null&&body.isStatic()&&body.space!=null)throw "Error: Cannot modifiy shapes of static object once added to Space";
        #end
    }
    #end
    private function gverts_validate(){
        validate_gverts();
    }
    private function edges_validate(){
        validate_lverts();
    }
    #if nape_swc@:keep #end
    public function getlverts(){
        var me=this;
        wrap_lverts=ZPP_MixVec2List.get(lverts);
        wrap_lverts.zpp_inner.post_adder=lverts_post_adder;
        wrap_lverts.zpp_inner.subber=lverts_subber;
        wrap_lverts.zpp_inner._invalidate=lverts_invalidate;
        wrap_lverts.zpp_inner._validate=lverts_validate;
        #if(!NAPE_RELEASE_BUILD)
        wrap_lverts.zpp_inner._modifiable=lverts_modifiable;
        #end
        wrap_lverts.zpp_inner.reverse_flag=reverse_flag;
    }
    public function getgverts(){
        var me=this;
        wrap_gverts=ZPP_MixVec2List.get(gverts,true);
        wrap_gverts.zpp_inner.reverse_flag=reverse_flag;
        wrap_gverts.zpp_inner._validate=gverts_validate;
    }
    public function getedges(){
        var me=this;
        wrap_edges=ZPP_EdgeList.get(edges,true);
        wrap_edges.zpp_inner.reverse_flag=reverse_flag;
        wrap_edges.zpp_inner._validate=edges_validate;
    }
    public var zip_lverts:Bool=false;
    public function invalidate_lverts(){
        invalidate_laxi();
        invalidate_area_inertia();
        invalidate_angDrag();
        invalidate_localCOM();
        invalidate_gverts();
        zip_lverts=true;
        zip_valid=true;
        zip_sanitation=true;
        if(body!=null)body.wake();
    }
    public var zip_laxi:Bool=false;
    public function invalidate_laxi(){
        invalidate_gaxi();
        invalidate_sweepRadius();
        zip_laxi=true;
    }
    public var zip_gverts:Bool=false;
    public function invalidate_gverts(){
        invalidate_aabb();
        zip_gverts=true;
    }
    public var zip_gaxi:Bool=false;
    public function invalidate_gaxi(){
        zip_gaxi=true;
    }
    public var zip_valid:Bool=false;
    public var validation:ValidationResult;
    public function valid(){
        if(zip_valid){
            zip_valid=false;
            splice_collinear();
            if(lverts.size()<3)return validation=ValidationResult.DEGENERATE;
            else{
                validate_lverts();
                validate_area_inertia();
                if(area<Config.epsilon)return validation=ValidationResult.DEGENERATE;
                else{
                    var neg=false;
                    var pos=false;
                    {
                        var cx_cont=true;
                        var cx_ite=lverts.begin();
                        var u=cx_ite.elem();
                        cx_ite=cx_ite.next;
                        var v=cx_ite.elem();
                        cx_ite=cx_ite.next;
                        while(cx_ite!=null){
                            var w=cx_ite.elem();
                            {
                                var ax:Float=0.0;
                                var ay:Float=0.0;
                                {
                                    ax=w.x-v.x;
                                    ay=w.y-v.y;
                                };
                                var bx:Float=0.0;
                                var by:Float=0.0;
                                {
                                    bx=v.x-u.x;
                                    by=v.y-u.y;
                                };
                                var dot=(by*ax-bx*ay);
                                if(dot>Config.epsilon)pos=true;
                                else if(dot<-Config.epsilon)neg=true;
                                if(pos&&neg){
                                    cx_cont=false;
                                    break;
                                };
                            };
                            {
                                u=v;
                                v=w;
                                cx_ite=cx_ite.next;
                            };
                        }
                        if(cx_cont){
                            cx_ite=lverts.begin();
                            var w=cx_ite.elem();
                            do{
                                {
                                    var ax:Float=0.0;
                                    var ay:Float=0.0;
                                    {
                                        ax=w.x-v.x;
                                        ay=w.y-v.y;
                                    };
                                    var bx:Float=0.0;
                                    var by:Float=0.0;
                                    {
                                        bx=v.x-u.x;
                                        by=v.y-u.y;
                                    };
                                    var dot=(by*ax-bx*ay);
                                    if(dot>Config.epsilon)pos=true;
                                    else if(dot<-Config.epsilon)neg=true;
                                    if(pos&&neg){
                                        cx_cont=false;
                                        break;
                                    };
                                };
                            }
                            while(false);
                            if(cx_cont){
                                {
                                    u=v;
                                    v=w;
                                    cx_ite=cx_ite.next;
                                };
                                var w=cx_ite.elem();
                                do{
                                    {
                                        var ax:Float=0.0;
                                        var ay:Float=0.0;
                                        {
                                            ax=w.x-v.x;
                                            ay=w.y-v.y;
                                        };
                                        var bx:Float=0.0;
                                        var by:Float=0.0;
                                        {
                                            bx=v.x-u.x;
                                            by=v.y-u.y;
                                        };
                                        var dot=(by*ax-bx*ay);
                                        if(dot>Config.epsilon)pos=true;
                                        else if(dot<-Config.epsilon)neg=true;
                                        if(pos&&neg)break;
                                    };
                                }
                                while(false);
                            }
                        }
                    };
                    if(pos&&neg)return validation=ValidationResult.CONCAVE;
                    else{
                        var cont=true;
                        {
                            var cx_cont=true;
                            var cx_ite=lverts.begin();
                            var u=cx_ite.elem();
                            cx_ite=cx_ite.next;
                            while(cx_ite!=null){
                                var v=cx_ite.elem();
                                {
                                    if(!cont){
                                        cx_cont=false;
                                        break;
                                    };
                                    {
                                        var cx_cont=true;
                                        var cx_ite=lverts.begin();
                                        var a=cx_ite.elem();
                                        cx_ite=cx_ite.next;
                                        while(cx_ite!=null){
                                            var b=cx_ite.elem();
                                            {
                                                if(u==a||u==b||v==a||v==b){
                                                    {
                                                        a=b;
                                                        cx_ite=cx_ite.next;
                                                    };
                                                    continue;
                                                };
                                                var sx:Float=0.0;
                                                var sy:Float=0.0;
                                                {
                                                    sx=u.x-a.x;
                                                    sy=u.y-a.y;
                                                };
                                                var vx:Float=0.0;
                                                var vy:Float=0.0;
                                                {
                                                    vx=v.x-u.x;
                                                    vy=v.y-u.y;
                                                };
                                                var qx:Float=0.0;
                                                var qy:Float=0.0;
                                                {
                                                    qx=b.x-a.x;
                                                    qy=b.y-a.y;
                                                };
                                                var den=(vy*qx-vx*qy);
                                                if(den*den>Config.epsilon){
                                                    den=1/den;
                                                    var t=(qy*sx-qx*sy)*den;
                                                    if(t>Config.epsilon&&t<1-Config.epsilon){
                                                        var s=(vy*sx-vx*sy)*den;
                                                        if(s>Config.epsilon&&s<1-Config.epsilon){
                                                            cont=false;
                                                            {
                                                                cx_cont=false;
                                                                break;
                                                            };
                                                        }
                                                    }
                                                }
                                            };
                                            {
                                                a=b;
                                                cx_ite=cx_ite.next;
                                            };
                                        }
                                        if(cx_cont){
                                            do{
                                                var b=lverts.front();
                                                {
                                                    if(u==a||u==b||v==a||v==b)break;
                                                    var sx:Float=0.0;
                                                    var sy:Float=0.0;
                                                    {
                                                        sx=u.x-a.x;
                                                        sy=u.y-a.y;
                                                    };
                                                    var vx:Float=0.0;
                                                    var vy:Float=0.0;
                                                    {
                                                        vx=v.x-u.x;
                                                        vy=v.y-u.y;
                                                    };
                                                    var qx:Float=0.0;
                                                    var qy:Float=0.0;
                                                    {
                                                        qx=b.x-a.x;
                                                        qy=b.y-a.y;
                                                    };
                                                    var den=(vy*qx-vx*qy);
                                                    if(den*den>Config.epsilon){
                                                        den=1/den;
                                                        var t=(qy*sx-qx*sy)*den;
                                                        if(t>Config.epsilon&&t<1-Config.epsilon){
                                                            var s=(vy*sx-vx*sy)*den;
                                                            if(s>Config.epsilon&&s<1-Config.epsilon){
                                                                cont=false;
                                                                break;
                                                            }
                                                        }
                                                    }
                                                };
                                            }
                                            while(false);
                                        }
                                    };
                                };
                                {
                                    u=v;
                                    cx_ite=cx_ite.next;
                                };
                            }
                            if(cx_cont){
                                do{
                                    var v=lverts.front();
                                    {
                                        if(!cont)break;
                                        {
                                            var cx_cont=true;
                                            var cx_ite=lverts.begin();
                                            var a=cx_ite.elem();
                                            cx_ite=cx_ite.next;
                                            while(cx_ite!=null){
                                                var b=cx_ite.elem();
                                                {
                                                    if(u==a||u==b||v==a||v==b){
                                                        {
                                                            a=b;
                                                            cx_ite=cx_ite.next;
                                                        };
                                                        continue;
                                                    };
                                                    var sx:Float=0.0;
                                                    var sy:Float=0.0;
                                                    {
                                                        sx=u.x-a.x;
                                                        sy=u.y-a.y;
                                                    };
                                                    var vx:Float=0.0;
                                                    var vy:Float=0.0;
                                                    {
                                                        vx=v.x-u.x;
                                                        vy=v.y-u.y;
                                                    };
                                                    var qx:Float=0.0;
                                                    var qy:Float=0.0;
                                                    {
                                                        qx=b.x-a.x;
                                                        qy=b.y-a.y;
                                                    };
                                                    var den=(vy*qx-vx*qy);
                                                    if(den*den>Config.epsilon){
                                                        den=1/den;
                                                        var t=(qy*sx-qx*sy)*den;
                                                        if(t>Config.epsilon&&t<1-Config.epsilon){
                                                            var s=(vy*sx-vx*sy)*den;
                                                            if(s>Config.epsilon&&s<1-Config.epsilon){
                                                                cont=false;
                                                                {
                                                                    cx_cont=false;
                                                                    break;
                                                                };
                                                            }
                                                        }
                                                    }
                                                };
                                                {
                                                    a=b;
                                                    cx_ite=cx_ite.next;
                                                };
                                            }
                                            if(cx_cont){
                                                do{
                                                    var b=lverts.front();
                                                    {
                                                        if(u==a||u==b||v==a||v==b)break;
                                                        var sx:Float=0.0;
                                                        var sy:Float=0.0;
                                                        {
                                                            sx=u.x-a.x;
                                                            sy=u.y-a.y;
                                                        };
                                                        var vx:Float=0.0;
                                                        var vy:Float=0.0;
                                                        {
                                                            vx=v.x-u.x;
                                                            vy=v.y-u.y;
                                                        };
                                                        var qx:Float=0.0;
                                                        var qy:Float=0.0;
                                                        {
                                                            qx=b.x-a.x;
                                                            qy=b.y-a.y;
                                                        };
                                                        var den=(vy*qx-vx*qy);
                                                        if(den*den>Config.epsilon){
                                                            den=1/den;
                                                            var t=(qy*sx-qx*sy)*den;
                                                            if(t>Config.epsilon&&t<1-Config.epsilon){
                                                                var s=(vy*sx-vx*sy)*den;
                                                                if(s>Config.epsilon&&s<1-Config.epsilon){
                                                                    cont=false;
                                                                    break;
                                                                }
                                                            }
                                                        }
                                                    };
                                                }
                                                while(false);
                                            }
                                        };
                                    };
                                }
                                while(false);
                            }
                        };
                        if(!cont)return validation=ValidationResult.SELF_INTERSECTING;
                        else return validation=ValidationResult.VALID;
                    }
                }
            }
        }
        else return validation;
    }
    public function validate_lverts(){
        if(zip_lverts){
            zip_lverts=false;
            if(lverts.size()>2){
                validate_area_inertia();
                if(area<0){
                    reverse_vertices();
                    area=-area;
                }
            }
        }
    }
    public function cleanup_lvert(x:ZPP_Vec2){
        var ite:ZPP_Vec2=null;
        var ite2:ZNPNode_ZPP_Edge=null;
        {
            var cx_ite=lverts.begin();
            while(cx_ite!=null){
                var i=cx_ite.elem();
                {
                    if(i==x)break;
                    else{
                        ite=ite==null?gverts.begin():ite.next;
                        ite2=ite2==null?edges.begin():ite2.next;
                    }
                };
                cx_ite=cx_ite.next;
            }
        };
        var rem=(ite==null?gverts.front():ite.next.elem());
        gverts.erase(ite);
        {
            var o=rem;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Vec2"+", in obj: "+"rem"+")");
                #end
            };
            o.free();
            o.next=ZPP_Vec2.zpp_pool;
            ZPP_Vec2.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_Vec2.POOL_CNT++;
            ZPP_Vec2.POOL_SUB++;
            #end
        };
        if(edgeCnt==2){
            var rem=edges.pop_unsafe();
            {
                var o=rem;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o!=null;
                    };
                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Edge"+", in obj: "+"rem"+")");
                    #end
                };
                o.free();
                o.next=ZPP_Edge.zpp_pool;
                ZPP_Edge.zpp_pool=o;
                #if NAPE_POOL_STATS ZPP_Edge.POOL_CNT++;
                ZPP_Edge.POOL_SUB++;
                #end
            };
            rem=edges.pop_unsafe();
            {
                var o=rem;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o!=null;
                    };
                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Edge"+", in obj: "+"rem"+")");
                    #end
                };
                o.free();
                o.next=ZPP_Edge.zpp_pool;
                ZPP_Edge.zpp_pool=o;
                #if NAPE_POOL_STATS ZPP_Edge.POOL_CNT++;
                ZPP_Edge.POOL_SUB++;
                #end
            };
            edgeCnt=0;
        }
        else if(edgeCnt!=0){
            var rem=(ite2==null?edges.front():ite2.next.elem());
            edges.erase(ite2);
            {
                var o=rem;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o!=null;
                    };
                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Edge"+", in obj: "+"rem"+")");
                    #end
                };
                o.free();
                o.next=ZPP_Edge.zpp_pool;
                ZPP_Edge.zpp_pool=o;
                #if NAPE_POOL_STATS ZPP_Edge.POOL_CNT++;
                ZPP_Edge.POOL_SUB++;
                #end
            };
            edgeCnt--;
        }
    }
    public var zip_sanitation:Bool=false;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function splice_collinear(){
        if(zip_sanitation){
            zip_sanitation=false;
            splice_collinear_real();
        }
    }
    public function splice_collinear_real(){
        if(lverts.begin()==null)return;
        if(lverts.begin().next==null)return;
        if(lverts.begin().next.next==null)return;
        var pre=null;
        var cur=lverts.begin();
        while(cur!=null){
            var nxt=if(cur.next==null)lverts.begin()else cur.next;
            var u=cur.elem();
            var v=nxt.elem();
            if(ZPP_VecMath.vec_dsq(u.x,u.y,v.x,v.y)<Config.epsilon*Config.epsilon){
                cleanup_lvert(cur.elem());
                cur=lverts.erase(pre);
            }
            else{
                pre=cur;
                cur=cur.next;
            }
        }
        if(lverts.empty())return;
        var removed;
        do{
            removed=false;
            var pre=lverts.begin();
            while(pre!=null){
                var cur=if(pre.next==null)lverts.begin()else pre.next;
                var nxt=if(cur.next==null)lverts.begin()else cur.next;
                var u=pre.elem();
                var v=cur.elem();
                var w=nxt.elem();
                var ax:Float=0.0;
                var ay:Float=0.0;
                {
                    ax=v.x-u.x;
                    ay=v.y-u.y;
                };
                var bx:Float=0.0;
                var by:Float=0.0;
                {
                    bx=w.x-v.x;
                    by=w.y-v.y;
                };
                var crs=(by*ax-bx*ay);
                if(crs*crs>=Config.epsilon*Config.epsilon){
                    pre=pre.next;
                }
                else{
                    cleanup_lvert(cur.elem());
                    lverts.erase(pre.next==null?null:pre);
                    removed=true;
                    pre=pre.next;
                }
            }
        }
        while(removed);
    }
    public function reverse_vertices(){
        lverts.reverse();
        gverts.reverse();
        edges.reverse();
        var ite=edges.iterator_at(edgeCnt-1);
        var elem=edges.pop_unsafe();
        edges.insert(ite,elem);
        reverse_flag=!reverse_flag;
        if(wrap_lverts!=null)wrap_lverts.zpp_inner.reverse_flag=reverse_flag;
        if(wrap_gverts!=null)wrap_gverts.zpp_inner.reverse_flag=reverse_flag;
        if(wrap_edges!=null)wrap_edges.zpp_inner.reverse_flag=reverse_flag;
    }
    public function validate_laxi(){
        if(zip_laxi){
            zip_laxi=false;
            validate_lverts();
            var ite=edges.begin();
            {
                var cx_ite=lverts.begin();
                var u=cx_ite.elem();
                cx_ite=cx_ite.next;
                while(cx_ite!=null){
                    var v=cx_ite.elem();
                    {
                        var edge=ite.elem();
                        ite=ite.next;
                        edge.lp0=u;
                        edge.lp1=v;
                        var dx:Float=0.0;
                        var dy:Float=0.0;
                        {
                            dx=u.x-v.x;
                            dy=u.y-v.y;
                        };
                        var l=Math.sqrt((dx*dx+dy*dy));
                        edge.length=l;
                        {
                            var t=(1.0/(l));
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"d"+",in s: "+"1.0/(l)"+")");
                                #end
                            };
                            dx*=t;
                            dy*=t;
                        };
                        {
                            var t=dx;
                            dx=-dy;
                            dy=t;
                        };
                        edge.lprojection=(dx*u.x+dy*u.y);
                        {
                            edge.lnormx=dx;
                            edge.lnormy=dy;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((edge.lnormx!=edge.lnormx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(edge.lnormx)"+") :: "+("vec_set(in n: "+"edge.lnorm"+",in x: "+"dx"+",in y: "+"dy"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((edge.lnormy!=edge.lnormy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(edge.lnormy)"+") :: "+("vec_set(in n: "+"edge.lnorm"+",in x: "+"dx"+",in y: "+"dy"+")");
                                #end
                            };
                        };
                        if(edge.wrap_lnorm!=null){
                            edge.wrap_lnorm.zpp_inner.x=dx;
                            edge.wrap_lnorm.zpp_inner.y=dy;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((edge.wrap_lnorm.zpp_inner.x!=edge.wrap_lnorm.zpp_inner.x));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(edge.wrap_lnorm.zpp_inner.x)"+") :: "+("vec_set(in n: "+"edge.wrap_lnorm.zpp_inner."+",in x: "+"dx"+",in y: "+"dy"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((edge.wrap_lnorm.zpp_inner.y!=edge.wrap_lnorm.zpp_inner.y));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(edge.wrap_lnorm.zpp_inner.y)"+") :: "+("vec_set(in n: "+"edge.wrap_lnorm.zpp_inner."+",in x: "+"dx"+",in y: "+"dy"+")");
                                #end
                            };
                        };
                    };
                    u=v;
                    cx_ite=cx_ite.next;
                }
                var v=lverts.front();
                {
                    var edge=ite.elem();
                    ite=ite.next;
                    edge.lp0=u;
                    edge.lp1=v;
                    var dx:Float=0.0;
                    var dy:Float=0.0;
                    {
                        dx=u.x-v.x;
                        dy=u.y-v.y;
                    };
                    var l=Math.sqrt((dx*dx+dy*dy));
                    edge.length=l;
                    {
                        var t=(1.0/(l));
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"d"+",in s: "+"1.0/(l)"+")");
                            #end
                        };
                        dx*=t;
                        dy*=t;
                    };
                    {
                        var t=dx;
                        dx=-dy;
                        dy=t;
                    };
                    edge.lprojection=(dx*u.x+dy*u.y);
                    {
                        edge.lnormx=dx;
                        edge.lnormy=dy;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((edge.lnormx!=edge.lnormx));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(edge.lnormx)"+") :: "+("vec_set(in n: "+"edge.lnorm"+",in x: "+"dx"+",in y: "+"dy"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((edge.lnormy!=edge.lnormy));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(edge.lnormy)"+") :: "+("vec_set(in n: "+"edge.lnorm"+",in x: "+"dx"+",in y: "+"dy"+")");
                            #end
                        };
                    };
                    if(edge.wrap_lnorm!=null){
                        edge.wrap_lnorm.zpp_inner.x=dx;
                        edge.wrap_lnorm.zpp_inner.y=dy;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((edge.wrap_lnorm.zpp_inner.x!=edge.wrap_lnorm.zpp_inner.x));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(edge.wrap_lnorm.zpp_inner.x)"+") :: "+("vec_set(in n: "+"edge.wrap_lnorm.zpp_inner."+",in x: "+"dx"+",in y: "+"dy"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((edge.wrap_lnorm.zpp_inner.y!=edge.wrap_lnorm.zpp_inner.y));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(edge.wrap_lnorm.zpp_inner.y)"+") :: "+("vec_set(in n: "+"edge.wrap_lnorm.zpp_inner."+",in x: "+"dx"+",in y: "+"dy"+")");
                            #end
                        };
                    };
                };
            };
        }
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function validate_gverts(){
        if(zip_gverts){
            if(body!=null){
                zip_gverts=false;
                validate_lverts();
                body.validate_axis();
                var li=lverts.begin();
                {
                    var cx_ite=gverts.begin();
                    while(cx_ite!=null){
                        var g=cx_ite.elem();
                        {
                            var l=li.elem();
                            li=li.next;
                            {
                                g.x=body.posx+(body.axisy*l.x-body.axisx*l.y);
                                g.y=body.posy+(l.x*body.axisx+l.y*body.axisy);
                            };
                        };
                        cx_ite=cx_ite.next;
                    }
                };
            }
        }
    }
    public#if NAPE_NO_INLINE#else inline #end
    function validate_gaxi(){
        if(zip_gaxi){
            if(body!=null){
                zip_gaxi=false;
                validate_laxi();
                body.validate_axis();
                validate_gverts();
                var ite=edges.begin();
                {
                    var cx_ite=gverts.begin();
                    var u=cx_ite.elem();
                    cx_ite=cx_ite.next;
                    while(cx_ite!=null){
                        var v=cx_ite.elem();
                        {
                            var e=ite.elem();
                            ite=ite.next;
                            e.gp0=u;
                            e.gp1=v;
                            {
                                e.gnormx=(body.axisy*e.lnormx-body.axisx*e.lnormy);
                                e.gnormy=(e.lnormx*body.axisx+e.lnormy*body.axisy);
                            };
                            e.gprojection=(body.posx*e.gnormx+body.posy*e.gnormy)+e.lprojection;
                            if(e.wrap_gnorm!=null){
                                e.wrap_gnorm.zpp_inner.x=e.gnormx;
                                e.wrap_gnorm.zpp_inner.y=e.gnormy;
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((e.wrap_gnorm.zpp_inner.x!=e.wrap_gnorm.zpp_inner.x));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(e.wrap_gnorm.zpp_inner.x)"+") :: "+("vec_set(in n: "+"e.wrap_gnorm.zpp_inner."+",in x: "+"e.gnormx"+",in y: "+"e.gnormy"+")");
                                    #end
                                };
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((e.wrap_gnorm.zpp_inner.y!=e.wrap_gnorm.zpp_inner.y));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(e.wrap_gnorm.zpp_inner.y)"+") :: "+("vec_set(in n: "+"e.wrap_gnorm.zpp_inner."+",in x: "+"e.gnormx"+",in y: "+"e.gnormy"+")");
                                    #end
                                };
                            };
                            e.tp0=(e.gp0.y*e.gnormx-e.gp0.x*e.gnormy);
                            e.tp1=(e.gp1.y*e.gnormx-e.gp1.x*e.gnormy);
                        };
                        u=v;
                        cx_ite=cx_ite.next;
                    }
                    var v=gverts.front();
                    {
                        var e=ite.elem();
                        ite=ite.next;
                        e.gp0=u;
                        e.gp1=v;
                        {
                            e.gnormx=(body.axisy*e.lnormx-body.axisx*e.lnormy);
                            e.gnormy=(e.lnormx*body.axisx+e.lnormy*body.axisy);
                        };
                        e.gprojection=(body.posx*e.gnormx+body.posy*e.gnormy)+e.lprojection;
                        if(e.wrap_gnorm!=null){
                            e.wrap_gnorm.zpp_inner.x=e.gnormx;
                            e.wrap_gnorm.zpp_inner.y=e.gnormy;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((e.wrap_gnorm.zpp_inner.x!=e.wrap_gnorm.zpp_inner.x));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(e.wrap_gnorm.zpp_inner.x)"+") :: "+("vec_set(in n: "+"e.wrap_gnorm.zpp_inner."+",in x: "+"e.gnormx"+",in y: "+"e.gnormy"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((e.wrap_gnorm.zpp_inner.y!=e.wrap_gnorm.zpp_inner.y));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(e.wrap_gnorm.zpp_inner.y)"+") :: "+("vec_set(in n: "+"e.wrap_gnorm.zpp_inner."+",in x: "+"e.gnormx"+",in y: "+"e.gnormy"+")");
                                #end
                            };
                        };
                        e.tp0=(e.gp0.y*e.gnormx-e.gp0.x*e.gnormy);
                        e.tp1=(e.gp1.y*e.gnormx-e.gp1.x*e.gnormy);
                    };
                };
            }
        }
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function __validate_aabb(){
        validate_gverts();
        #if(!NAPE_RELEASE_BUILD)
        if(lverts.empty())throw "Error: An empty polygon has no meaningful bounds";
        #end
        var p0=gverts.front();
        {
            aabb.minx=p0.x;
            aabb.miny=p0.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((aabb.minx!=aabb.minx));
                };
                if(!res)throw "assert("+"!assert_isNaN(aabb.minx)"+") :: "+("vec_set(in n: "+"aabb.min"+",in x: "+"p0.x"+",in y: "+"p0.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((aabb.miny!=aabb.miny));
                };
                if(!res)throw "assert("+"!assert_isNaN(aabb.miny)"+") :: "+("vec_set(in n: "+"aabb.min"+",in x: "+"p0.x"+",in y: "+"p0.y"+")");
                #end
            };
        };
        {
            aabb.maxx=p0.x;
            aabb.maxy=p0.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((aabb.maxx!=aabb.maxx));
                };
                if(!res)throw "assert("+"!assert_isNaN(aabb.maxx)"+") :: "+("vec_set(in n: "+"aabb.max"+",in x: "+"p0.x"+",in y: "+"p0.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((aabb.maxy!=aabb.maxy));
                };
                if(!res)throw "assert("+"!assert_isNaN(aabb.maxy)"+") :: "+("vec_set(in n: "+"aabb.max"+",in x: "+"p0.x"+",in y: "+"p0.y"+")");
                #end
            };
        };
        {
            var cx_ite=gverts.begin().next;
            while(cx_ite!=null){
                var p=cx_ite.elem();
                {
                    if(p.x<aabb.minx)aabb.minx=p.x;
                    if(p.x>aabb.maxx)aabb.maxx=p.x;
                    if(p.y<aabb.miny)aabb.miny=p.y;
                    if(p.y>aabb.maxy)aabb.maxy=p.y;
                };
                cx_ite=cx_ite.next;
            }
        };
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function _force_validate_aabb(){
        var li=lverts.begin();
        var p0=gverts.front();
        var l=li.elem();
        li=li.next;
        {
            p0.x=body.posx+(body.axisy*l.x-body.axisx*l.y);
            p0.y=body.posy+(l.x*body.axisx+l.y*body.axisy);
        };
        {
            aabb.minx=p0.x;
            aabb.miny=p0.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((aabb.minx!=aabb.minx));
                };
                if(!res)throw "assert("+"!assert_isNaN(aabb.minx)"+") :: "+("vec_set(in n: "+"aabb.min"+",in x: "+"p0.x"+",in y: "+"p0.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((aabb.miny!=aabb.miny));
                };
                if(!res)throw "assert("+"!assert_isNaN(aabb.miny)"+") :: "+("vec_set(in n: "+"aabb.min"+",in x: "+"p0.x"+",in y: "+"p0.y"+")");
                #end
            };
        };
        {
            aabb.maxx=p0.x;
            aabb.maxy=p0.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((aabb.maxx!=aabb.maxx));
                };
                if(!res)throw "assert("+"!assert_isNaN(aabb.maxx)"+") :: "+("vec_set(in n: "+"aabb.max"+",in x: "+"p0.x"+",in y: "+"p0.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((aabb.maxy!=aabb.maxy));
                };
                if(!res)throw "assert("+"!assert_isNaN(aabb.maxy)"+") :: "+("vec_set(in n: "+"aabb.max"+",in x: "+"p0.x"+",in y: "+"p0.y"+")");
                #end
            };
        };
        {
            var cx_ite=gverts.begin().next;
            while(cx_ite!=null){
                var p=cx_ite.elem();
                {
                    var l=li.elem();
                    li=li.next;
                    {
                        p.x=body.posx+(body.axisy*l.x-body.axisx*l.y);
                        p.y=body.posy+(l.x*body.axisx+l.y*body.axisy);
                    };
                    if(p.x<aabb.minx)aabb.minx=p.x;
                    if(p.x>aabb.maxx)aabb.maxx=p.x;
                    if(p.y<aabb.miny)aabb.miny=p.y;
                    if(p.y>aabb.maxy)aabb.maxy=p.y;
                };
                cx_ite=cx_ite.next;
            }
        };
    }
    public function __validate_sweepRadius(){
        var maxRadius=0.0;
        var minRadius=0.0;
        validate_laxi();
        {
            var cx_ite=lverts.begin();
            while(cx_ite!=null){
                var x=cx_ite.elem();
                {
                    var r=(x.x*x.x+x.y*x.y);
                    if(r>maxRadius){
                        maxRadius=r;
                    }
                };
                cx_ite=cx_ite.next;
            }
        };
        {
            var cx_ite=edges.begin();
            while(cx_ite!=null){
                var e=cx_ite.elem();
                {
                    if(e.lprojection<minRadius){
                        minRadius=e.lprojection;
                        if(minRadius<0)break;
                    }
                };
                cx_ite=cx_ite.next;
            }
        };
        if(minRadius<0)minRadius=0;
        sweepRadius=Math.sqrt(maxRadius);
        sweepCoef=sweepRadius-minRadius;
    }
    public function __validate_area_inertia(){
        if(lverts.begin()==null||lverts.begin().next==null||lverts.begin().next.next==null){
            area=0;
            inertia=0;
        }
        else{
            area=0.0;
            var s1=0.0;
            var s2=0.0;
            {
                var cx_ite=lverts.begin();
                var u=cx_ite.elem();
                cx_ite=cx_ite.next;
                var v=cx_ite.elem();
                cx_ite=cx_ite.next;
                while(cx_ite!=null){
                    var w=cx_ite.elem();
                    {
                        var a=(v.y*u.x-v.x*u.y);
                        var b=(v.x*v.x+v.y*v.y)+(v.x*u.x+v.y*u.y)+(u.x*u.x+u.y*u.y);
                        s1+=a*b;
                        s2+=a;
                        area+=v.x*(w.y-u.y);
                    };
                    u=v;
                    v=w;
                    cx_ite=cx_ite.next;
                }
                cx_ite=lverts.begin();
                var w=cx_ite.elem();
                {
                    var a=(v.y*u.x-v.x*u.y);
                    var b=(v.x*v.x+v.y*v.y)+(v.x*u.x+v.y*u.y)+(u.x*u.x+u.y*u.y);
                    s1+=a*b;
                    s2+=a;
                    area+=v.x*(w.y-u.y);
                };
                u=v;
                v=w;
                cx_ite=cx_ite.next;
                var w=cx_ite.elem();
                {
                    var a=(v.y*u.x-v.x*u.y);
                    var b=(v.x*v.x+v.y*v.y)+(v.x*u.x+v.y*u.y)+(u.x*u.x+u.y*u.y);
                    s1+=a*b;
                    s2+=a;
                    area+=v.x*(w.y-u.y);
                };
            };
            inertia=s1/(6*s2);
            area*=0.5;
            if(area<0){
                area=-area;
                reverse_vertices();
            }
        }
    }
    public function __validate_angDrag(){
        #if(!NAPE_RELEASE_BUILD)
        if(lverts.size()<3)throw "Error: Polygon's with less than 3 vertices have no meaningful angDrag";
        #end
        validate_area_inertia();
        validate_laxi();
        var accum=0.0;
        var ei=edges.begin();
        var perim=0.0;
        {
            var cx_cont=true;
            var cx_itei=lverts.begin();
            var u=cx_itei.elem();
            var cx_itej=cx_itei.next;
            while(cx_itej!=null){
                var v=cx_itej.elem();
                {
                    var edge=ei.elem();
                    ei=ei.next;
                    perim+=edge.length;
                    var dx:Float=0.0;
                    var dy:Float=0.0;
                    {
                        dx=v.x-u.x;
                        dy=v.y-u.y;
                    };
                    accum+=edge.length*Config.fluidAngularDragFriction*material.dynamicFriction*edge.lprojection*edge.lprojection;
                    var t=-(u.y*edge.lnormx-u.x*edge.lnormy)/(dy*edge.lnormx-dx*edge.lnormy);
                    if(t>0){
                        var ta=if(t>1)1 else t;
                        var cx:Float=0.0;
                        var cy:Float=0.0;
                        {
                            cx=u.x;
                            cy=u.y;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((cx!=cx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(cx)"+") :: "+("vec_set(in n: "+"c"+",in x: "+"u.x"+",in y: "+"u.y"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((cy!=cy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(cy)"+") :: "+("vec_set(in n: "+"c"+",in x: "+"u.x"+",in y: "+"u.y"+")");
                                #end
                            };
                        };
                        {
                            var t=(ta);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"c"+",in b: "+"d"+",in s: "+"ta"+")");
                                #end
                            };
                            cx+=dx*t;
                            cy+=dy*t;
                        };
                        var dota=(edge.lnormy*u.x-edge.lnormx*u.y);
                        var dotb=(edge.lnormy*cx-edge.lnormx*cy);
                        var dots=(dotb*dotb*dotb-dota*dota*dota)/(3*(dotb-dota));
                        accum+=dots*ta*edge.length*Config.fluidAngularDrag;
                    }
                    if(t<1){
                        var tb=if(t<0)0 else t;
                        var cx:Float=0.0;
                        var cy:Float=0.0;
                        {
                            cx=u.x;
                            cy=u.y;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((cx!=cx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(cx)"+") :: "+("vec_set(in n: "+"c"+",in x: "+"u.x"+",in y: "+"u.y"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((cy!=cy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(cy)"+") :: "+("vec_set(in n: "+"c"+",in x: "+"u.x"+",in y: "+"u.y"+")");
                                #end
                            };
                        };
                        {
                            var t=(tb);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"c"+",in b: "+"d"+",in s: "+"tb"+")");
                                #end
                            };
                            cx+=dx*t;
                            cy+=dy*t;
                        };
                        var dota=(edge.lnormy*cx-edge.lnormx*cy);
                        var dotb=(edge.lnormy*v.x-edge.lnormx*v.y);
                        var dots=(dotb*dotb*dotb-dota*dota*dota)/(3*(dotb-dota));
                        accum+=dots*Config.fluidVacuumDrag*(1-tb)*edge.length*Config.fluidAngularDrag;
                    }
                };
                {
                    cx_itei=cx_itej;
                    u=v;
                    cx_itej=cx_itej.next;
                };
            }
            if(cx_cont){
                do{
                    cx_itej=lverts.begin();
                    var v=cx_itej.elem();
                    {
                        var edge=ei.elem();
                        ei=ei.next;
                        perim+=edge.length;
                        var dx:Float=0.0;
                        var dy:Float=0.0;
                        {
                            dx=v.x-u.x;
                            dy=v.y-u.y;
                        };
                        accum+=edge.length*Config.fluidAngularDragFriction*material.dynamicFriction*edge.lprojection*edge.lprojection;
                        var t=-(u.y*edge.lnormx-u.x*edge.lnormy)/(dy*edge.lnormx-dx*edge.lnormy);
                        if(t>0){
                            var ta=if(t>1)1 else t;
                            var cx:Float=0.0;
                            var cy:Float=0.0;
                            {
                                cx=u.x;
                                cy=u.y;
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((cx!=cx));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(cx)"+") :: "+("vec_set(in n: "+"c"+",in x: "+"u.x"+",in y: "+"u.y"+")");
                                    #end
                                };
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((cy!=cy));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(cy)"+") :: "+("vec_set(in n: "+"c"+",in x: "+"u.x"+",in y: "+"u.y"+")");
                                    #end
                                };
                            };
                            {
                                var t=(ta);
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((t!=t));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"c"+",in b: "+"d"+",in s: "+"ta"+")");
                                    #end
                                };
                                cx+=dx*t;
                                cy+=dy*t;
                            };
                            var dota=(edge.lnormy*u.x-edge.lnormx*u.y);
                            var dotb=(edge.lnormy*cx-edge.lnormx*cy);
                            var dots=(dotb*dotb*dotb-dota*dota*dota)/(3*(dotb-dota));
                            accum+=dots*ta*edge.length*Config.fluidAngularDrag;
                        }
                        if(t<1){
                            var tb=if(t<0)0 else t;
                            var cx:Float=0.0;
                            var cy:Float=0.0;
                            {
                                cx=u.x;
                                cy=u.y;
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((cx!=cx));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(cx)"+") :: "+("vec_set(in n: "+"c"+",in x: "+"u.x"+",in y: "+"u.y"+")");
                                    #end
                                };
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((cy!=cy));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(cy)"+") :: "+("vec_set(in n: "+"c"+",in x: "+"u.x"+",in y: "+"u.y"+")");
                                    #end
                                };
                            };
                            {
                                var t=(tb);
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((t!=t));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"c"+",in b: "+"d"+",in s: "+"tb"+")");
                                    #end
                                };
                                cx+=dx*t;
                                cy+=dy*t;
                            };
                            var dota=(edge.lnormy*cx-edge.lnormx*cy);
                            var dotb=(edge.lnormy*v.x-edge.lnormx*v.y);
                            var dots=(dotb*dotb*dotb-dota*dota*dota)/(3*(dotb-dota));
                            accum+=dots*Config.fluidVacuumDrag*(1-tb)*edge.length*Config.fluidAngularDrag;
                        }
                    };
                }
                while(false);
            }
        };
        angDrag=accum/(inertia*perim);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function __validate_localCOM(){
        #if(!NAPE_RELEASE_BUILD)
        if(lverts.empty())throw "Error: An empty polygon has no meaningful localCOM";
        #end
        if(lverts.begin().next==null){
            localCOMx=lverts.front().x;
            localCOMy=lverts.front().y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((localCOMx!=localCOMx));
                };
                if(!res)throw "assert("+"!assert_isNaN(localCOMx)"+") :: "+("vec_set(in n: "+"localCOM"+",in x: "+"lverts.front().x"+",in y: "+"lverts.front().y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((localCOMy!=localCOMy));
                };
                if(!res)throw "assert("+"!assert_isNaN(localCOMy)"+") :: "+("vec_set(in n: "+"localCOM"+",in x: "+"lverts.front().x"+",in y: "+"lverts.front().y"+")");
                #end
            };
        };
        else if(lverts.begin().next.next==null){
            {
                localCOMx=lverts.front().x;
                localCOMy=lverts.front().y;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((localCOMx!=localCOMx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(localCOMx)"+") :: "+("vec_set(in n: "+"localCOM"+",in x: "+"lverts.front().x"+",in y: "+"lverts.front().y"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((localCOMy!=localCOMy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(localCOMy)"+") :: "+("vec_set(in n: "+"localCOM"+",in x: "+"lverts.front().x"+",in y: "+"lverts.front().y"+")");
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
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"localCOM"+",in b: "+"lverts.begin().next.elem()."+",in s: "+"1.0"+")");
                    #end
                };
                localCOMx+=lverts.begin().next.elem().x*t;
                localCOMy+=lverts.begin().next.elem().y*t;
            };
            {
                var t=(0.5);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"localCOM"+",in s: "+"0.5"+")");
                    #end
                };
                localCOMx*=t;
                localCOMy*=t;
            };
        }
        else{
            {
                localCOMx=0;
                localCOMy=0;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((localCOMx!=localCOMx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(localCOMx)"+") :: "+("vec_set(in n: "+"localCOM"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((localCOMy!=localCOMy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(localCOMy)"+") :: "+("vec_set(in n: "+"localCOM"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
            };
            var area=0.0;
            {
                var cx_ite=lverts.begin();
                var u=cx_ite.elem();
                cx_ite=cx_ite.next;
                var v=cx_ite.elem();
                cx_ite=cx_ite.next;
                while(cx_ite!=null){
                    var w=cx_ite.elem();
                    {
                        area+=v.x*(w.y-u.y);
                        var cf=(w.y*v.x-w.x*v.y);
                        localCOMx+=(v.x+w.x)*cf;
                        localCOMy+=(v.y+w.y)*cf;
                    };
                    u=v;
                    v=w;
                    cx_ite=cx_ite.next;
                }
                cx_ite=lverts.begin();
                var w=cx_ite.elem();
                {
                    area+=v.x*(w.y-u.y);
                    var cf=(w.y*v.x-w.x*v.y);
                    localCOMx+=(v.x+w.x)*cf;
                    localCOMy+=(v.y+w.y)*cf;
                };
                u=v;
                v=w;
                cx_ite=cx_ite.next;
                var w=cx_ite.elem();
                {
                    area+=v.x*(w.y-u.y);
                    var cf=(w.y*v.x-w.x*v.y);
                    localCOMx+=(v.x+w.x)*cf;
                    localCOMy+=(v.y+w.y)*cf;
                };
            };
            area=1/(3*area);
            {
                var t=(area);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"localCOM"+",in s: "+"area"+")");
                    #end
                };
                localCOMx*=t;
                localCOMy*=t;
            };
        }
    }
    private function localCOM_validate(){
        #if(!NAPE_RELEASE_BUILD)
        if(lverts.empty())throw "Error: An empty polygon does not have any meaningful localCOM";
        #end
        validate_localCOM();
    }
    private function localCOM_invalidate(x:ZPP_Vec2){
        validate_localCOM();
        var delx:Float=0;
        var dely:Float=0;
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((delx!=delx));
            };
            if(!res)throw "assert("+"!assert_isNaN(delx)"+") :: "+("vec_new(in n: "+"del"+",in x: "+"0"+",in y: "+"0"+")");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((dely!=dely));
            };
            if(!res)throw "assert("+"!assert_isNaN(dely)"+") :: "+("vec_new(in n: "+"del"+",in x: "+"0"+",in y: "+"0"+")");
            #end
        };
        {
            delx=x.x-localCOMx;
            dely=x.y-localCOMy;
        };
        {
            var cx_ite=lverts.begin();
            while(cx_ite!=null){
                var p=cx_ite.elem();
                {
                    var t=(1.0);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"p."+",in b: "+"del"+",in s: "+"1.0"+")");
                        #end
                    };
                    p.x+=delx*t;
                    p.y+=dely*t;
                };
                cx_ite=cx_ite.next;
            }
        };
        invalidate_lverts();
    }
    public function setupLocalCOM(){
        wrap_localCOM=Vec2.get(localCOMx,localCOMy);
        wrap_localCOM.zpp_inner._inuse=true;
        wrap_localCOM.zpp_inner._validate=localCOM_validate;
        wrap_localCOM.zpp_inner._invalidate=localCOM_invalidate;
    }
    public function new(){
        super(ZPP_Flags.id_ShapeType_POLYGON);
        polygon=this;
        lverts=new ZPP_Vec2();
        gverts=new ZPP_Vec2();
        edges=new ZNPList_ZPP_Edge();
        edgeCnt=0;
    }
    public function __translate(dx:Float,dy:Float){
        {
            var cx_ite=lverts.begin();
            while(cx_ite!=null){
                var p=cx_ite.elem();
                {
                    var t=(1.0);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"p."+",in b: "+"d"+",in s: "+"1.0"+")");
                        #end
                    };
                    p.x+=dx*t;
                    p.y+=dy*t;
                };
                cx_ite=cx_ite.next;
            }
        };
        invalidate_lverts();
    }
    public function __scale(sx:Float,sy:Float){
        {
            var cx_ite=lverts.begin();
            while(cx_ite!=null){
                var p=cx_ite.elem();
                {
                    p.x*=sx;
                    p.y*=sy;
                };
                cx_ite=cx_ite.next;
            }
        };
        invalidate_lverts();
    }
    public function __rotate(ax:Float,ay:Float){
        {
            var cx_ite=lverts.begin();
            while(cx_ite!=null){
                var p=cx_ite.elem();
                {
                    var tempx:Float=0.0;
                    var tempy:Float=0.0;
                    {
                        tempx=(ay*p.x-ax*p.y);
                        tempy=(p.x*ax+p.y*ay);
                    };
                    {
                        p.x=tempx;
                        p.y=tempy;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((p.x!=p.x));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(p.x)"+") :: "+("vec_set(in n: "+"p."+",in x: "+"tempx"+",in y: "+"tempy"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((p.y!=p.y));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(p.y)"+") :: "+("vec_set(in n: "+"p."+",in x: "+"tempx"+",in y: "+"tempy"+")");
                            #end
                        };
                    };
                };
                cx_ite=cx_ite.next;
            }
        };
        invalidate_lverts();
    }
    public function __transform(mat:Mat23){
        {
            var cx_ite=lverts.begin();
            while(cx_ite!=null){
                var p=cx_ite.elem();
                {
                    {
                        var t=mat.a*p.x+mat.b*p.y+mat.tx;
                        p.y=mat.c*p.x+mat.d*p.y+mat.ty;
                        p.x=t;
                    };
                };
                cx_ite=cx_ite.next;
            }
        };
        invalidate_lverts();
    }
    public function __copy(){
        var ret=new Polygon(outer_zn.localVerts).zpp_inner_zn;
        return ret;
    }
}

package zpp_nape.phys;
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
class ZPP_Body extends ZPP_Interactor{
    public var outer:Body=null;
    public var world:Bool=false;
    public var type:Int=0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function isStatic(){
        return type==ZPP_Flags.id_BodyType_STATIC;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function isDynamic(){
        return type==ZPP_Flags.id_BodyType_DYNAMIC;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function isKinematic(){
        return type==ZPP_Flags.id_BodyType_KINEMATIC;
    }
    public static var types:Array<BodyType>=[null,BodyType.STATIC,BodyType.DYNAMIC,BodyType.KINEMATIC];
    public function invalidate_type(){
        invalidate_mass();
        invalidate_inertia();
    }
    public var compound:ZPP_Compound=null;
    public var shapes:ZNPList_ZPP_Shape=null;
    public var wrap_shapes:ShapeList=null;
    public function invalidate_shapes(){
        invalidate_aabb();
        invalidate_localCOM();
        invalidate_mass();
        invalidate_inertia();
    }
    public var space:ZPP_Space=null;
    public var arbiters:ZNPList_ZPP_Arbiter=null;
    public var wrap_arbiters:ArbiterList=null;
    public var constraints:ZNPList_ZPP_Constraint=null;
    public var wrap_constraints:ConstraintList=null;
    public var component:ZPP_Component=null;
    static var bodystack:ZNPList_ZPP_Body=null;
    static var bodyset:ZPP_Set_ZPP_Body=null;
    static function bodysetlt(a:ZPP_Body,b:ZPP_Body){
        return a.id<b.id;
    }
    public var graph_depth:Int=0;
    #if NAPE_NO_INLINE#else inline #end
    function init_bodysetlist(){
        if(bodyset==null){
            bodyset=new ZPP_Set_ZPP_Body();
            bodyset.lt=bodysetlt;
            bodystack=new ZNPList_ZPP_Body();
        }
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                bodyset.empty();
            };
            if(!res)throw "assert("+"bodyset.empty()"+") :: "+("non-empty bodyset in connected bodies at init.");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                bodystack.empty();
            };
            if(!res)throw "assert("+"bodystack.empty()"+") :: "+("non-empty bodystack in connected bodies at init.");
            #end
        };
    }
    static var cur_graph_depth:Int=0;
    function connectedBodies_cont(b:Body){
        if(bodyset.try_insert_bool(b.zpp_inner)){
            bodystack.add(b.zpp_inner);
            b.zpp_inner.graph_depth=cur_graph_depth+1;
        }
    }
    public function connectedBodies(depth:Int,output:BodyList):BodyList{
        init_bodysetlist();
        var ret=(output==null?new BodyList():output);
        bodystack.add(this);
        bodyset.insert(this);
        graph_depth=0;
        while(!bodystack.empty()){
            var cur=bodystack.pop_unsafe();
            if(cur.graph_depth==depth)continue;
            cur_graph_depth=cur.graph_depth;
            {
                var cx_ite=cur.constraints.begin();
                while(cx_ite!=null){
                    var c=cx_ite.elem();
                    c.outer.visitBodies(connectedBodies_cont);
                    cx_ite=cx_ite.next;
                }
            };
        }
        bodyset.clear_with(function(b:ZPP_Body){
            if(b!=this)ret.add(b.outer);
        });
        return ret;
    }
    public function interactingBodies(arbiter_type:Int,depth:Int,output:BodyList):BodyList{
        init_bodysetlist();
        var ret=(output==null?new BodyList():output);
        bodyset.insert(this);
        bodystack.add(this);
        graph_depth=0;
        while(!bodystack.empty()){
            var cur=bodystack.pop_unsafe();
            if(cur.graph_depth==depth)continue;
            {
                var cx_ite=cur.arbiters.begin();
                while(cx_ite!=null){
                    var arb=cx_ite.elem();
                    {
                        if((arb.type&arbiter_type)!=0){
                            var other=if(arb.b1==cur)arb.b2 else arb.b1;
                            if(bodyset.try_insert_bool(other)){
                                bodystack.add(other);
                                other.graph_depth=cur.graph_depth+1;
                            }
                        }
                    };
                    cx_ite=cx_ite.next;
                }
            };
        }
        bodyset.clear_with(function(b:ZPP_Body){
            if(b!=this)ret.add(b.outer);
        });
        return ret;
    }
    public function atRest(dt:Float){
        if(!isDynamic())return component.sleeping;
        else{
            var linSq=Config.linearSleepThreshold;
            linSq*=linSq;
            var cansleep=if((velx*velx+vely*vely)>linSq)false;
            else if(ZPP_VecMath.vec_dsq(posx,posy,pre_posx,pre_posy)>0.25*linSq*dt*dt)false else{
                var dx:Float=0.0;
                var dy:Float=0.0;
                {
                    dx=aabb.maxx-aabb.minx;
                    dy=aabb.maxy-aabb.miny;
                };
                var idl=(dx*dx+dy*dy);
                var angSq=Config.angularSleepThreshold;
                angSq*=angSq;
                if(4*angvel*angvel*idl>angSq)false else{
                    var dr=rot-pre_rot;
                    if(dr*dr*idl>angSq*dt*dt)false else true;
                }
            }
            if(!cansleep)component.waket=space.stamp;
            return component.waket+Config.sleepDelay<space.stamp;
        }
    }
    public function refreshArbiters(){
        {
            var cx_ite=arbiters.begin();
            while(cx_ite!=null){
                var arb=cx_ite.elem();
                arb.invalidated=true;
                cx_ite=cx_ite.next;
            }
        };
    }
    public var sweepTime:Float=0.0;
    public var sweep_angvel:Float=0.0;
    public var sweepFrozen:Bool=false;
    public var sweepRadius:Float=0.0;
    public var bullet:Bool=false;
    public var bulletEnabled:Bool=false;
    public var disableCCD:Bool=false;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function sweepIntegrate(dt:Float){
        var delta=dt-sweepTime;
        if(delta!=0){
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !(delta!=delta);
                };
                if(!res)throw "assert("+"!(delta!=delta)"+") :: "+(delta+" "+dt+" "+sweepTime);
                #end
            };
            sweepTime=dt;
            {
                var t=(delta);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"pos"+",in b: "+"vel"+",in s: "+"delta"+")");
                    #end
                };
                posx+=velx*t;
                posy+=vely*t;
            };
            if(angvel!=0)delta_rot(sweep_angvel*delta);
        }
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function sweepValidate(s:ZPP_Shape){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                s.body==this;
            };
            if(!res)throw "assert("+"s.body==this"+") :: "+("sweepValidate with non owned shape");
            #end
        };
        if(s.isCircle()){
            {
                s.worldCOMx=posx+(axisy*s.localCOMx-axisx*s.localCOMy);
                s.worldCOMy=posy+(s.localCOMx*axisx+s.localCOMy*axisy);
            };
        }
        else{
            var p=s.polygon;
            var li=p.lverts.begin();
            {
                var cx_ite=p.gverts.begin();
                while(cx_ite!=null){
                    var g=cx_ite.elem();
                    {
                        var l=li.elem();
                        li=li.next;
                        {
                            g.x=posx+(axisy*l.x-axisx*l.y);
                            g.y=posy+(l.x*axisx+l.y*axisy);
                        };
                    };
                    cx_ite=cx_ite.next;
                }
            };
            var ite=p.edges.begin();
            {
                var cx_ite=p.gverts.begin();
                var u=cx_ite.elem();
                cx_ite=cx_ite.next;
                while(cx_ite!=null){
                    var v=cx_ite.elem();
                    {
                        var e=ite.elem();
                        ite=ite.next;
                        {
                            e.gnormx=(axisy*e.lnormx-axisx*e.lnormy);
                            e.gnormy=(e.lnormx*axisx+e.lnormy*axisy);
                        };
                        e.gprojection=(posx*e.gnormx+posy*e.gnormy)+e.lprojection;
                        e.tp0=(u.y*e.gnormx-u.x*e.gnormy);
                        e.tp1=(v.y*e.gnormx-v.x*e.gnormy);
                    };
                    u=v;
                    cx_ite=cx_ite.next;
                }
                var v=p.gverts.front();
                {
                    var e=ite.elem();
                    ite=ite.next;
                    {
                        e.gnormx=(axisy*e.lnormx-axisx*e.lnormy);
                        e.gnormy=(e.lnormx*axisx+e.lnormy*axisy);
                    };
                    e.gprojection=(posx*e.gnormx+posy*e.gnormy)+e.lprojection;
                    e.tp0=(u.y*e.gnormx-u.x*e.gnormy);
                    e.tp1=(v.y*e.gnormx-v.x*e.gnormy);
                };
            };
        }
    }
    public var pre_posx:Float=0.0;
    public var pre_posy:Float=0.0;
    public var posx:Float=0.0;
    public var posy:Float=0.0;
    public var wrap_pos:Vec2=null;
    public var velx:Float=0.0;
    public var vely:Float=0.0;
    public var wrap_vel:Vec2=null;
    public var forcex:Float=0.0;
    public var forcey:Float=0.0;
    public var wrap_force:Vec2=null;
    public var kinvelx:Float=0.0;
    public var kinvely:Float=0.0;
    public var wrap_kinvel:Vec2=null;
    public var svelx:Float=0.0;
    public var svely:Float=0.0;
    public var wrap_svel:Vec2=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function invalidate_pos(){
        {
            var cx_ite=shapes.begin();
            while(cx_ite!=null){
                var s=cx_ite.elem();
                {
                    if(s.type==ZPP_Flags.id_ShapeType_POLYGON){
                        s.polygon.invalidate_gverts();
                        s.polygon.invalidate_gaxi();
                    }
                    s.invalidate_worldCOM();
                };
                cx_ite=cx_ite.next;
            }
        };
        invalidate_worldCOM();
    }
    private function pos_invalidate(pos:ZPP_Vec2){
        immutable_midstep("Body::position");
        #if(!NAPE_RELEASE_BUILD)
        if(isStatic()&&space!=null)throw "Error: Cannot move a static object once inside a Space";
        #end
        if(!(posx==pos.x&&posy==pos.y)){
            {
                posx=pos.x;
                posy=pos.y;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((posx!=posx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(posx)"+") :: "+("vec_set(in n: "+"pos"+",in x: "+"pos.x"+",in y: "+"pos.y"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((posy!=posy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(posy)"+") :: "+("vec_set(in n: "+"pos"+",in x: "+"pos.x"+",in y: "+"pos.y"+")");
                    #end
                };
            };
            invalidate_pos();
            wake();
        }
    }
    private function pos_validate(){
        {
            wrap_pos.zpp_inner.x=posx;
            wrap_pos.zpp_inner.y=posy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_pos.zpp_inner.x!=wrap_pos.zpp_inner.x));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_pos.zpp_inner.x)"+") :: "+("vec_set(in n: "+"wrap_pos.zpp_inner."+",in x: "+"posx"+",in y: "+"posy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_pos.zpp_inner.y!=wrap_pos.zpp_inner.y));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_pos.zpp_inner.y)"+") :: "+("vec_set(in n: "+"wrap_pos.zpp_inner."+",in x: "+"posx"+",in y: "+"posy"+")");
                #end
            };
        };
    }
    private function vel_invalidate(vel:ZPP_Vec2){
        #if(!NAPE_RELEASE_BUILD)
        if(isStatic())throw "Error: Static body cannot have its velocity set.";
        #end
        {
            velx=vel.x;
            vely=vel.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((velx!=velx));
                };
                if(!res)throw "assert("+"!assert_isNaN(velx)"+") :: "+("vec_set(in n: "+"vel"+",in x: "+"vel.x"+",in y: "+"vel.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((vely!=vely));
                };
                if(!res)throw "assert("+"!assert_isNaN(vely)"+") :: "+("vec_set(in n: "+"vel"+",in x: "+"vel.x"+",in y: "+"vel.y"+")");
                #end
            };
        };
        invalidate_wake();
    }
    private function vel_validate(){
        {
            wrap_vel.zpp_inner.x=velx;
            wrap_vel.zpp_inner.y=vely;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_vel.zpp_inner.x!=wrap_vel.zpp_inner.x));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_vel.zpp_inner.x)"+") :: "+("vec_set(in n: "+"wrap_vel.zpp_inner."+",in x: "+"velx"+",in y: "+"vely"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_vel.zpp_inner.y!=wrap_vel.zpp_inner.y));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_vel.zpp_inner.y)"+") :: "+("vec_set(in n: "+"wrap_vel.zpp_inner."+",in x: "+"velx"+",in y: "+"vely"+")");
                #end
            };
        };
    }
    private function kinvel_invalidate(vel:ZPP_Vec2){
        {
            kinvelx=vel.x;
            kinvely=vel.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((kinvelx!=kinvelx));
                };
                if(!res)throw "assert("+"!assert_isNaN(kinvelx)"+") :: "+("vec_set(in n: "+"kinvel"+",in x: "+"vel.x"+",in y: "+"vel.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((kinvely!=kinvely));
                };
                if(!res)throw "assert("+"!assert_isNaN(kinvely)"+") :: "+("vec_set(in n: "+"kinvel"+",in x: "+"vel.x"+",in y: "+"vel.y"+")");
                #end
            };
        };
        invalidate_wake();
    }
    private function kinvel_validate(){
        {
            wrap_kinvel.zpp_inner.x=kinvelx;
            wrap_kinvel.zpp_inner.y=kinvely;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_kinvel.zpp_inner.x!=wrap_kinvel.zpp_inner.x));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_kinvel.zpp_inner.x)"+") :: "+("vec_set(in n: "+"wrap_kinvel.zpp_inner."+",in x: "+"kinvelx"+",in y: "+"kinvely"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_kinvel.zpp_inner.y!=wrap_kinvel.zpp_inner.y));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_kinvel.zpp_inner.y)"+") :: "+("vec_set(in n: "+"wrap_kinvel.zpp_inner."+",in x: "+"kinvelx"+",in y: "+"kinvely"+")");
                #end
            };
        };
    }
    private function svel_invalidate(vel:ZPP_Vec2){
        {
            svelx=vel.x;
            svely=vel.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((svelx!=svelx));
                };
                if(!res)throw "assert("+"!assert_isNaN(svelx)"+") :: "+("vec_set(in n: "+"svel"+",in x: "+"vel.x"+",in y: "+"vel.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((svely!=svely));
                };
                if(!res)throw "assert("+"!assert_isNaN(svely)"+") :: "+("vec_set(in n: "+"svel"+",in x: "+"vel.x"+",in y: "+"vel.y"+")");
                #end
            };
        };
        invalidate_wake();
    }
    private function svel_validate(){
        {
            wrap_svel.zpp_inner.x=svelx;
            wrap_svel.zpp_inner.y=svely;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_svel.zpp_inner.x!=wrap_svel.zpp_inner.x));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_svel.zpp_inner.x)"+") :: "+("vec_set(in n: "+"wrap_svel.zpp_inner."+",in x: "+"svelx"+",in y: "+"svely"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_svel.zpp_inner.y!=wrap_svel.zpp_inner.y));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_svel.zpp_inner.y)"+") :: "+("vec_set(in n: "+"wrap_svel.zpp_inner."+",in x: "+"svelx"+",in y: "+"svely"+")");
                #end
            };
        };
    }
    private function force_invalidate(force:ZPP_Vec2){
        #if(!NAPE_RELEASE_BUILD)
        if(!isDynamic())throw "Error: Non-dynamic body cannot have force applied.";
        #end
        {
            forcex=force.x;
            forcey=force.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((forcex!=forcex));
                };
                if(!res)throw "assert("+"!assert_isNaN(forcex)"+") :: "+("vec_set(in n: "+"force"+",in x: "+"force.x"+",in y: "+"force.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((forcey!=forcey));
                };
                if(!res)throw "assert("+"!assert_isNaN(forcey)"+") :: "+("vec_set(in n: "+"force"+",in x: "+"force.x"+",in y: "+"force.y"+")");
                #end
            };
        };
        invalidate_wake();
    }
    private function force_validate(){
        wrap_force.zpp_inner.x=forcex;
        wrap_force.zpp_inner.y=forcey;
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((wrap_force.zpp_inner.x!=wrap_force.zpp_inner.x));
            };
            if(!res)throw "assert("+"!assert_isNaN(wrap_force.zpp_inner.x)"+") :: "+("vec_set(in n: "+"wrap_force.zpp_inner."+",in x: "+"forcex"+",in y: "+"forcey"+")");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((wrap_force.zpp_inner.y!=wrap_force.zpp_inner.y));
            };
            if(!res)throw "assert("+"!assert_isNaN(wrap_force.zpp_inner.y)"+") :: "+("vec_set(in n: "+"wrap_force.zpp_inner."+",in x: "+"forcex"+",in y: "+"forcey"+")");
            #end
        };
    }
    public function setupPosition(){
        wrap_pos=Vec2.get(posx,posy);
        wrap_pos.zpp_inner._inuse=true;
        if(world)wrap_pos.zpp_inner._immutable=true;
        else{
            wrap_pos.zpp_inner._invalidate=pos_invalidate;
            wrap_pos.zpp_inner._validate=pos_validate;
        }
    }
    public function setupVelocity(){
        wrap_vel=Vec2.get(velx,vely);
        wrap_vel.zpp_inner._inuse=true;
        if(world)wrap_vel.zpp_inner._immutable=true;
        else{
            wrap_vel.zpp_inner._invalidate=vel_invalidate;
            wrap_vel.zpp_inner._validate=vel_validate;
        }
    }
    public function setupkinvel(){
        wrap_kinvel=Vec2.get(kinvelx,kinvely);
        wrap_kinvel.zpp_inner._inuse=true;
        if(world)wrap_kinvel.zpp_inner._immutable=true;
        else{
            wrap_kinvel.zpp_inner._invalidate=kinvel_invalidate;
            wrap_kinvel.zpp_inner._validate=kinvel_validate;
        }
    }
    public function setupsvel(){
        wrap_svel=Vec2.get(svelx,svely);
        wrap_svel.zpp_inner._inuse=true;
        if(world)wrap_svel.zpp_inner._immutable=true;
        else{
            wrap_svel.zpp_inner._invalidate=svel_invalidate;
            wrap_svel.zpp_inner._validate=svel_validate;
        }
    }
    public function setupForce(){
        wrap_force=Vec2.get(forcex,forcey);
        wrap_force.zpp_inner._inuse=true;
        if(world)wrap_force.zpp_inner._immutable=true;
        else{
            wrap_force.zpp_inner._invalidate=force_invalidate;
            wrap_force.zpp_inner._validate=force_validate;
        }
    }
    private function cvel_validate(){
        wrapcvel.zpp_inner.x=velx+kinvelx;
        wrapcvel.zpp_inner.y=vely+kinvely;
        wrapcvel.zpp_inner.z=angvel+kinangvel;
    }
    public var wrapcvel:Vec3=null;
    public function setup_cvel(){
        var me=this;
        wrapcvel=Vec3.get();
        wrapcvel.zpp_inner.immutable=true;
        wrapcvel.zpp_inner._validate=cvel_validate;
    }
    public var angvel:Float=0.0;
    public var torque:Float=0.0;
    public var kinangvel:Float=0.0;
    public var pre_rot:Float=0.0;
    public var rot:Float=0.0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function invalidate_rot(){
        zip_axis=true;
        {
            var cx_ite=shapes.begin();
            while(cx_ite!=null){
                var s=cx_ite.elem();
                {
                    if(s.type==ZPP_Flags.id_ShapeType_POLYGON){
                        s.polygon.invalidate_gverts();
                        s.polygon.invalidate_gaxi();
                    }
                    s.invalidate_worldCOM();
                };
                cx_ite=cx_ite.next;
            }
        };
        invalidate_worldCOM();
    }
    public var axisx:Float=0.0;
    public var axisy:Float=0.0;
    public var zip_axis:Bool=false;
    public#if NAPE_NO_INLINE#else inline #end
    function validate_axis(){
        if(zip_axis){
            zip_axis=false;
            quick_validate_axis();
        }
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function quick_validate_axis(){
        {
            axisx=Math.sin(rot);
            axisy=Math.cos(rot);
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((axisx!=axisx));
                };
                if(!res)throw "assert("+"!assert_isNaN(axisx)"+") :: "+("vec_set(in n: "+"axis"+",in x: "+"Math.sin(rot)"+",in y: "+"Math.cos(rot)"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((axisy!=axisy));
                };
                if(!res)throw "assert("+"!assert_isNaN(axisy)"+") :: "+("vec_set(in n: "+"axis"+",in x: "+"Math.sin(rot)"+",in y: "+"Math.cos(rot)"+")");
                #end
            };
        };
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function delta_rot(dr:Float){
        rot+=dr;
        if(dr*dr>0.0001)quick_validate_axis();
        else{
            var d2=dr*dr;
            var p=1-0.5*d2;
            var m=1-d2*d2/8;
            var nx=(p*axisx+dr*axisy)*m;
            axisy=(p*axisy-dr*axisx)*m;
            axisx=nx;
        }
    }
    public var kinematicDelaySleep:Bool;
    public var mass:Float=0.0;
    public var zip_mass:Bool=false;
    public var massMode:Int=0;
    public var imass:Float=0.0;
    public var smass:Float=0.0;
    public var cmass:Float=0.0;
    public var nomove:Bool=false;
    public function invalidate_mass(){
        zip_mass=true;
        invalidate_gravMass();
    }
    public function validate_mass(){
        var exist=false;
        if(zip_mass||(massMode==ZPP_Flags.id_MassMode_DEFAULT&&exist)){
            zip_mass=false;
            if(massMode==ZPP_Flags.id_MassMode_DEFAULT){
                cmass=0;
                {
                    var cx_ite=shapes.begin();
                    while(cx_ite!=null){
                        var s=cx_ite.elem();
                        {
                            s.refmaterial.density=s.material.density;
                            s.validate_area_inertia();
                            cmass+=s.area*s.material.density;
                        };
                        cx_ite=cx_ite.next;
                    }
                };
            }
            if(isDynamic()&&!nomove){
                mass=cmass;
                imass=smass=1.0/mass;
            }
            else{
                mass=ZPP_Const.POSINF();
                imass=smass=0.0;
            }
            if(exist)invalidate_inertia();
        }
    }
    public var gravMass:Float=0.0;
    public var zip_gravMass:Bool=false;
    public var gravMassMode:Int=0;
    public var gravMassScale:Float=0.0;
    public var zip_gravMassScale:Bool=false;
    public function invalidate_gravMass(){
        if(gravMassMode!=ZPP_Flags.id_GravMassMode_FIXED)zip_gravMass=true;
        if(gravMassMode!=ZPP_Flags.id_GravMassMode_SCALED)zip_gravMassScale=true;
        wake();
    }
    public function validate_gravMass(){
        if(zip_gravMass){
            zip_gravMass=false;
            validate_mass();
            if(gravMassMode==ZPP_Flags.id_GravMassMode_DEFAULT){
                validate_mass();
                gravMass=cmass;
            }
            else if(gravMassMode==ZPP_Flags.id_GravMassMode_SCALED){
                validate_mass();
                gravMass=cmass*gravMassScale;
            }
        }
    }
    public function invalidate_gravMassScale(){
        if(gravMassMode!=ZPP_Flags.id_GravMassMode_SCALED)zip_gravMassScale=true;
        else invalidate_gravMass();
    }
    public function validate_gravMassScale(){
        if(zip_gravMassScale){
            zip_gravMassScale=false;
            if(gravMassMode==ZPP_Flags.id_GravMassMode_DEFAULT)gravMassScale=1.0;
            else if(gravMassMode==ZPP_Flags.id_GravMassMode_FIXED){
                validate_mass();
                gravMassScale=gravMass/cmass;
            }
        }
    }
    public var inertiaMode:Int=0;
    public var inertia:Float=0.0;
    public var zip_inertia:Bool=false;
    public var cinertia:Float=0.0;
    public var iinertia:Float=0.0;
    public var sinertia:Float=0.0;
    public var norotate:Bool=false;
    public function invalidate_inertia(){
        zip_inertia=true;
        wake();
    }
    public function validate_inertia(){
        var exist=false;
        if(zip_inertia||(inertiaMode==ZPP_Flags.id_InertiaMode_DEFAULT&&exist)){
            zip_inertia=false;
            if(inertiaMode==ZPP_Flags.id_InertiaMode_DEFAULT){
                cinertia=0;
                {
                    var cx_ite=shapes.begin();
                    while(cx_ite!=null){
                        var s=cx_ite.elem();
                        {
                            s.refmaterial.density=s.material.density;
                            s.validate_area_inertia();
                            cinertia+=s.inertia*s.area*s.material.density;
                        };
                        cx_ite=cx_ite.next;
                    }
                };
            }
            if(isDynamic()&&!norotate){
                inertia=cinertia;
                sinertia=iinertia=1.0/inertia;
            }
            else{
                inertia=ZPP_Const.POSINF();
                sinertia=iinertia=0;
            }
            if(exist)invalidate_inertia();
        }
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function invalidate_wake(){
        wake();
    }
    public var aabb:ZPP_AABB=null;
    public var zip_aabb:Bool=false;
    public#if NAPE_NO_INLINE#else inline #end
    function validate_aabb(){
        #if(!NAPE_RELEASE_BUILD)
        if(shapes.empty())throw "Error: Body bounds only makes sense if it contains shapes";
        #end
        if(zip_aabb){
            zip_aabb=false;
            {
                aabb.minx=ZPP_Const.POSINF();
                aabb.miny=ZPP_Const.POSINF();
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((aabb.minx!=aabb.minx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(aabb.minx)"+") :: "+("vec_set(in n: "+"aabb.min"+",in x: "+"ZPP_Const.POSINF()"+",in y: "+"ZPP_Const.POSINF()"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((aabb.miny!=aabb.miny));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(aabb.miny)"+") :: "+("vec_set(in n: "+"aabb.min"+",in x: "+"ZPP_Const.POSINF()"+",in y: "+"ZPP_Const.POSINF()"+")");
                    #end
                };
            };
            {
                aabb.maxx=ZPP_Const.NEGINF();
                aabb.maxy=ZPP_Const.NEGINF();
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((aabb.maxx!=aabb.maxx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(aabb.maxx)"+") :: "+("vec_set(in n: "+"aabb.max"+",in x: "+"ZPP_Const.NEGINF()"+",in y: "+"ZPP_Const.NEGINF()"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((aabb.maxy!=aabb.maxy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(aabb.maxy)"+") :: "+("vec_set(in n: "+"aabb.max"+",in x: "+"ZPP_Const.NEGINF()"+",in y: "+"ZPP_Const.NEGINF()"+")");
                    #end
                };
            };
            {
                var cx_ite=shapes.begin();
                while(cx_ite!=null){
                    var s=cx_ite.elem();
                    {
                        s.validate_aabb();
                        aabb.combine(s.aabb);
                    };
                    cx_ite=cx_ite.next;
                }
            };
        }
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function invalidate_aabb(){
        zip_aabb=true;
    }
    public var localCOMx:Float=0.0;
    public var localCOMy:Float=0.0;
    public var zip_localCOM:Bool=false;
    public var worldCOMx:Float=0.0;
    public var worldCOMy:Float=0.0;
    public var zip_worldCOM:Bool=false;
    public var wrap_localCOM:Vec2=null;
    public var wrap_worldCOM:Vec2=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function invalidate_localCOM(){
        zip_localCOM=true;
        invalidate_worldCOM();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function invalidate_worldCOM(){
        zip_worldCOM=true;
    }
    public function validate_localCOM(){
        if(zip_localCOM){
            zip_localCOM=false;
            var tempx:Float=0;
            var tempy:Float=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((tempx!=tempx));
                };
                if(!res)throw "assert("+"!assert_isNaN(tempx)"+") :: "+("vec_new(in n: "+"temp"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((tempy!=tempy));
                };
                if(!res)throw "assert("+"!assert_isNaN(tempy)"+") :: "+("vec_new(in n: "+"temp"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            var msum=0.0;
            {
                var cx_ite=shapes.begin();
                while(cx_ite!=null){
                    var s=cx_ite.elem();
                    {
                        s.validate_localCOM();
                        s.validate_area_inertia();
                        {
                            var t=(s.area*s.material.density);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"temp"+",in b: "+"s.localCOM"+",in s: "+"s.area*s.material.density"+")");
                                #end
                            };
                            tempx+=s.localCOMx*t;
                            tempy+=s.localCOMy*t;
                        };
                        msum+=s.area*s.material.density;
                    };
                    cx_ite=cx_ite.next;
                }
            };
            if(msum!=0){
                {
                    var t=(1.0/(msum));
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_mul(in a: "+"temp"+",in s: "+"1.0/(msum)"+",out r: "+"localCOM"+")");
                        #end
                    };
                    localCOMx=tempx*t;
                    localCOMy=tempy*t;
                };
            }
            if(wrap_localCOM!=null){
                wrap_localCOM.zpp_inner.x=localCOMx;
                wrap_localCOM.zpp_inner.y=localCOMy;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((wrap_localCOM.zpp_inner.x!=wrap_localCOM.zpp_inner.x));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(wrap_localCOM.zpp_inner.x)"+") :: "+("vec_set(in n: "+"wrap_localCOM.zpp_inner."+",in x: "+"localCOMx"+",in y: "+"localCOMy"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((wrap_localCOM.zpp_inner.y!=wrap_localCOM.zpp_inner.y));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(wrap_localCOM.zpp_inner.y)"+") :: "+("vec_set(in n: "+"wrap_localCOM.zpp_inner."+",in x: "+"localCOMx"+",in y: "+"localCOMy"+")");
                    #end
                };
            };
            if(zip_mass&&massMode==ZPP_Flags.id_MassMode_DEFAULT){
                zip_mass=false;
                cmass=msum;
                if(isDynamic()){
                    mass=cmass;
                    imass=smass=1.0/mass;
                }
                else{
                    mass=ZPP_Const.POSINF();
                    imass=smass=0.0;
                }
            }
        }
    }
    public function validate_worldCOM(){
        if(zip_worldCOM){
            zip_worldCOM=false;
            validate_localCOM();
            validate_axis();
            {
                worldCOMx=posx+(axisy*localCOMx-axisx*localCOMy);
                worldCOMy=posy+(localCOMx*axisx+localCOMy*axisy);
            };
            if(wrap_worldCOM!=null){
                wrap_worldCOM.zpp_inner.x=worldCOMx;
                wrap_worldCOM.zpp_inner.y=worldCOMy;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((wrap_worldCOM.zpp_inner.x!=wrap_worldCOM.zpp_inner.x));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(wrap_worldCOM.zpp_inner.x)"+") :: "+("vec_set(in n: "+"wrap_worldCOM.zpp_inner."+",in x: "+"worldCOMx"+",in y: "+"worldCOMy"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((wrap_worldCOM.zpp_inner.y!=wrap_worldCOM.zpp_inner.y));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(wrap_worldCOM.zpp_inner.y)"+") :: "+("vec_set(in n: "+"wrap_worldCOM.zpp_inner."+",in x: "+"worldCOMx"+",in y: "+"worldCOMy"+")");
                    #end
                };
            };
        }
    }
    public function getlocalCOM(){
        #if(!NAPE_RELEASE_BUILD)
        if(shapes.empty())throw "Error: localCOM only makes sense when Body has Shapes";
        #end
        validate_localCOM();
    }
    public function getworldCOM(){
        #if(!NAPE_RELEASE_BUILD)
        if(shapes.empty())throw "Error: worldCOM only makes sense when Body has Shapes";
        #end
        validate_worldCOM();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function __immutable_midstep(name:String){
        #if(!NAPE_RELEASE_BUILD)
        if(space!=null&&space.midstep)throw "Error: "+name+" cannot be set during a space step()";
        #end
    }
    public function clear(){
        #if(!NAPE_RELEASE_BUILD)
        if(space!=null)throw "Error: Cannot clear a Body if it is currently being used by a Space!";
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(!constraints.empty())throw "Error: Cannot clear a Body if it is currently being used by a constraint!";
        #end
        {
            while(!shapes.empty()){
                var s=shapes.pop_unsafe();
                {
                    s.removedFromBody();
                    s.body=null;
                };
            }
        };
        invalidate_shapes();
        {
            pre_posx=0;
            pre_posy=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((pre_posx!=pre_posx));
                };
                if(!res)throw "assert("+"!assert_isNaN(pre_posx)"+") :: "+("vec_set(in n: "+"pre_pos"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((pre_posy!=pre_posy));
                };
                if(!res)throw "assert("+"!assert_isNaN(pre_posy)"+") :: "+("vec_set(in n: "+"pre_pos"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        {
            posx=0;
            posy=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((posx!=posx));
                };
                if(!res)throw "assert("+"!assert_isNaN(posx)"+") :: "+("vec_set(in n: "+"pos"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((posy!=posy));
                };
                if(!res)throw "assert("+"!assert_isNaN(posy)"+") :: "+("vec_set(in n: "+"pos"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        {
            velx=0;
            vely=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((velx!=velx));
                };
                if(!res)throw "assert("+"!assert_isNaN(velx)"+") :: "+("vec_set(in n: "+"vel"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((vely!=vely));
                };
                if(!res)throw "assert("+"!assert_isNaN(vely)"+") :: "+("vec_set(in n: "+"vel"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        {
            forcex=0;
            forcey=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((forcex!=forcex));
                };
                if(!res)throw "assert("+"!assert_isNaN(forcex)"+") :: "+("vec_set(in n: "+"force"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((forcey!=forcey));
                };
                if(!res)throw "assert("+"!assert_isNaN(forcey)"+") :: "+("vec_set(in n: "+"force"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        {
            kinvelx=0;
            kinvely=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((kinvelx!=kinvelx));
                };
                if(!res)throw "assert("+"!assert_isNaN(kinvelx)"+") :: "+("vec_set(in n: "+"kinvel"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((kinvely!=kinvely));
                };
                if(!res)throw "assert("+"!assert_isNaN(kinvely)"+") :: "+("vec_set(in n: "+"kinvel"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        {
            svelx=0;
            svely=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((svelx!=svelx));
                };
                if(!res)throw "assert("+"!assert_isNaN(svelx)"+") :: "+("vec_set(in n: "+"svel"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((svely!=svely));
                };
                if(!res)throw "assert("+"!assert_isNaN(svely)"+") :: "+("vec_set(in n: "+"svel"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        angvel=torque=kinangvel=pre_rot=rot=0;
        invalidate_pos();
        invalidate_rot();
        {
            axisx=0;
            axisy=1;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((axisx!=axisx));
                };
                if(!res)throw "assert("+"!assert_isNaN(axisx)"+") :: "+("vec_set(in n: "+"axis"+",in x: "+"0"+",in y: "+"1"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((axisy!=axisy));
                };
                if(!res)throw "assert("+"!assert_isNaN(axisy)"+") :: "+("vec_set(in n: "+"axis"+",in x: "+"0"+",in y: "+"1"+")");
                #end
            };
        };
        zip_axis=false;
        massMode=ZPP_Flags.id_MassMode_DEFAULT;
        gravMassMode=ZPP_Flags.id_GravMassMode_DEFAULT;
        gravMassScale=1.0;
        inertiaMode=ZPP_Flags.id_InertiaMode_DEFAULT;
        norotate=false;
        nomove=false;
    }
    public static function __static():Body{
        var ret=new Body(BodyType.STATIC);
        var si=ret.zpp_inner;
        si.world=true;
        si.wrap_shapes.zpp_inner.immutable=true;
        si.smass=si.imass=si.cmass=si.mass=si.gravMass=0.0;
        si.sinertia=si.iinertia=si.cinertia=si.inertia=0.0;
        si.cbTypes.clear();
        return ret;
    }
    private function aabb_validate(){
        #if(!NAPE_RELEASE_BUILD)
        if(shapes.empty())throw "Error: bounds only makes sense when Body has shapes";
        #end
        validate_aabb();
    }
    private function shapes_adder(s:Shape){
        if(s.zpp_inner.body!=this){
            if(s.zpp_inner.body!=null)s.zpp_inner.body.wrap_shapes.remove(s);
            s.zpp_inner.body=this;
            s.zpp_inner.addedToBody();
            if(space!=null)space.added_shape(s.zpp_inner);
            if(s.zpp_inner.isPolygon()){
                s.zpp_inner.polygon.invalidate_gaxi();
                s.zpp_inner.polygon.invalidate_gverts();
            }
            return true;
        }
        else return false;
    }
    private function shapes_subber(s:Shape){
        if(space!=null)space.removed_shape(s.zpp_inner);
        s.zpp_inner.body=null;
        s.zpp_inner.removedFromBody();
    }
    private function shapes_invalidate(_){
        invalidate_shapes();
    }
    #if(!NAPE_RELEASE_BUILD)
    private function shapes_modifiable(){
        immutable_midstep("Body::shapes");
        if(isStatic()&&space!=null)throw "Error: Cannot modifiy shapes of static object once added to Space";
    }
    #end
    public function new(){
        super();
        ibody=this;
        world=false;
        bulletEnabled=false;
        sweepTime=0;
        sweep_angvel=0;
        norotate=nomove=false;
        disableCCD=false;
        {
            posx=0;
            posy=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((posx!=posx));
                };
                if(!res)throw "assert("+"!assert_isNaN(posx)"+") :: "+("vec_set(in n: "+"pos"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((posy!=posy));
                };
                if(!res)throw "assert("+"!assert_isNaN(posy)"+") :: "+("vec_set(in n: "+"pos"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        rot=0;
        {
            axisx=0;
            axisy=1;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((axisx!=axisx));
                };
                if(!res)throw "assert("+"!assert_isNaN(axisx)"+") :: "+("vec_set(in n: "+"axis"+",in x: "+"0"+",in y: "+"1"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((axisy!=axisy));
                };
                if(!res)throw "assert("+"!assert_isNaN(axisy)"+") :: "+("vec_set(in n: "+"axis"+",in x: "+"0"+",in y: "+"1"+")");
                #end
            };
        };
        {
            svelx=0;
            svely=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((svelx!=svelx));
                };
                if(!res)throw "assert("+"!assert_isNaN(svelx)"+") :: "+("vec_set(in n: "+"svel"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((svely!=svely));
                };
                if(!res)throw "assert("+"!assert_isNaN(svely)"+") :: "+("vec_set(in n: "+"svel"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        {
            velx=0;
            vely=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((velx!=velx));
                };
                if(!res)throw "assert("+"!assert_isNaN(velx)"+") :: "+("vec_set(in n: "+"vel"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((vely!=vely));
                };
                if(!res)throw "assert("+"!assert_isNaN(vely)"+") :: "+("vec_set(in n: "+"vel"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        {
            kinvelx=0;
            kinvely=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((kinvelx!=kinvelx));
                };
                if(!res)throw "assert("+"!assert_isNaN(kinvelx)"+") :: "+("vec_set(in n: "+"kinvel"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((kinvely!=kinvely));
                };
                if(!res)throw "assert("+"!assert_isNaN(kinvely)"+") :: "+("vec_set(in n: "+"kinvel"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        {
            forcex=0;
            forcey=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((forcex!=forcex));
                };
                if(!res)throw "assert("+"!assert_isNaN(forcex)"+") :: "+("vec_set(in n: "+"force"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((forcey!=forcey));
                };
                if(!res)throw "assert("+"!assert_isNaN(forcey)"+") :: "+("vec_set(in n: "+"force"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        torque=angvel=kinangvel=0;
        {
            pre_posx=ZPP_Const.POSINF();
            pre_posy=ZPP_Const.POSINF();
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((pre_posx!=pre_posx));
                };
                if(!res)throw "assert("+"!assert_isNaN(pre_posx)"+") :: "+("vec_set(in n: "+"pre_pos"+",in x: "+"ZPP_Const.POSINF()"+",in y: "+"ZPP_Const.POSINF()"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((pre_posy!=pre_posy));
                };
                if(!res)throw "assert("+"!assert_isNaN(pre_posy)"+") :: "+("vec_set(in n: "+"pre_pos"+",in x: "+"ZPP_Const.POSINF()"+",in y: "+"ZPP_Const.POSINF()"+")");
                #end
            };
        };
        pre_rot=ZPP_Const.POSINF();
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
        {
            worldCOMx=0;
            worldCOMy=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((worldCOMx!=worldCOMx));
                };
                if(!res)throw "assert("+"!assert_isNaN(worldCOMx)"+") :: "+("vec_set(in n: "+"worldCOM"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((worldCOMy!=worldCOMy));
                };
                if(!res)throw "assert("+"!assert_isNaN(worldCOMy)"+") :: "+("vec_set(in n: "+"worldCOM"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        zip_aabb=true;
        aabb=ZPP_AABB.get(0,0,0,0);
        aabb._immutable=true;
        var me=this;
        aabb._validate=aabb_validate;
        massMode=ZPP_Flags.id_MassMode_DEFAULT;
        gravMassMode=ZPP_Flags.id_GravMassMode_DEFAULT;
        gravMassScale=1.0;
        inertiaMode=ZPP_Flags.id_InertiaMode_DEFAULT;
        arbiters=new ZNPList_ZPP_Arbiter();
        constraints=new ZNPList_ZPP_Constraint();
        shapes=new ZNPList_ZPP_Shape();
        wrap_shapes=ZPP_ShapeList.get(shapes);
        wrap_shapes.zpp_inner.adder=shapes_adder;
        wrap_shapes.zpp_inner.subber=shapes_subber;
        wrap_shapes.zpp_inner._invalidate=shapes_invalidate;
        #if(!NAPE_RELEASE_BUILD)
        wrap_shapes.zpp_inner._modifiable=shapes_modifiable;
        #end
        kinematicDelaySleep=false;
    }
    public function addedToSpace(){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                component==null;
            };
            if(!res)throw "assert("+"component==null"+") :: "+("already has a component?");
            #end
        };
        {
            if(ZPP_Component.zpp_pool==null){
                component=new ZPP_Component();
                #if NAPE_POOL_STATS ZPP_Component.POOL_TOT++;
                ZPP_Component.POOL_ADDNEW++;
                #end
            }
            else{
                component=ZPP_Component.zpp_pool;
                ZPP_Component.zpp_pool=component.next;
                component.next=null;
                #if NAPE_POOL_STATS ZPP_Component.POOL_CNT--;
                ZPP_Component.POOL_ADD++;
                #end
            }
            component.alloc();
        };
        component.isBody=true;
        component.body=this;
        __iaddedToSpace();
    }
    public function removedFromSpace(){
        while(!arbiters.empty()){
            var arb=arbiters.pop_unsafe();
            arb.lazyRetire(space,this);
        }
        {
            var o=component;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Component"+", in obj: "+"component"+")");
                #end
            };
            o.free();
            o.next=ZPP_Component.zpp_pool;
            ZPP_Component.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_Component.POOL_CNT++;
            ZPP_Component.POOL_SUB++;
            #end
        };
        component=null;
        __iremovedFromSpace();
    }
    public function copy(){
        var ret=new Body().zpp_inner;
        ret.type=type;
        ret.bulletEnabled=bulletEnabled;
        ret.disableCCD=disableCCD;
        {
            var cx_ite=shapes.begin();
            while(cx_ite!=null){
                var s=cx_ite.elem();
                {
                    ret.outer.shapes.add(s.outer.copy());
                };
                cx_ite=cx_ite.next;
            }
        };
        {
            ret.posx=posx;
            ret.posy=posy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.posx!=ret.posx));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.posx)"+") :: "+("vec_set(in n: "+"ret.pos"+",in x: "+"posx"+",in y: "+"posy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.posy!=ret.posy));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.posy)"+") :: "+("vec_set(in n: "+"ret.pos"+",in x: "+"posx"+",in y: "+"posy"+")");
                #end
            };
        };
        {
            ret.velx=velx;
            ret.vely=vely;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.velx!=ret.velx));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.velx)"+") :: "+("vec_set(in n: "+"ret.vel"+",in x: "+"velx"+",in y: "+"vely"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.vely!=ret.vely));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.vely)"+") :: "+("vec_set(in n: "+"ret.vel"+",in x: "+"velx"+",in y: "+"vely"+")");
                #end
            };
        };
        {
            ret.forcex=forcex;
            ret.forcey=forcey;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.forcex!=ret.forcex));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.forcex)"+") :: "+("vec_set(in n: "+"ret.force"+",in x: "+"forcex"+",in y: "+"forcey"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.forcey!=ret.forcey));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.forcey)"+") :: "+("vec_set(in n: "+"ret.force"+",in x: "+"forcex"+",in y: "+"forcey"+")");
                #end
            };
        };
        ret.rot=rot;
        ret.angvel=angvel;
        ret.torque=torque;
        {
            ret.kinvelx=kinvelx;
            ret.kinvely=kinvely;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.kinvelx!=ret.kinvelx));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.kinvelx)"+") :: "+("vec_set(in n: "+"ret.kinvel"+",in x: "+"kinvelx"+",in y: "+"kinvely"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.kinvely!=ret.kinvely));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.kinvely)"+") :: "+("vec_set(in n: "+"ret.kinvel"+",in x: "+"kinvelx"+",in y: "+"kinvely"+")");
                #end
            };
        };
        ret.kinangvel=kinangvel;
        {
            ret.svelx=svelx;
            ret.svely=svely;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.svelx!=ret.svelx));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.svelx)"+") :: "+("vec_set(in n: "+"ret.svel"+",in x: "+"svelx"+",in y: "+"svely"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.svely!=ret.svely));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.svely)"+") :: "+("vec_set(in n: "+"ret.svel"+",in x: "+"svelx"+",in y: "+"svely"+")");
                #end
            };
        };
        if(!zip_axis){
            ret.axisx=axisx;
            ret.axisy=axisy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.axisx!=ret.axisx));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.axisx)"+") :: "+("vec_set(in n: "+"ret.axis"+",in x: "+"axisx"+",in y: "+"axisy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.axisy!=ret.axisy));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.axisy)"+") :: "+("vec_set(in n: "+"ret.axis"+",in x: "+"axisx"+",in y: "+"axisy"+")");
                #end
            };
        };
        else ret.invalidate_rot();
        ret.rot=rot;
        ret.massMode=massMode;
        ret.gravMassMode=gravMassMode;
        ret.inertiaMode=inertiaMode;
        ret.norotate=norotate;
        ret.nomove=nomove;
        ret.cmass=cmass;
        ret.cinertia=cinertia;
        if(!zip_mass)ret.mass=mass;
        else ret.invalidate_mass();
        if(!zip_gravMass)ret.gravMass=gravMass;
        else ret.invalidate_gravMass();
        if(!zip_gravMassScale)ret.gravMassScale=gravMassScale;
        else ret.invalidate_gravMassScale();
        if(!zip_inertia)ret.inertia=inertia;
        else ret.invalidate_inertia();
        if(!zip_aabb){
            {
                ret.aabb.minx=aabb.minx;
                ret.aabb.miny=aabb.miny;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((ret.aabb.minx!=ret.aabb.minx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(ret.aabb.minx)"+") :: "+("vec_set(in n: "+"ret.aabb.min"+",in x: "+"aabb.minx"+",in y: "+"aabb.miny"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((ret.aabb.miny!=ret.aabb.miny));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(ret.aabb.miny)"+") :: "+("vec_set(in n: "+"ret.aabb.min"+",in x: "+"aabb.minx"+",in y: "+"aabb.miny"+")");
                    #end
                };
            };
            {
                ret.aabb.maxx=aabb.maxx;
                ret.aabb.maxy=aabb.maxy;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((ret.aabb.maxx!=ret.aabb.maxx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(ret.aabb.maxx)"+") :: "+("vec_set(in n: "+"ret.aabb.max"+",in x: "+"aabb.maxx"+",in y: "+"aabb.maxy"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((ret.aabb.maxy!=ret.aabb.maxy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(ret.aabb.maxy)"+") :: "+("vec_set(in n: "+"ret.aabb.max"+",in x: "+"aabb.maxx"+",in y: "+"aabb.maxy"+")");
                    #end
                };
            };
        }
        else ret.invalidate_aabb();
        if(!zip_localCOM){
            ret.localCOMx=localCOMx;
            ret.localCOMy=localCOMy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.localCOMx!=ret.localCOMx));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.localCOMx)"+") :: "+("vec_set(in n: "+"ret.localCOM"+",in x: "+"localCOMx"+",in y: "+"localCOMy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.localCOMy!=ret.localCOMy));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.localCOMy)"+") :: "+("vec_set(in n: "+"ret.localCOM"+",in x: "+"localCOMx"+",in y: "+"localCOMy"+")");
                #end
            };
        };
        else ret.invalidate_localCOM();
        if(!zip_worldCOM){
            ret.worldCOMx=worldCOMx;
            ret.worldCOMy=worldCOMy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.worldCOMx!=ret.worldCOMx));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.worldCOMx)"+") :: "+("vec_set(in n: "+"ret.worldCOM"+",in x: "+"worldCOMx"+",in y: "+"worldCOMy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.worldCOMy!=ret.worldCOMy));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.worldCOMy)"+") :: "+("vec_set(in n: "+"ret.worldCOM"+",in x: "+"worldCOMx"+",in y: "+"worldCOMy"+")");
                #end
            };
        };
        else ret.invalidate_worldCOM();
        copyto(ret.outer);
        return ret.outer;
    }
}

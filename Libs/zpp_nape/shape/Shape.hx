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
class ZPP_Shape extends ZPP_Interactor{
    public var outer:Shape=null;
    public var body:ZPP_Body=null;
    public var type:Int=0;
    public static var types:Array<ShapeType>=[ShapeType.CIRCLE,ShapeType.POLYGON];
    public#if NAPE_NO_INLINE#else inline #end
    function isCircle(){
        return type==ZPP_Flags.id_ShapeType_CIRCLE;
    }
    public#if NAPE_NO_INLINE#else inline #end
    function isPolygon(){
        return type==ZPP_Flags.id_ShapeType_POLYGON;
    }
    public var area:Float=0.0;
    public var zip_area_inertia:Bool=false;
    public var inertia:Float=0.0;
    public var angDrag:Float=0.0;
    public var zip_angDrag:Bool=false;
    public var localCOMx:Float=0.0;
    public var localCOMy:Float=0.0;
    public var zip_localCOM:Bool=false;
    public var worldCOMx:Float=0.0;
    public var worldCOMy:Float=0.0;
    public var zip_worldCOM:Bool=false;
    public var wrap_localCOM:Vec2=null;
    public var wrap_worldCOM:Vec2=null;
    public var sweepRadius:Float=0.0;
    public var zip_sweepRadius:Bool=false;
    public var sweepCoef:Float=0.0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function invalidate_sweepRadius(){
        zip_sweepRadius=true;
    }
    public function validate_sweepRadius(){
        if(zip_sweepRadius){
            zip_sweepRadius=false;
            if(isCircle())circle.__validate_sweepRadius();
            else polygon.__validate_sweepRadius();
        }
    }
    public var circle:ZPP_Circle=null;
    public var polygon:ZPP_Polygon=null;
    public var refmaterial:ZPP_Material=null;
    public var material:ZPP_Material=null;
    public var filter:ZPP_InteractionFilter=null;
    public var fluidProperties:ZPP_FluidProperties=null;
    public var fluidEnabled:Bool=false;
    public var sensorEnabled:Bool=false;
    public var sweep:ZPP_SweepData=null;
    public var node:ZPP_AABBNode=null;
    public var pairs:ZNPList_ZPP_AABBPair=null;
    public function clear(){
        if(isCircle())circle.__clear();
        else polygon.__clear();
    }
    public var aabb:ZPP_AABB=null;
    public var zip_aabb:Bool=false;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function validate_aabb(){
        if(zip_aabb){
            if(body!=null){
                zip_aabb=false;
                if(isCircle())circle.__validate_aabb();
                else polygon.__validate_aabb();
            }
        }
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function force_validate_aabb(){
        if(isCircle())circle._force_validate_aabb();
        else polygon._force_validate_aabb();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function invalidate_aabb(){
        zip_aabb=true;
        if(body!=null)body.invalidate_aabb();
    }
    public function validate_area_inertia(){
        if(zip_area_inertia){
            zip_area_inertia=false;
            if(isCircle())circle.__validate_area_inertia();
            else polygon.__validate_area_inertia();
        }
    }
    public function validate_angDrag(){
        if(zip_angDrag||refmaterial.dynamicFriction!=material.dynamicFriction){
            zip_angDrag=false;
            refmaterial.dynamicFriction=material.dynamicFriction;
            if(isCircle())circle.__validate_angDrag();
            else polygon.__validate_angDrag();
        }
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function validate_localCOM(){
        if(zip_localCOM){
            zip_localCOM=false;
            if(isPolygon())polygon.__validate_localCOM();
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
        }
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function validate_worldCOM(){
        if(zip_worldCOM){
            if(body!=null){
                zip_worldCOM=false;
                validate_localCOM();
                body.validate_axis();
                {
                    worldCOMx=body.posx+(body.axisy*localCOMx-body.axisx*localCOMy);
                    worldCOMy=body.posy+(localCOMx*body.axisx+localCOMy*body.axisy);
                };
            }
        }
    }
    public function getworldCOM(){
        #if(!NAPE_RELEASE_BUILD)
        if(body==null)throw "Error: worldCOM only makes sense when Shape belongs to a Body";
        #end
        validate_worldCOM();
        {
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
    public function invalidate_area_inertia(){
        zip_area_inertia=true;
        if(body!=null){
            body.invalidate_localCOM();
            body.invalidate_mass();
            body.invalidate_inertia();
        }
    }
    public function invalidate_angDrag(){
        zip_angDrag=true;
    }
    public function invalidate_localCOM(){
        zip_localCOM=true;
        invalidate_area_inertia();
        if(isCircle())invalidate_sweepRadius();
        invalidate_angDrag();
        invalidate_worldCOM();
        if(body!=null)body.invalidate_localCOM();
    }
    public function invalidate_worldCOM(){
        zip_worldCOM=true;
        invalidate_aabb();
    }
    public function invalidate_material(flags:Int){
        if((flags&ZPP_Material.WAKE)!=0)wake();
        if((flags&ZPP_Material.ARBITERS)!=0){
            if(body!=null)body.refreshArbiters();
        }
        if((flags&ZPP_Material.PROPS)!=0){
            if(body!=null){
                body.invalidate_localCOM();
                body.invalidate_mass();
                body.invalidate_inertia();
            }
        }
        if((flags&ZPP_Material.ANGDRAG)!=0){
            invalidate_angDrag();
        }
        refmaterial.set(material);
    }
    public function invalidate_filter(){
        wake();
    }
    public function invalidate_fluidprops(){
        if(fluidEnabled)wake();
    }
    private function aabb_validate(){
        #if(!NAPE_RELEASE_BUILD)
        if(body==null)throw "Error: bounds only makes sense when Shape belongs to a Body";
        #end
        validate_aabb();
    }
    function new(type:Int){
        super();
        pairs=new ZNPList_ZPP_AABBPair();
        ishape=this;
        this.type=type;
        aabb=ZPP_AABB.get(0,0,0,0);
        aabb._immutable=true;
        var me=this;
        aabb._validate=aabb_validate;
        zip_area_inertia=zip_angDrag=zip_localCOM=zip_sweepRadius=true;
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
        fluidEnabled=false;
        sensorEnabled=false;
        fluidProperties=null;
        body=null;
        refmaterial=new ZPP_Material();
        sweepRadius=sweepCoef=0;
    }
    public function setMaterial(material:ZPP_Material){
        if(this.material!=material){
            if(body!=null&&body.space!=null){
                if(this.material!=null)this.material.remShape(this);
            }
            this.material=material;
            if(body!=null&&body.space!=null)material.addShape(this);
            wake();
            if(body!=null)body.refreshArbiters();
        }
    }
    public function setFilter(filter:ZPP_InteractionFilter){
        if(this.filter!=filter){
            if(body!=null&&body.space!=null){
                if(this.filter!=null)this.filter.remShape(this);
            }
            this.filter=filter;
            if(body!=null&&body.space!=null)filter.addShape(this);
            wake();
        }
    }
    public function setFluid(fluid:ZPP_FluidProperties){
        if(fluidProperties!=fluid){
            if(body!=null&&body.space!=null){
                if(fluidProperties!=null)fluidProperties.remShape(this);
            }
            fluidProperties=fluid;
            if(body!=null&&body.space!=null)fluid.addShape(this);
            if(fluidEnabled)wake();
        }
    }
    public function __immutable_midstep(name:String){
        #if(!NAPE_RELEASE_BUILD)
        if(body!=null&&body.space!=null&&body.space.midstep)throw "Error: "+name+" cannot be set during a space step()";
        #end
    }
    public function addedToBody(){
        invalidate_worldCOM();
        invalidate_aabb();
    }
    public function removedFromBody(){}
    public function addedToSpace(){
        __iaddedToSpace();
        material.addShape(this);
        filter.addShape(this);
        if(fluidProperties!=null)fluidProperties.addShape(this);
    }
    public function removedFromSpace(){
        __iremovedFromSpace();
        material.remShape(this);
        filter.remShape(this);
        if(fluidProperties!=null)fluidProperties.remShape(this);
    }
    public function copy(){
        var ret:ZPP_Shape=null;
        if(isCircle())ret=circle.__copy();
        else ret=polygon.__copy();
        if(!zip_area_inertia){
            ret.area=area;
            ret.inertia=inertia;
        }
        else ret.invalidate_area_inertia();
        if(!zip_sweepRadius){
            ret.sweepRadius=sweepRadius;
            ret.sweepCoef=sweepCoef;
        }
        else ret.invalidate_sweepRadius();
        if(!zip_angDrag)ret.angDrag=angDrag;
        else ret.invalidate_angDrag();
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
        {
            var o=ret.material;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Material"+", in obj: "+"ret.material"+")");
                #end
            };
            o.free();
            o.next=ZPP_Material.zpp_pool;
            ZPP_Material.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_Material.POOL_CNT++;
            ZPP_Material.POOL_SUB++;
            #end
        };
        {
            var o=ret.filter;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_InteractionFilter"+", in obj: "+"ret.filter"+")");
                #end
            };
            o.free();
            o.next=ZPP_InteractionFilter.zpp_pool;
            ZPP_InteractionFilter.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_InteractionFilter.POOL_CNT++;
            ZPP_InteractionFilter.POOL_SUB++;
            #end
        };
        ret.material=material;
        ret.filter=filter;
        if(fluidProperties!=null)ret.fluidProperties=fluidProperties;
        ret.fluidEnabled=fluidEnabled;
        ret.sensorEnabled=sensorEnabled;
        if(userData!=null)ret.userData=Reflect.copy(userData);
        copyto(ret.outer);
        return ret.outer;
    }
}

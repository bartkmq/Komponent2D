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
import zpp_nape.callbacks.OptionType;
import zpp_nape.phys.Body;
import zpp_nape.callbacks.CbSetPair;
import zpp_nape.callbacks.CbType;
import zpp_nape.callbacks.Callback;
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
class ZPP_CbSet{
    public var cbTypes:ZNPList_ZPP_CbType=null;
    public var count:Int=0;
    public var next:ZPP_CbSet=null;
    static public var zpp_pool:ZPP_CbSet=null;
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
    
    public var id:Int=0;
    public var manager:ZPP_CbSetManager=null;
    public var cbpairs:ZNPList_ZPP_CbSetPair=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function increment():Void{
        count++;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function decrement():Bool{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                count>0;
            };
            if(!res)throw "assert("+"count>0"+") :: "+("decrementing ref.count into negatives??");
            #end
        };
        return(--count)==0;
    }
    public function invalidate_pairs():Void{
        {
            var cx_ite=cbpairs.begin();
            while(cx_ite!=null){
                var cb=cx_ite.elem();
                cb.invalidate();
                cx_ite=cx_ite.next;
            }
        };
    }
    public var listeners:ZNPList_ZPP_InteractionListener=null;
    public var zip_listeners:Bool=false;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function invalidate_listeners():Void{
        zip_listeners=true;
        #if true invalidate_pairs();
        #end
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function validate_listeners():Void{
        if(zip_listeners){
            zip_listeners=false;
            realvalidate_listeners();
        }
    }
    public function realvalidate_listeners(){
        listeners.clear();
        {
            var cx_ite=cbTypes.begin();
            while(cx_ite!=null){
                var cb=cx_ite.elem();
                {
                    var npre=null;
                    var nite=listeners.begin();
                    var cite=cb.listeners.begin();
                    while(cite!=null){
                        var cx=cite.elem();
                        if(nite!=null&&nite.elem()==cx){
                            cite=cite.next;
                            npre=nite;
                            nite=nite.next;
                        }
                        else if(nite==null||ZPP_Listener.setlt(cx,nite.elem())){
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !listeners.has(cx);
                                };
                                if(!res)throw "assert("+"!listeners.has(cx)"+") :: "+("merged list already contains listener");
                                #end
                            };
                            if(#if true true#else!cx.options.excluded(cbTypes)#end
                            &&manager.valid_listener(cx)){
                                npre=listeners.inlined_insert(npre,cx);
                            }
                            cite=cite.next;
                        }
                        else{
                            npre=nite;
                            nite=nite.next;
                        }
                    }
                };
                cx_ite=cx_ite.next;
            }
        };
    }
    public var bodylisteners:ZNPList_ZPP_BodyListener=null;
    public var zip_bodylisteners:Bool=false;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function invalidate_bodylisteners():Void{
        zip_bodylisteners=true;
        #if false invalidate_pairs();
        #end
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function validate_bodylisteners():Void{
        if(zip_bodylisteners){
            zip_bodylisteners=false;
            realvalidate_bodylisteners();
        }
    }
    public function realvalidate_bodylisteners(){
        bodylisteners.clear();
        {
            var cx_ite=cbTypes.begin();
            while(cx_ite!=null){
                var cb=cx_ite.elem();
                {
                    var npre=null;
                    var nite=bodylisteners.begin();
                    var cite=cb.bodylisteners.begin();
                    while(cite!=null){
                        var cx=cite.elem();
                        if(nite!=null&&nite.elem()==cx){
                            cite=cite.next;
                            npre=nite;
                            nite=nite.next;
                        }
                        else if(nite==null||ZPP_Listener.setlt(cx,nite.elem())){
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !bodylisteners.has(cx);
                                };
                                if(!res)throw "assert("+"!bodylisteners.has(cx)"+") :: "+("merged list already contains listener");
                                #end
                            };
                            if(#if false true#else!cx.options.excluded(cbTypes)#end
                            &&manager.valid_listener(cx)){
                                npre=bodylisteners.inlined_insert(npre,cx);
                            }
                            cite=cite.next;
                        }
                        else{
                            npre=nite;
                            nite=nite.next;
                        }
                    }
                };
                cx_ite=cx_ite.next;
            }
        };
    }
    public var conlisteners:ZNPList_ZPP_ConstraintListener=null;
    public var zip_conlisteners:Bool=false;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function invalidate_conlisteners():Void{
        zip_conlisteners=true;
        #if false invalidate_pairs();
        #end
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function validate_conlisteners():Void{
        if(zip_conlisteners){
            zip_conlisteners=false;
            realvalidate_conlisteners();
        }
    }
    public function realvalidate_conlisteners(){
        conlisteners.clear();
        {
            var cx_ite=cbTypes.begin();
            while(cx_ite!=null){
                var cb=cx_ite.elem();
                {
                    var npre=null;
                    var nite=conlisteners.begin();
                    var cite=cb.conlisteners.begin();
                    while(cite!=null){
                        var cx=cite.elem();
                        if(nite!=null&&nite.elem()==cx){
                            cite=cite.next;
                            npre=nite;
                            nite=nite.next;
                        }
                        else if(nite==null||ZPP_Listener.setlt(cx,nite.elem())){
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !conlisteners.has(cx);
                                };
                                if(!res)throw "assert("+"!conlisteners.has(cx)"+") :: "+("merged list already contains listener");
                                #end
                            };
                            if(#if false true#else!cx.options.excluded(cbTypes)#end
                            &&manager.valid_listener(cx)){
                                npre=conlisteners.inlined_insert(npre,cx);
                            }
                            cite=cite.next;
                        }
                        else{
                            npre=nite;
                            nite=nite.next;
                        }
                    }
                };
                cx_ite=cx_ite.next;
            }
        };
    }
    public function validate(){
        
        validate_listeners();
        validate_bodylisteners();
        validate_conlisteners();
    }
    public var interactors:ZNPList_ZPP_Interactor=null;
    public var wrap_interactors:InteractorList=null;
    public var constraints:ZNPList_ZPP_Constraint=null;
    public var wrap_constraints:ConstraintList=null;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function addConstraint(con:ZPP_Constraint){
        constraints.add(con);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function addInteractor(intx:ZPP_Interactor){
        interactors.add(intx);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function remConstraint(con:ZPP_Constraint){
        constraints.remove(con);
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function remInteractor(intx:ZPP_Interactor){
        interactors.remove(intx);
    }
    public static function setlt(a:ZPP_CbSet,b:ZPP_CbSet):Bool{
        var i=a.cbTypes.begin();
        var j=b.cbTypes.begin();
        while(i!=null&&j!=null){
            var ca=i.elem();
            var cb=j.elem();
            if(ZPP_CbType.setlt(ca,cb))return true;
            if(ZPP_CbType.setlt(cb,ca))return false;
            else{
                i=i.next;
                j=j.next;
            }
        }
        return j!=null&&i==null;
    }
    public function new(){
        cbTypes=new ZNPList_ZPP_CbType();
        
        listeners=new ZNPList_ZPP_InteractionListener();
        zip_listeners=true;
        bodylisteners=new ZNPList_ZPP_BodyListener();
        zip_bodylisteners=true;
        conlisteners=new ZNPList_ZPP_ConstraintListener();
        zip_conlisteners=true;
        constraints=new ZNPList_ZPP_Constraint();
        interactors=new ZNPList_ZPP_Interactor();
        id=ZPP_ID.CbSet();
        cbpairs=new ZNPList_ZPP_CbSetPair();
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free():Void{
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                constraints.empty();
            };
            if(!res)throw "assert("+"constraints.empty()"+") :: "+("non-empty constraints");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                interactors.empty();
            };
            if(!res)throw "assert("+"interactors.empty()"+") :: "+("non-empty interactors");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                count==0;
            };
            if(!res)throw "assert("+"count==0"+") :: "+("deallocating with count!=0?");
            #end
        };
        
        listeners.clear();
        zip_listeners=true;
        bodylisteners.clear();
        zip_bodylisteners=true;
        conlisteners.clear();
        zip_conlisteners=true;
        {
            while(!cbTypes.empty()){
                var cb=cbTypes.pop_unsafe();
                cb.cbsets.remove(this);
            }
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                cbpairs.empty();
            };
            if(!res)throw "assert("+"cbpairs.empty()"+") :: "+("non-empty cbpairs");
            #end
        };
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc():Void{}
    #if NAPE_ASSERT public static function assert_cbTypes(cbTypes:ZNPList_ZPP_CbType):Void{
        var pre=null;
        {
            var cx_ite=cbTypes.begin();
            while(cx_ite!=null){
                var cur=cx_ite.elem();
                {
                    if(pre!=null){
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                ZPP_CbType.setlt(pre,cur);
                            };
                            if(!res)throw "assert("+"ZPP_CbType.setlt(pre,cur)"+") :: "+("cbTypes of CbSet not well-ordered!");
                            #end
                        };
                    }
                    pre=cur;
                };
                cx_ite=cx_ite.next;
            }
        };
    }
    #end
    public static function get(cbTypes:ZNPList_ZPP_CbType){
        var ret;
        {
            if(ZPP_CbSet.zpp_pool==null){
                ret=new ZPP_CbSet();
                #if NAPE_POOL_STATS ZPP_CbSet.POOL_TOT++;
                ZPP_CbSet.POOL_ADDNEW++;
                #end
            }
            else{
                ret=ZPP_CbSet.zpp_pool;
                ZPP_CbSet.zpp_pool=ret.next;
                ret.next=null;
                #if NAPE_POOL_STATS ZPP_CbSet.POOL_CNT--;
                ZPP_CbSet.POOL_ADD++;
                #end
            }
            ret.alloc();
        };
        var ite=null;
        #if NAPE_ASSERT assert_cbTypes(cbTypes);
        #end
        {
            var cx_ite=cbTypes.begin();
            while(cx_ite!=null){
                var cb=cx_ite.elem();
                {
                    ite=ret.cbTypes.insert(ite,cb);
                    cb.cbsets.add(ret);
                };
                cx_ite=cx_ite.next;
            }
        };
        return ret;
    }
    #if NAPE_NO_INLINE#else inline #end
    static function compatible(i:ZPP_InteractionListener,a:ZPP_CbSet,b:ZPP_CbSet):Bool{
        return(i.options1.compatible(a.cbTypes)&&i.options2.compatible(b.cbTypes))||(i.options2.compatible(a.cbTypes)&&i.options1.compatible(b.cbTypes));
    }
    public static#if NAPE_NO_INLINE#else inline #end
    function empty_intersection(a:ZPP_CbSet,b:ZPP_CbSet):Bool{
        return a.manager.pair(a,b).empty_intersection();
    }
    public static function single_intersection(a:ZPP_CbSet,b:ZPP_CbSet,i:ZPP_InteractionListener):Bool{
        return a.manager.pair(a,b).single_intersection(i);
    }
    #if NAPE_NO_INLINE#else inline #end
    public static function find_all(a:ZPP_CbSet,b:ZPP_CbSet,event:Int,cb:ZPP_InteractionListener->Void):Void{
        a.manager.pair(a,b).forall(event,cb);
    }
}

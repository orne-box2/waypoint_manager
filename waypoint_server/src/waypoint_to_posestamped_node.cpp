#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace queue_wait {
struct Point { double x, y; };
struct Pose { double x, y, yaw; };
double wrap(double a) { return std::atan2(std::sin(a),std::cos(a)); }
bool finite(Point p) { return std::isfinite(p.x)&&std::isfinite(p.y); }
bool finite(Pose p) { return finite(Point{p.x,p.y})&&std::isfinite(p.yaw); }
Point transform(Point p,Pose t) {
    return {t.x+std::cos(t.yaw)*p.x-std::sin(t.yaw)*p.y,
            t.y+std::sin(t.yaw)*p.x+std::cos(t.yaw)*p.y};
}
struct ScanView {
    Pose origin{};
    double angle_min=0,angle_increment=0,range_min=0,range_max=0;
    std::vector<float> ranges;
    bool clears(Point point,double margin=.05) const {
        if (!finite(origin)||!finite(point)||ranges.size()<2||!std::isfinite(angle_min)||
            !std::isfinite(angle_increment)||angle_increment<=0||!std::isfinite(range_min)||
            !std::isfinite(range_max)||range_min<0||range_max<=range_min||!std::isfinite(margin)||margin<=0) return false;
        double span=angle_increment*(ranges.size()-1);
        if (span>2*M_PI+.001) return false;
        double dx=point.x-origin.x,dy=point.y-origin.y,distance=std::hypot(dx,dy);
        if (distance<=range_min+margin||distance+margin>=range_max) return false;
        if (distance*angle_increment>2*margin) return false;
        double center=angle_min+.5*span;
        double angle=center+wrap(std::atan2(dy,dx)-origin.yaw-center),half=std::atan2(margin,distance);
        double first=(angle-half-angle_min)/angle_increment,last=(angle+half-angle_min)/angle_increment;
        if (first<0||last>ranges.size()-1) return false;
        for (size_t i=std::floor(first);i<=static_cast<size_t>(std::ceil(last));++i) {
            double range=ranges[i];
            if (std::isinf(range)&&range>0) range=range_max;
            if (!std::isfinite(range)||range<range_min||range>range_max||range<=distance+margin) return false;
        }
        return true;
    }
};
struct Projection { double lateral,along,signed_lateral=0; };
struct Path {
    struct Segment { Point a,b; double length,offset; };
    std::vector<Segment> segments;
    double length=0;
    explicit Path(const std::vector<Point>& points) {
        for (auto p:points) if (!finite(p)) throw std::invalid_argument("Non-finite queue path");
        for (size_t i=1;i<points.size();++i) {
            auto a=points[i-1],b=points[i]; double d=std::hypot(b.x-a.x,b.y-a.y);
            if (d>1e-6) { segments.push_back({a,b,d,length}); length+=d; }
        }
        if (segments.empty()) throw std::invalid_argument("Queue path needs two distinct points");
    }
    Projection project(Point p) const {
        Projection best{INFINITY,0};
        for (size_t i=0;i<segments.size();++i) {
            const auto& s=segments[i]; double dx=s.b.x-s.a.x,dy=s.b.y-s.a.y;
            double t=((p.x-s.a.x)*dx+(p.y-s.a.y)*dy)/(s.length*s.length);
            if (i!=0) t=std::max(0.0,t);
            if (i+1!=segments.size()) t=std::min(1.0,t);
            double d=std::hypot(p.x-s.a.x-t*dx,p.y-s.a.y-t*dy);
            if (d<best.lateral) best={d,s.offset+t*s.length,
                (dx*(p.y-s.a.y)-dy*(p.x-s.a.x))/s.length};
        }
        return best;
    }
    Point pointAt(double distance) const {
        for (const auto& s:segments) if (distance<=s.offset+s.length) {
            double t=std::max(0.0,std::min(1.0,(distance-s.offset)/s.length));
            return {s.a.x+t*(s.b.x-s.a.x),s.a.y+t*(s.b.y-s.a.y)};
        }
        return segments.back().b;
    }
    Pose poseAt(double distance,double lateral=0) const {
        for (size_t i=0;i<segments.size();++i) {
            const auto& s=segments[i];
            if (distance<=s.offset+s.length||i+1==segments.size()) {
                double t=std::max(0.0,std::min(1.0,(distance-s.offset)/s.length));
                double dx=(s.b.x-s.a.x)/s.length,dy=(s.b.y-s.a.y)/s.length;
                return {s.a.x+t*(s.b.x-s.a.x)-dy*lateral,
                        s.a.y+t*(s.b.y-s.a.y)+dx*lateral,std::atan2(dy,dx)};
            }
        }
        throw std::logic_error("Empty path");
    }
};
struct Waypoint { Point position; bool stop; };
struct Zone {
    std::string name,stop_id; std::set<std::string> ids; Path path;
    Point line,stop_goal; double line_yaw;
    std::map<std::string,Point> stop_waypoints;
    bool has_stop_line;
};
std::vector<Zone> loadZones(const YAML::Node& config,const std::vector<std::string>& route,
                           const std::map<std::string,Waypoint>& waypoints) {
    if (!config.IsMap()||!config["areas"]||!config["areas"].IsSequence())
        throw std::invalid_argument("Queue configuration areas must be a sequence");
    std::vector<Zone> zones; std::set<std::string> used;
    for (const auto& area:config["areas"]) {
        if (!area["queue_wait"]||!area["queue_wait"].as<bool>()) continue;
        auto start=area["start_id"].as<std::string>(),end=area["end_id"].as<std::string>();
        if (std::count(route.begin(),route.end(),start)!=1||std::count(route.begin(),route.end(),end)!=1)
            throw std::invalid_argument("Queue start_id/end_id must each occur once in the current route");
        auto begin=std::find(route.begin(),route.end(),start)-route.begin();
        auto finish=std::find(route.begin(),route.end(),end)-route.begin();
        if (begin>=finish) throw std::invalid_argument("Queue start_id must precede end_id");
        std::set<std::string> ids; std::map<std::string,Point> stops;
        for (auto i=begin;i<=finish;++i) {
            if (std::count(route.begin(),route.end(),route[i])!=1||!used.insert(route[i]).second)
                throw std::invalid_argument("Queue IDs must be unique; queue areas must not overlap");
            ids.insert(route[i]);
            const auto& waypoint=waypoints.at(route[i]);
            if (waypoint.stop) stops.emplace(route[i],waypoint.position);
        }
        if (!waypoints.at(end).stop) throw std::invalid_argument("Queue end waypoint needs stop: true");
        std::vector<Point> points;
        for (auto i=std::max<decltype(begin)>(0,begin-2);i<=finish;++i)
            points.push_back(waypoints.at(route[i]).position);
        const auto line=area["stop_line"];
        bool has_stop_line=line&&!line.IsNull();
        Point p{},goal=waypoints.at(end).position; double yaw=0;
        if (has_stop_line) {
            if (!line.IsMap()) throw std::invalid_argument("stop_line must contain point and yaw");
            auto point=line["point"];
            if (!point.IsSequence()||point.size()!=2) throw std::invalid_argument("stop_line.point needs [x,y]");
            p={point[0].as<double>(),point[1].as<double>()}; yaw=line["yaw"].as<double>();
            if (!finite(p)||!std::isfinite(yaw)) throw std::invalid_argument("Non-finite stop line");
            auto signedDistance=[&](Point v) { return (v.x-p.x)*std::cos(yaw)+(v.y-p.y)*std::sin(yaw); };
            auto approach=waypoints.at(route[std::max<decltype(begin)>(0,begin-1)]).position;
            if (signedDistance(approach)>=0||signedDistance(goal)>=0)
                throw std::invalid_argument("Approach and stop goal must be before the stop line");
        }
        zones.push_back({area["name"].as<std::string>(),end,ids,Path(points),p,goal,yaw,std::move(stops),has_stop_line});
    }
    return zones;
}
struct Grid {
    int width,height; double resolution; Pose origin; std::vector<int8_t> data;
    bool inflated_costmap;
    Grid(int w,int h,double r,Pose o,const std::vector<int8_t>& d,bool inflated=false)
        :width(w),height(h),resolution(r),origin(o),data(d),inflated_costmap(inflated) {
        if (w<=0||h<=0||!std::isfinite(r)||r<=0||!finite(o)||d.size()!=static_cast<size_t>(w)*h)
            throw std::invalid_argument("Invalid occupancy grid");
    }
    bool occupiedNear(Point point,double margin) const {
        double dx=point.x-origin.x,dy=point.y-origin.y,c=std::cos(origin.yaw),s=std::sin(origin.yaw);
        double x=(c*dx+s*dy)/resolution,y=(-s*dx+c*dy)/resolution,radius=margin/resolution;
        int left=std::max(0,static_cast<int>(std::floor(x-radius))),right=std::min(width-1,static_cast<int>(std::floor(x+radius)));
        int bottom=std::max(0,static_cast<int>(std::floor(y-radius))),top=std::min(height-1,static_cast<int>(std::floor(y+radius)));
        for (int row=bottom;row<=top;++row) for (int col=left;col<=right;++col) {
            if (data[row*width+col]<99) continue;
            double ax=std::max(0.0,std::max(col-x,x-col-1)),ay=std::max(0.0,std::max(row-y,y-row-1));
            if (ax*ax+ay*ay<=radius*radius) return true;
        }
        return false;
    }
    bool observedFree(Point point,double margin=.05) const {
        if (!finite(point)||!std::isfinite(margin)||margin<=0) return false;
        double dx=point.x-origin.x,dy=point.y-origin.y,c=std::cos(origin.yaw),s=std::sin(origin.yaw);
        double x=(c*dx+s*dy)/resolution,y=(-s*dx+c*dy)/resolution,radius=margin/resolution;
        if (!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(radius)) return false;
        if (x-radius<0||y-radius<0||x+radius>=width||y+radius>=height) return false;
        for (int row=std::floor(y-radius);row<=std::floor(y+radius);++row)
            for (int col=std::floor(x-radius);col<=std::floor(x+radius);++col) {
                double ax=std::max(0.0,std::max(col-x,x-col-1)),ay=std::max(0.0,std::max(row-y,y-row-1));
                if (ax*ax+ay*ay>radius*radius) continue;
                auto cost=data[row*width+col];
                if (cost<0||cost>=99) return false;
            }
        return true;
    }
    bool free(Pose pose,const std::vector<Point>& footprint) const {
        double minx=INFINITY,miny=INFINITY,maxx=-INFINITY,maxy=-INFINITY;
        std::vector<Point> polygon;
        for (auto vertex:footprint) {
            auto p=transform(vertex,pose); double dx=p.x-origin.x,dy=p.y-origin.y;
            double x=(std::cos(origin.yaw)*dx+std::sin(origin.yaw)*dy)/resolution;
            double y=(-std::sin(origin.yaw)*dx+std::cos(origin.yaw)*dy)/resolution;
            minx=std::min(minx,x); maxx=std::max(maxx,x); miny=std::min(miny,y); maxy=std::max(maxy,y);
            polygon.push_back({x,y});
        }
        if (minx<0||miny<0||maxx>=width||maxy>=height) return false;
        struct Axis { double x,y,low,high; };
        std::vector<Axis> axes{{1,0,minx,maxx},{0,1,miny,maxy}};
        for (size_t i=0;i<polygon.size();++i) {
            auto a=polygon[i],b=polygon[(i+1)%polygon.size()];
            double ax=a.y-b.y,ay=b.x-a.x,low=INFINITY,high=-INFINITY;
            if (std::hypot(ax,ay)<1e-12) continue;
            for (auto p:polygon) {
                double projection=ax*p.x+ay*p.y;
                low=std::min(low,projection); high=std::max(high,projection);
            }
            axes.push_back({ax,ay,low,high});
        }
        for (int y=std::floor(miny);y<=std::floor(maxy);++y)
            for (int x=std::floor(minx);x<=std::floor(maxx);++x) {
                if (data[y*width+x]>=0&&data[y*width+x]<(inflated_costmap?100:99)) continue;
                bool separated=false;
                for (auto axis:axes) {
                    double center=axis.x*(x+.5)+axis.y*(y+.5);
                    double radius=.5*(std::abs(axis.x)+std::abs(axis.y));
                    if (center+radius<axis.low||center-radius>axis.high) { separated=true; break; }
                }
                if (!separated) return false;
            }
        return true;
    }
};
struct Decision { std::string state; bool active=false,hold=false; double linear=0,angular=0; };
struct ObstacleTrackingSettings {
    double avoidance_stationary_duration=1.0;
    double obstacle_stationary_max_speed=0.08;
    double obstacle_cluster_distance=0.18;
    double obstacle_association_distance=0.35;
    double obstacle_stationary_distance=0.08;
    double obstacle_motion_interval=0.5;
};
class Controller {
public:
    const ObstacleTrackingSettings obstacle_tracking{};
    std::map<std::string,double> p{
        {"max_speed",.3},{"max_angular",.4},{"lookahead",.7},
        {"detection_distance",1},{"detection_half_width",.8},
        {"resume_distance",1.3},{"clear_duration",2},
        {"scan_timeout",.5},{"pose_timeout",.5},{"map_timeout",1.5},
        {"max_lateral_deviation",.25},{"corridor_half_width",.65},
        {"deceleration",.3},{"stop_margin",.2},{"stop_goal_radius",.2},
        {"prediction_time",1},{"avoidance_wait",90},
        {"avoidance_max_offset",0},
        {"avoidance_obstacle_max_diagonal",.3},{"queue_center_half_width",.12}};
    static constexpr double control_rate=10;
    std::vector<Zone> zones; std::vector<Point> footprint;
    std::vector<Point> avoidance_footprint;
    std::function<bool(Point)> static_obstacle;
    std::function<bool(Point)> observed_free;
    int zone=-1;
    bool fault=false,released=false,waiting=true,line_reached=false,waypoint_reached=false;
    double clear_since=NAN,last_time=NAN;
    std::string identity; Decision last{"NORMAL"};
    struct Track {
        size_t id;
        Point center,velocity,anchor;
        std::vector<Point> points;
        double first_seen,last_seen,still_since,outside_since;
        unsigned observations;
        Point motion_center{};
        std::vector<Point> motion_points;
        double motion_stamp=NAN;
    };
    struct Bounds { double first,last,left,right; };
    std::vector<Track> tracks;
    std::map<size_t,double> queue_limits;
    struct QueueMemory {
        std::vector<Point> points;
        double occupied_stamp=NAN,clear_since=NAN,last_scan=NAN,last_map=NAN;
    };
    std::map<size_t,QueueMemory> queue_memory;
    std::unique_ptr<Path> detour;
    size_t next_track=1,blocked_track=0,detour_track=0;
    double observed_scan=NAN,blocked_since=NAN,blocked_tick=NAN,avoidance_clear=NAN;
    Point blocked_position{};
    double detour_end=0;
    double next_avoidance_try=NAN;
    bool oncoming_seen=false;
    Controller(std::vector<Zone> z,const YAML::Node& parameters,std::vector<Point> f):zones(std::move(z)),footprint(std::move(f)) {
        if (parameters&&!parameters.IsNull()&&!parameters.IsMap()) throw std::invalid_argument("queue_wait parameters must be a map");
        for (const auto& value:parameters) {
            auto key=value.first.as<std::string>();
            if (!p.count(key)) throw std::invalid_argument("Unknown queue parameter: "+key);
            p[key]=value.second.as<double>();
        }
        for (const auto& value:p) if (!std::isfinite(value.second)||value.second<0||
            (value.second==0&&value.first!="avoidance_max_offset"))
            throw std::invalid_argument("Queue parameters must be finite and positive");
        for (double value:{obstacle_tracking.avoidance_stationary_duration,obstacle_tracking.obstacle_stationary_max_speed,
                           obstacle_tracking.obstacle_cluster_distance,obstacle_tracking.obstacle_association_distance,
                           obstacle_tracking.obstacle_stationary_distance,obstacle_tracking.obstacle_motion_interval})
            if (!std::isfinite(value)||value<=0)
                throw std::invalid_argument("Obstacle tracking settings must be finite and positive");
        if (p["resume_distance"]<=p["detection_distance"]) throw std::invalid_argument("resume_distance must exceed detection_distance");
        if (footprint.size()<3) throw std::invalid_argument("Footprint needs at least three vertices");
        for (auto v:footprint) if (!finite(v)) throw std::invalid_argument("Non-finite footprint");
        double min_x=INFINITY,max_x=-INFINITY,min_y=INFINITY,max_y=-INFINITY;
        for (auto v:footprint) {
            min_x=std::min(min_x,v.x); max_x=std::max(max_x,v.x);
            min_y=std::min(min_y,v.y); max_y=std::max(max_y,v.y);
        }
        avoidance_footprint={{max_x+.05,max_y+.05},{min_x-.05,max_y+.05},
                             {min_x-.05,min_y-.05},{max_x+.05,min_y-.05}};
        if (p["avoidance_max_offset"]>=p["corridor_half_width"])
            throw std::invalid_argument("avoidance_max_offset must be smaller than corridor_half_width");
        if (p["avoidance_max_offset"]>0&&p["queue_center_half_width"]>=p["corridor_half_width"])
            throw std::invalid_argument("queue_center_half_width must be smaller than corridor_half_width when avoidance is enabled");
    }
    void resetBlocked() { blocked_since=blocked_tick=next_avoidance_try=NAN; blocked_track=0; oncoming_seen=false; }
    void resetAvoidance() {
        resetBlocked(); detour.reset(); detour_track=0; tracks.clear(); queue_limits.clear(); queue_memory.clear();
        observed_scan=avoidance_clear=NAN;
    }
    void waypoint(const std::string& id) {
        if (zone>=0&&!zones[zone].ids.count(id)) {
            if (!released) { fault=true; return; }
            zone=-1; released=false; line_reached=false; resetAvoidance();
        }
        if (zone<0) {
            for (size_t i=0;i<zones.size();++i) if (zones[i].ids.count(id)) { zone=i; break; }
            waiting=false; clear_since=NAN; last_time=NAN; resetAvoidance();
        }
        if (id!=identity) { waypoint_reached=false; resetBlocked(); }
        identity=id;
    }
    void resetClearProofs() {
        for (auto& memory:queue_memory) memory.second.clear_since=memory.second.last_scan=memory.second.last_map=NAN;
    }
    void pause() { waiting=true; clear_since=avoidance_clear=NAN; resetBlocked(); resetClearProofs(); }
    const Path& drivingPath() const { return detour?*detour:zones.at(zone).path; }
    bool entryReady(const Zone& area,Pose pose) const {
        if (!finite(pose)||area.path.project({pose.x,pose.y}).lateral>.8*p.at("max_lateral_deviation")) return false;
        for (auto vertex:footprint)
            if (area.path.project(transform(vertex,pose)).lateral>p.at("corridor_half_width")) return false;
        return true;
    }
    double approachTime() const {
        return p.at("prediction_time")+p.at("max_speed")/p.at("deceleration")+.1;
    }
    struct ObstacleWindow { double start,front,end,stop_end,half_width; };
    ObstacleWindow obstacleWindow(Pose pose) const {
        const auto& path=drivingPath();
        double start=path.project({pose.x,pose.y}).along,front=-INFINITY;
        for (auto v:footprint) front=std::max(front,path.project(transform(v,pose)).along);
        double stop_end=front+p.at("detection_distance");
        double preview=std::max(p.at("resume_distance"),p.at("detection_distance")+p.at("max_speed")*approachTime());
        preview+=p.at("max_speed")*(p.at("scan_timeout")+1/control_rate);
        double width=p.at("detection_half_width");
        if (detour) {
            width=.05;
            for (auto v:footprint) width=std::max(width,std::abs(v.y)+.05);
        }
        return {start,front,front+preview,stop_end,width};
    }
    double clearance(Pose pose,const std::vector<Point>& points) const {
        const auto& path=drivingPath();
        const auto window=obstacleWindow(pose); double closest=INFINITY;
        for (auto point:points) {
            auto q=path.project(point);
            if (q.lateral<=window.half_width&&q.along>=window.start&&q.along<=window.end)
                closest=std::min(closest,q.along-window.front);
        }
        return closest;
    }
    double boundary(Pose pose) const {
        const auto& z=zones[zone]; double closest=INFINITY;
        if (!z.has_stop_line) return closest;
        for (auto v:footprint) {
            auto q=transform(v,pose);
            closest=std::min(closest,(z.line.x-q.x)*std::cos(z.line_yaw)+(z.line.y-q.y)*std::sin(z.line_yaw));
        }
        return closest;
    }
    Bounds bounds(const Track& track) const {
        Bounds result{INFINITY,-INFINITY,INFINITY,-INFINITY};
        for (auto point:track.points) {
            auto q=zones[zone].path.project(point);
            result.first=std::min(result.first,q.along); result.last=std::max(result.last,q.along);
            result.left=std::min(result.left,q.signed_lateral); result.right=std::max(result.right,q.signed_lateral);
        }
        return result;
    }
    static double radius(const std::vector<Point>& points,Point center) {
        double value=0;
        for (auto point:points) value=std::max(value,std::hypot(point.x-center.x,point.y-center.y));
        return value;
    }
    void updateMotion(Track& track,Point center,const std::vector<Point>& points,double stamp) {
        double elapsed=stamp-track.motion_stamp;
        if (elapsed<obstacle_tracking.obstacle_motion_interval) return;
        Point measured{};
        double old_radius=radius(track.motion_points,track.motion_center),new_radius=radius(points,center);
        bool stable=points.size()>=3&&track.motion_points.size()>=3&&
            points.size()>=.4*track.motion_points.size()&&points.size()<=2.5*track.motion_points.size()&&
            std::max(old_radius,new_radius)<=.75&&std::abs(new_radius-old_radius)<.05+.3*old_radius;
        if (stable) {
            Point shift{center.x-track.motion_center.x,center.y-track.motion_center.y};
            std::vector<double> original_errors,shifted_errors;
            for (auto point:points) {
                double nearest=INFINITY,shifted=INFINITY;
                for (auto before:track.motion_points) {
                    double dx=point.x-before.x,dy=point.y-before.y;
                    nearest=std::min(nearest,dx*dx+dy*dy);
                    shifted=std::min(shifted,(dx-shift.x)*(dx-shift.x)+(dy-shift.y)*(dy-shift.y));
                }
                original_errors.push_back(std::sqrt(nearest)); shifted_errors.push_back(std::sqrt(shifted));
            }
            auto upperError=[](std::vector<double>& values) {
                auto sample=values.begin()+3*values.size()/4;
                std::nth_element(values.begin(),sample,values.end()); return *sample;
            };
            double before=upperError(original_errors),after=upperError(shifted_errors);
            if (std::hypot(shift.x,shift.y)>=.035&&after<=.04&&before-after>=.02)
                measured={shift.x/elapsed,shift.y/elapsed};
        }
        track.velocity=measured;
        track.motion_center=center; track.motion_points=points; track.motion_stamp=stamp;
    }
    void rememberLimit(size_t id,double limit,const std::vector<Point>& points,double stamp) {
        queue_limits[id]=limit;
        auto& memory=queue_memory[id]; memory=QueueMemory{}; memory.occupied_stamp=stamp;
        for (auto point:points)
            if (zones[zone].path.project(point).lateral<=p.at("queue_center_half_width")) memory.points.push_back(point);
        if (memory.points.empty()) memory.points=points;
    }
    void releaseClearedLimits(double scan_stamp,double map_stamp) {
        for (auto it=queue_memory.begin();it!=queue_memory.end();) {
            if (!queue_limits.count(it->first)) { it=queue_memory.erase(it); continue; }
            auto& memory=it->second;
            bool clear=observed_free&&!memory.points.empty()&&scan_stamp>memory.occupied_stamp&&map_stamp>memory.occupied_stamp;
            if (clear) for (auto point:memory.points) if (!observed_free(point)) { clear=false; break; }
            if (!clear) memory.clear_since=memory.last_scan=memory.last_map=NAN;
            else {
                double stamp=std::min(scan_stamp,map_stamp);
                if (!std::isfinite(memory.clear_since)||scan_stamp<memory.last_scan||map_stamp<memory.last_map||
                    scan_stamp-memory.last_scan>p.at("scan_timeout")||map_stamp-memory.last_map>p.at("map_timeout"))
                    memory.clear_since=stamp;
                memory.last_scan=scan_stamp; memory.last_map=map_stamp;
                if (stamp-memory.clear_since>=p.at("clear_duration")) {
                    queue_limits.erase(it->first); it=queue_memory.erase(it); continue;
                }
            }
            ++it;
        }
    }
    void observe(const std::vector<Point>& points,double stamp,Pose pose) {
        if (std::isfinite(observed_scan)&&stamp<=observed_scan) return;
        observed_scan=stamp;
        const auto& reference=zones[zone].path;
        const double core=p.at("queue_center_half_width");
        double start=reference.project({pose.x,pose.y}).along,front=-INFINITY,rear=INFINITY;
        for (auto vertex:footprint) {
            double along=reference.project(transform(vertex,pose)).along;
            front=std::max(front,along); rear=std::min(rear,along);
        }
        std::vector<std::vector<Point>> clusters;
        for (auto point:points) {
            if (!finite(point)) continue;
            if (static_obstacle&&static_obstacle(point)) {
                auto q=reference.project(point);
                if (std::abs(q.signed_lateral)<=core&&q.along>=front&&q.along<=front+6) {
                    auto found=queue_limits.find(0);
                    double limit=q.along-p.at("stop_margin");
                    if (found==queue_limits.end()||limit<=found->second) rememberLimit(0,limit,{point},stamp);
                }
                continue;
            }
            if (clusters.empty()||std::hypot(point.x-clusters.back().back().x,point.y-clusters.back().back().y)>obstacle_tracking.obstacle_cluster_distance)
                clusters.push_back({});
            clusters.back().push_back(point);
        }
        std::set<size_t> matched;
        for (auto& cluster:clusters) {
            Point center{};
            for (auto point:cluster) { center.x+=point.x; center.y+=point.y; }
            center.x/=cluster.size(); center.y/=cluster.size();
            size_t selected=tracks.size(); double nearest=obstacle_tracking.obstacle_association_distance;
            for (size_t i=0;i<tracks.size();++i) {
                const auto& t=tracks[i]; double dt=stamp-t.last_seen;
                if (matched.count(t.id)||dt<=0||dt>2) continue;
                double d=std::hypot(center.x-t.center.x-t.velocity.x*dt,center.y-t.center.y-t.velocity.y*dt);
                if (d<nearest) { nearest=d; selected=i; }
            }
            if (selected==tracks.size()) {
                tracks.push_back({next_track++,center,{0,0},center,std::move(cluster),stamp,stamp,stamp,NAN,1,center,{},stamp});
                auto& created=tracks.back();
                created.motion_center=center; created.motion_points=created.points; created.motion_stamp=stamp;
                matched.insert(tracks.back().id);
            } else {
                auto& t=tracks[selected];
                updateMotion(t,center,cluster,stamp);
                if (std::hypot(center.x-t.anchor.x,center.y-t.anchor.y)>obstacle_tracking.obstacle_stationary_distance) {
                    t.still_since=stamp; t.anchor=center;
                }
                t.center=center; t.points=std::move(cluster); t.last_seen=stamp; ++t.observations;
                matched.insert(t.id);
            }
        }
        for (auto& t:tracks) if (t.last_seen==stamp) {
            auto b=bounds(t);
            bool outside=b.left>p.at("corridor_half_width")||b.right<-p.at("corridor_half_width");
            if (!outside) t.outside_since=NAN;
            else if (!std::isfinite(t.outside_since)) t.outside_since=stamp;
            if (b.left<=core&&b.right>=-core&&b.last>=front&&b.first<=front+6)
                rememberLimit(t.id,b.first-p.at("stop_margin"),t.points,stamp);
            else if (b.last<rear-.1||
                (std::isfinite(t.outside_since)&&stamp-t.outside_since>=p.at("clear_duration")))
                queue_limits.erase(t.id);
        }
        for (auto i=queue_limits.begin();i!=queue_limits.end();)
            if (i->second<start-.5) i=queue_limits.erase(i); else ++i;
        tracks.erase(std::remove_if(tracks.begin(),tracks.end(),[&](const Track& t) {
            return stamp-t.last_seen>2;
        }),tracks.end());
    }
    bool pointClear(Pose pose,Point point,double margin) const {
        double dx=point.x-pose.x,dy=point.y-pose.y,c=std::cos(pose.yaw),s=std::sin(pose.yaw);
        Point local{c*dx+s*dy,-s*dx+c*dy};
        bool inside=false; double distance=INFINITY;
        for (size_t i=0,j=footprint.size()-1;i<footprint.size();j=i++) {
            auto a=footprint[j],b=footprint[i];
            if ((a.y>local.y)!=(b.y>local.y)&&local.x<(b.x-a.x)*(local.y-a.y)/(b.y-a.y)+a.x) inside=!inside;
            double ax=b.x-a.x,ay=b.y-a.y,length=ax*ax+ay*ay;
            double k=length>0?std::max(0.0,std::min(1.0,((local.x-a.x)*ax+(local.y-a.y)*ay)/length)):0;
            distance=std::min(distance,std::hypot(local.x-a.x-k*ax,local.y-a.y-k*ay));
        }
        return !inside&&distance>margin;
    }
    bool poseClear(Pose pose,const std::vector<Point>& scan,
                   const std::function<bool(Pose,const std::vector<Point>&)>& collision) const {
        const auto& path=zones[zone].path;
        auto q=path.project({pose.x,pose.y});
        if (q.lateral>std::max(p.at("max_lateral_deviation"),p.at("avoidance_max_offset")+.02)||
            q.along>path.length||boundary(pose)<p.at("stop_margin")||!collision(pose,avoidance_footprint)) return false;
        double limit=INFINITY;
        for (const auto& value:queue_limits) limit=std::min(limit,value.second);
        for (auto vertex:avoidance_footprint) {
            auto v=path.project(transform(vertex,pose));
            if (v.lateral>p.at("corridor_half_width")||v.along>limit) return false;
        }
        for (auto point:scan) if (!pointClear(pose,point,.05)) return false;
        return true;
    }
    bool movingClear(Pose pose,double seconds,double now) const {
        for (const auto& t:tracks) {
            double speed=std::hypot(t.velocity.x,t.velocity.y);
            if (t.observations<3||speed<obstacle_tracking.obstacle_stationary_max_speed||now-t.last_seen>2) continue;
            double extent=radius(t.points,t.center);
            if (extent>.75) continue;
            double dt=std::min(4.0,seconds)+std::max(0.0,now-t.last_seen);
            Point future{t.center.x+t.velocity.x*dt,t.center.y+t.velocity.y*dt};
            if (!pointClear(pose,future,std::max(.25,extent)+.05+.03*dt)) return false;
        }
        return true;
    }
    const Track* blocker(Pose pose,double gap) const {
        auto window=obstacleWindow(pose);
        for (const auto& t:tracks) {
            if (observed_scan-t.last_seen>.05) continue;
            auto b=bounds(t);
            if (b.right<-window.half_width||b.left>window.half_width||b.last<window.start) continue;
            if (std::abs(b.first-window.front-gap)<.03) return &t;
        }
        return nullptr;
    }
    bool makeDetour(Pose pose,const Track& obstacle,double target_end,double now,
                    const std::vector<Point>& scan,
                    const std::function<bool(Pose,const std::vector<Point>&)>& collision) {
        oncoming_seen=false;
        auto b=bounds(obstacle);
        if (now-obstacle.last_seen>p.at("scan_timeout")||now-obstacle.still_since<obstacle_tracking.avoidance_stationary_duration||
            std::hypot(obstacle.velocity.x,obstacle.velocity.y)>obstacle_tracking.obstacle_stationary_max_speed||
            std::hypot(b.last-b.first,b.right-b.left)>p.at("avoidance_obstacle_max_diagonal")||
            !(b.left>p.at("queue_center_half_width")||b.right<-p.at("queue_center_half_width"))) return false;
        const auto& reference=zones[zone].path; auto start=reference.project({pose.x,pose.y});
        double rear=0;
        for (auto v:footprint) rear=std::max(rear,-v.x);
        double pass=std::max(start.along+.8,b.last+rear+.15),finish=pass+.8;
        for (const auto& stop:zones[zone].stop_waypoints) {
            double along=reference.project(stop.second).along;
            if (along>start.along+.05) target_end=std::min(target_end,along);
        }
        if (finish>target_end-.3) return false;
        for (double scale:{1.0,.5}) {
            double offset=(b.left>0?-1:1)*scale*p.at("avoidance_max_offset");
            std::vector<Point> points;
            auto smooth=[](double t) { t=std::max(0.0,std::min(1.0,t)); return t*t*(3-2*t); };
            for (double along=start.along;along<finish+.02;along+=.04) {
                double s=std::min(along,finish),lateral=offset;
                if (s<start.along+.8) lateral=start.signed_lateral+(offset-start.signed_lateral)*smooth((s-start.along)/.8);
                else if (s>pass) lateral=offset*(1-smooth((s-pass)/.8));
                auto point=reference.poseAt(s,lateral); points.push_back({point.x,point.y});
            }
            auto end=reference.poseAt(finish); points.push_back({end.x,end.y});
            auto candidate=std::unique_ptr<Path>(new Path(points));
            bool valid=true;
            for (double s=0;s<=candidate->length+.02;s+=.04) {
                auto prediction=candidate->poseAt(std::min(s,candidate->length));
                if (!movingClear(prediction,s/p.at("max_speed"),now)) { oncoming_seen=true; valid=false; break; }
                if (!poseClear(prediction,scan,collision)) { valid=false; break; }
            }
            if (valid) {
                detour=std::move(candidate); detour_end=finish; detour_track=obstacle.id; waiting=false;
                avoidance_clear=now-p.at("clear_duration"); return true;
            }
        }
        return false;
    }
    Decision blockedDetour(double now,Pose pose,const std::vector<Point>& scan,
                           const std::function<bool(Pose,const std::vector<Point>&)>& collision,double target_end) {
        avoidance_clear=NAN; waiting=true;
        if (!std::isfinite(next_avoidance_try)||now>=next_avoidance_try) {
            next_avoidance_try=now+.5;
            for (const auto& track:tracks) if (track.id==detour_track&&now-track.last_seen<=p.at("scan_timeout")) {
                if (makeDetour(pose,track,target_end,now,scan,collision)) {
                    waiting=true; avoidance_clear=NAN; return hold("AVOIDANCE_WAIT_CLEAR");
                }
                break;
            }
        }
        return hold(oncoming_seen?"ONCOMING_YIELD":"AVOIDANCE_BLOCKED");
    }
    Decision followDetour(double now,Pose pose,const std::vector<Point>& scan,
                          const std::function<bool(Pose,const std::vector<Point>&)>& collision,double target_end) {
        auto projection=detour->project({pose.x,pose.y});
        for (double s=std::max(0.0,projection.along);s<=detour->length+.02;s+=.04) {
            auto prediction=detour->poseAt(std::min(s,detour->length));
            if (!poseClear(prediction,scan,collision)) return blockedDetour(now,pose,scan,collision,target_end);
            if (!movingClear(prediction,(s-projection.along)/p.at("max_speed"),now)) {
                avoidance_clear=NAN; waiting=true; return hold("ONCOMING_YIELD");
            }
        }
        if (waiting) {
            if (!std::isfinite(avoidance_clear)) avoidance_clear=now;
            if (now-avoidance_clear<p.at("clear_duration")) return hold("AVOIDANCE_WAIT_CLEAR");
            waiting=false;
        }
        auto target=detour->pointAt(std::min(detour->length,projection.along+.35));
        double error=wrap(std::atan2(target.y-pose.y,target.x-pose.x)-pose.yaw);
        double speed=p.at("max_speed")*std::max(0.0,std::cos(error));
        double remaining=detour->length-projection.along;
        speed=std::min(speed,std::sqrt(2*p.at("deceleration")*std::max(0.0,remaining)));
        if (std::abs(error)>.6) speed=0;
        double angular=0;
        bool moving_conflict=false;
        auto feasible=[&](double candidate) {
            if (candidate<1e-5&&std::abs(error)<.05) return false;
            double rotation=std::max(-p.at("max_angular"),std::min(p.at("max_angular"),
                2*std::max(candidate,.03)*std::sin(error)/std::max(.1,std::hypot(target.x-pose.x,target.y-pose.y))));
            Pose prediction=pose;
            double horizon=std::max(p.at("prediction_time"),candidate/p.at("deceleration")+.1);
            for (double t=0;t<=horizon;t+=.05) {
                if (!poseClear(prediction,scan,collision)) return false;
                if (!movingClear(prediction,t,now)) { moving_conflict=true; return false; }
                prediction.x+=candidate*std::cos(prediction.yaw)*.05;
                prediction.y+=candidate*std::sin(prediction.yaw)*.05;
                prediction.yaw=wrap(prediction.yaw+rotation*.05);
            }
            if (candidate>1e-5) for (double s=std::max(0.0,projection.along);s<=detour->length+.02;s+=.04) {
                auto future=detour->poseAt(std::min(s,detour->length));
                if (!movingClear(future,(s-projection.along)/candidate,now)) { moving_conflict=true; return false; }
            }
            angular=rotation;
            return true;
        };
        bool safe=false;
        for (int attempt=0;attempt<5;++attempt) {
            if (feasible(speed)) { safe=true; break; }
            if (moving_conflict) { avoidance_clear=NAN; waiting=true; return hold("ONCOMING_YIELD"); }
            speed=attempt<3?speed*.5:0;
        }
        if (!safe) return blockedDetour(now,pose,scan,collision,target_end);
        last={"QUEUE_AVOID",true,false,speed,angular}; return last;
    }
    Decision waitForDetour(double now,Pose pose,const std::vector<Point>& scan,
                           const std::function<bool(Pose,const std::vector<Point>&)>& collision,
                           double target_end,double gap,const std::string& reason) {
        const Track* obstacle=nullptr;
        const auto window=obstacleWindow(pose);
        double c=std::cos(pose.yaw),s=std::sin(pose.yaw);
        auto relative=[&](Point point) {
            double dx=point.x-pose.x,dy=point.y-pose.y;
            return Point{c*dx+s*dy,-s*dx+c*dy};
        };
        if (p["avoidance_max_offset"]>0) {
            for (const auto& t:tracks) if (t.id==blocked_track&&std::isfinite(blocked_since)) {
                auto b=bounds(t); auto position=relative(t.center);
                if (observed_scan-t.last_seen<=.05&&now-t.last_seen<=p["scan_timeout"]&&
                    b.last>=window.start&&b.first<=window.front+p["resume_distance"]&&
                    b.right>=-p["corridor_half_width"]&&b.left<=p["corridor_half_width"]&&
                    std::hypot(position.x-blocked_position.x,position.y-blocked_position.y)<=.05)
                    obstacle=&t;
                break;
            }
            if (!obstacle) obstacle=blocker(pose,gap);
        }
        if (!obstacle&&p["avoidance_max_offset"]>0) {
            double nearest=INFINITY;
            for (const auto& t:tracks) {
                auto b=bounds(t);
                if (observed_scan-t.last_seen>.05||b.last<window.start||
                    b.first>window.front+p["resume_distance"]||
                    b.right<-p["corridor_half_width"]||b.left>p["corridor_half_width"]) continue;
                if (b.first<nearest) { nearest=b.first; obstacle=&t; }
            }
        }
        if (obstacle&&!waypoint_reached) {
            auto position=relative(obstacle->center);
            if (blocked_track!=obstacle->id||!std::isfinite(blocked_since)||now-blocked_tick>.5||
                std::hypot(position.x-blocked_position.x,position.y-blocked_position.y)>.05) {
                resetBlocked(); blocked_since=now; blocked_track=obstacle->id;
                blocked_position=position;
            }
            blocked_tick=now;
            if (now-blocked_since>=p["avoidance_wait"]&&
                (!std::isfinite(next_avoidance_try)||now>=next_avoidance_try)) {
                next_avoidance_try=now+.5;
                if (makeDetour(pose,*obstacle,target_end,now,scan,collision))
                    return followDetour(now,pose,scan,collision,target_end);
            }
        } else resetBlocked();
        return hold(oncoming_seen?"ONCOMING_YIELD":reason);
    }
    Decision hold(const std::string& reason) { last={reason,true,true}; return last; }
    Decision step(double now,const Pose* pose,double pose_stamp,const std::vector<Point>* scan,double scan_stamp,
                  double map_stamp,const std::function<bool(Pose,const std::vector<Point>&)>& collision) {
        if (zone<0) { last={"NORMAL"}; return last; }
        if (std::isfinite(last_time)&&now<last_time) {
            pause(); observed_scan=NAN; tracks.clear();
            for (auto& memory:queue_memory) memory.second.occupied_stamp=now;
        }
        last_time=now;
        if (fault) return hold("WAYPOINT_CHANGED_WHILE_WAITING");
        auto fresh=[&](double stamp,double timeout) { return std::isfinite(stamp)&&now-stamp>=-.05&&now-stamp<=timeout; };
        if (!pose||!finite(*pose)||!fresh(pose_stamp,p["pose_timeout"])) { pause(); return hold("POSE_UNAVAILABLE"); }
        if (!scan||!fresh(scan_stamp,p["scan_timeout"])) { pause(); return hold("SCAN_UNAVAILABLE"); }
        if (!collision||!fresh(map_stamp,p["map_timeout"])) { pause(); return hold("MAP_UNAVAILABLE"); }
        if (!collision(*pose,footprint)) { pause(); return hold("CURRENT_POSE_BLOCKED"); }
        const auto& z=zones[zone]; auto q=z.path.project({pose->x,pose->y});
        double lateral_limit=detour?std::max(p["max_lateral_deviation"],p["avoidance_max_offset"]+.02):p["max_lateral_deviation"];
        if (q.lateral>lateral_limit) { pause(); return hold("OUTSIDE_CORRIDOR"); }
        if (p["avoidance_max_offset"]>0) {
            observe(*scan,scan_stamp,*pose);
            releaseClearedLimits(scan_stamp,map_stamp);
        }
        if (detour&&q.along>=detour_end-.05&&std::hypot(pose->x-detour->segments.back().b.x,
            pose->y-detour->segments.back().b.y)<.08) {
            detour.reset(); resetBlocked(); waiting=false; clear_since=NAN;
        }
        double gap=detour?INFINITY:clearance(*pose,*scan);
        if (!detour) {
            if (gap<=p["detection_distance"]) { waiting=true; clear_since=NAN; }
            else if (gap>=p["resume_distance"]) {
                if (!std::isfinite(clear_since)) clear_since=now;
                if (now-clear_since>=p["clear_duration"]) waiting=false;
            } else clear_since=NAN;
        }
        double remaining=boundary(*pose);
        if (remaining<=p["stop_margin"]+.03||
            (!z.has_stop_line&&q.along>=z.path.length)||
            (identity==z.stop_id&&std::hypot(pose->x-z.stop_goal.x,pose->y-z.stop_goal.y)<=p["stop_goal_radius"]))
            line_reached=true;
        if (!released&&line_reached) { detour.reset(); resetBlocked(); return hold("STOP_LINE"); }
        double target_end=z.path.length,stop_remaining=z.has_stop_line?INFINITY:z.path.length-q.along;
        auto stop=z.stop_waypoints.find(identity);
        if (identity!=z.stop_id&&stop!=z.stop_waypoints.end()) {
            target_end=z.path.project(stop->second).along;
            stop_remaining=target_end-q.along;
            if (stop_remaining<=0||std::hypot(pose->x-stop->second.x,pose->y-stop->second.y)<=p["stop_goal_radius"])
                waypoint_reached=true;
        }
        if (released) {
            detour.reset(); resetBlocked();
            if (waiting) return hold("QUEUE_WAIT");
            last={"RELEASED_WAITING_FOR_NEXT_WAYPOINT",true,false}; return last;
        }
        if (waypoint_reached&&!waiting) { detour.reset(); resetBlocked(); last={"STOP_WAYPOINT",true,false}; return last; }
        if (detour) return followDetour(now,*pose,*scan,collision,target_end);
        if (waiting) return waitForDetour(now,*pose,*scan,collision,target_end,gap,"QUEUE_WAIT");
        auto target=z.path.pointAt(std::min(q.along+p["lookahead"],target_end));
        double error=wrap(std::atan2(target.y-pose->y,target.x-pose->x)-pose->yaw);
        double angular=std::max(-p["max_angular"],std::min(p["max_angular"],1.5*error));
        double available=std::max(0.0,std::min(remaining-p["stop_margin"],gap-p["detection_distance"]));
        available=std::max(0.0,std::min(available,stop_remaining));
        double queue_limit=INFINITY;
        for (const auto& limit:queue_limits) queue_limit=std::min(queue_limit,limit.second);
        available=std::max(0.0,std::min(available,queue_limit-obstacleWindow(*pose).front));
        double speed=std::min(p["max_speed"],std::sqrt(2*p["deceleration"]*available));
        speed=std::min(speed,available/approachTime());
        speed*=std::max(0.0,std::cos(error)); if (std::abs(error)>.6) speed=0;
        auto feasible=[&](double candidate) {
            if (candidate<1e-5&&std::abs(error)<.05) return false;
            Pose prediction=*pose;
            double horizon=std::max(p["prediction_time"],candidate/p["deceleration"]+1/control_rate);
            double dt=std::min(.05,.5*p["max_lateral_deviation"]/std::max(candidate,.01));
            for (int i=0;i<=std::ceil(horizon/dt);++i) {
                auto projected=z.path.project({prediction.x,prediction.y});
                if (projected.lateral>p["max_lateral_deviation"]||
                    (!z.has_stop_line&&projected.along>z.path.length)||
                    boundary(prediction)<p["stop_margin"]||!collision(prediction,footprint)) return false;
                for (auto v:footprint) {
                    auto vertex=z.path.project(transform(v,prediction));
                    if (vertex.lateral>p["corridor_half_width"]||vertex.along>queue_limit) return false;
                }
                prediction.x+=candidate*std::cos(prediction.yaw)*dt;
                prediction.y+=candidate*std::sin(prediction.yaw)*dt;
                prediction.yaw=wrap(prediction.yaw+angular*dt);
            }
            return true;
        };
        bool safe=false;
        for (int attempt=0;attempt<5;++attempt) {
            if (feasible(speed)) { safe=true; break; }
            speed=attempt<3?speed*.5:0;
        }
        if (!safe) return waitForDetour(now,*pose,*scan,collision,target_end,gap,"TRAJECTORY_BLOCKED");
        resetBlocked(); last={"QUEUE_ADVANCE",true,false,speed,angular}; return last;
    }
    bool release(double now,std::string& message) {
        if (!std::isfinite(last_time)||now-last_time>2/control_rate||last.state!="STOP_LINE") {
            message="Release requires fresh data and a stop at the end of the queue area"; return false;
        }
        if (!std::isfinite(clear_since)||now-clear_since<p["clear_duration"]||waiting) {
            message="Front space must be continuously clear before release"; return false;
        }
        released=true; message="Released; use Next Waypoint to continue"; return true;
    }
};
}

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <actionlib_msgs/GoalID.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>
#include <waypoint_manager_msgs/Route.h>
#include <waypoint_manager_msgs/Waypoint.h>
#include <waypoint_manager_msgs/Waypoints.h>

class Node {
    ros::NodeHandle nh,private_nh{"~"};
    tf::TransformListener tf;
    std::unique_ptr<tf::MessageFilter<sensor_msgs::LaserScan>> scan_filter;
    ros::Publisher goal_pub,cmd_pub,cancel_pub,active_pub,hold_pub,state_pub,detection_pub;
    std::vector<ros::Subscriber> subscribers;
    ros::Subscriber accepted_goal_sub;
    ros::Timer timer; ros::ServiceServer release_service;
    bool queue_enabled=false,running=false,was_active=false,config_ready=false;
    bool route_ready=false,waypoints_ready=false,waypoint_ready=false,scan_valid=false,fault=false;
    bool normal_goal_sent=false,goal_pending=true,front_stop=false,waypoint_frame_valid=false,running_known=false;
    bool navigation_ready=false,published_active=true,published_hold=true;
    std::string frame,base,input_topic,output_topic,previous_state,queue_error;
    YAML::Node config;
    std::unique_ptr<queue_wait::Controller> controller;
    std::vector<std::string> route;
    std::map<std::string,queue_wait::Waypoint> waypoints;
    waypoint_manager_msgs::Waypoint current_waypoint;
    std::vector<queue_wait::Point> scan;
    std::shared_ptr<queue_wait::ScanView> scan_view;
    sensor_msgs::LaserScan::ConstPtr latest_scan;
    std::set<uint64_t> goal_requests;
    double scan_stamp=NAN;
    ros::WallTime next_initialization;
    struct Map { std::shared_ptr<queue_wait::Grid> grid; std::string frame; double stamp; };
    std::map<std::string,Map> maps;
    queue_wait::Pose transform(const std::string& target,const std::string& source,ros::Time stamp) {
        if (target==source) return {0,0,0};
        tf.waitForTransform(target,source,stamp,ros::Duration(.02));
        tf::StampedTransform t; tf.lookupTransform(target,source,stamp,t);
        queue_wait::Pose result{t.getOrigin().x(),t.getOrigin().y(),tf::getYaw(t.getRotation())};
        if (!queue_wait::finite(result)) throw std::runtime_error("Non-finite transform");
        return result;
    }
    static bool sameFrame(std::string a,std::string b) {
        while (!a.empty()&&a.front()=='/') a.erase(a.begin());
        while (!b.empty()&&b.front()=='/') b.erase(b.begin());
        return a==b;
    }
    static bool samePose(const geometry_msgs::Pose& a,const geometry_msgs::Pose& b) {
        auto square=[](double v) { return v*v; };
        double position=square(a.position.x-b.position.x)+square(a.position.y-b.position.y)+square(a.position.z-b.position.z);
        double direct=square(a.orientation.x-b.orientation.x)+square(a.orientation.y-b.orientation.y)+
            square(a.orientation.z-b.orientation.z)+square(a.orientation.w-b.orientation.w);
        double opposite=square(a.orientation.x+b.orientation.x)+square(a.orientation.y+b.orientation.y)+
            square(a.orientation.z+b.orientation.z)+square(a.orientation.w+b.orientation.w);
        return position<=1e-12&&std::min(direct,opposite)<=1e-12;
    }
    void stop() { cmd_pub.publish(geometry_msgs::Twist{}); }
    void cancel() {
        normal_goal_sent=false; goal_pending=true; goal_requests.clear();
        cancel_pub.publish(actionlib_msgs::GoalID{});
    }
    bool queueTarget() const {
        if (!config_ready||!route_ready||!waypoints_ready||!waypoint_ready) return true;
        auto target=std::find(route.begin(),route.end(),current_waypoint.identity);
        if (target==route.end()) return true;
        if (!config["areas"]||!config["areas"].IsSequence()) return true;
        try {
            for (const auto& area:config["areas"]) {
                if (!area["queue_wait"]||!area["queue_wait"].as<bool>()) continue;
                auto begin=std::find(route.begin(),route.end(),area["start_id"]?area["start_id"].as<std::string>():std::string{});
                auto end=std::find(route.begin(),route.end(),area["end_id"]?area["end_id"].as<std::string>():std::string{});
                if (begin==route.end()&&end==route.end()) return true;
                if (begin==route.end()) begin=route.begin();
                if (end==route.end()) end=route.end()-1;
                if (begin>end) std::swap(begin,end);
                if (std::find(begin,end+1,current_waypoint.identity)!=end+1) return true;
            }
        } catch (const std::exception&) { return true; }
        return false;
    }
    bool entryHold() {
        if (!controller||!waypoint_ready) return false;
        auto current=std::find(route.begin(),route.end(),current_waypoint.identity);
        if (current==route.end()||current+1==route.end()) return false;
        for (const auto& area:controller->zones) {
            if (area.ids.count(*current)||!area.ids.count(*(current+1))) continue;
            try {
                tf::StampedTransform pose;
                tf.lookupTransform(frame,base,ros::Time(0),pose);
                double age=(ros::Time::now()-pose.stamp_).toSec();
                if (age<-.05||age>controller->p.at("pose_timeout")) return true;
                return !controller->entryReady(area,{pose.getOrigin().x(),pose.getOrigin().y(),tf::getYaw(pose.getRotation())});
            } catch (const std::exception&) { return true; }
        }
        return false;
    }
    void publishGoal(bool target_changed=true) {
        if (queue_enabled&&(!navigation_ready||!goal_pub.getNumSubscribers()||!accepted_goal_sub.getNumPublishers())) {
            normal_goal_sent=false; goal_pending=true; goal_requests.clear(); return;
        }
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id=frame; goal.header.stamp=ros::Time::now(); goal.pose=current_waypoint.pose;
        if (target_changed||!normal_goal_sent) { goal_pending=true; goal_requests.clear(); }
        if (goal_pending) {
            goal_requests.insert(goal.header.stamp.toNSec());
            if (goal_requests.size()>64) goal_requests.erase(goal_requests.begin());
        }
        normal_goal_sent=true; goal_pub.publish(goal);
    }
    void onAcceptedGoal(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (!running||fault||!navigation_ready||queueTarget()) { cancel(); return; }
        if (!goal_pending) return;
        if (sameFrame(msg->header.frame_id,frame)&&samePose(msg->pose,current_waypoint.pose)&&
            goal_requests.count(msg->header.stamp.toNSec())) {
            goal_pending=false; goal_requests.clear();
        }
    }
    void fail(const std::string& reason) {
        if (!fault) { cancel(); if (running) stop(); }
        fault=true; if (controller) controller->fault=true;
        ROS_ERROR("queue_wait: %s",reason.c_str());
    }
    void initialize() {
        if (controller||fault||!config_ready||!route_ready||!waypoints_ready) return;
        try {
            std::string wired_topic; ros::param::get("/move_base/queue_wait_cmd_vel_topic",wired_topic);
            navigation_ready=wired_topic==input_topic;
            if (!navigation_ready) throw std::invalid_argument("Enable use_queue_wait in navigation launch too");
            if (!waypoint_frame_valid) throw std::invalid_argument("Waypoint frame differs from global frame");
            std::vector<queue_wait::Point> footprint; XmlRpc::XmlRpcValue f;
            if (!ros::param::get("/move_base/local_costmap/footprint",f)) throw std::invalid_argument("Missing local_costmap/footprint");
            if (f.getType()==XmlRpc::XmlRpcValue::TypeArray) {
                auto number=[](XmlRpc::XmlRpcValue v)->double {
                    if (v.getType()==XmlRpc::XmlRpcValue::TypeInt) return static_cast<int>(v);
                    if (v.getType()==XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(v);
                    throw std::invalid_argument("Footprint values must be numbers");
                };
                for (int i=0;i<f.size();++i) {
                    if (f[i].getType()!=XmlRpc::XmlRpcValue::TypeArray||f[i].size()!=2) throw std::invalid_argument("Footprint needs [x,y] vertices");
                    footprint.push_back({number(f[i][0]),number(f[i][1])});
                }
            } else if (f.getType()==XmlRpc::XmlRpcValue::TypeString) {
                auto vertices=YAML::Load(static_cast<std::string>(f));
                for (const auto& v:vertices) {
                    if (!v.IsSequence()||v.size()!=2) throw std::invalid_argument("Footprint needs [x,y] vertices");
                    footprint.push_back({v[0].as<double>(),v[1].as<double>()});
                }
            } else throw std::invalid_argument("Invalid footprint type");
            controller.reset(new queue_wait::Controller(queue_wait::loadZones(config,route,waypoints),config["queue_wait"],footprint));
            if (waypoint_ready) controller->waypoint(current_waypoint.identity);
            queue_error.clear();
            ROS_INFO("queue_wait: %zu configured crossing areas",controller->zones.size());
        } catch (const std::exception& e) {
            if (queue_error!=e.what()) ROS_ERROR("queue_wait: %s",e.what());
            queue_error=e.what();
        }
        if (running&&waypoint_ready&&!queueTarget()&&!normal_goal_sent) publishGoal();
    }
    static bool sameConfig(const YAML::Node& a,const YAML::Node& b) {
        if (a.Type()!=b.Type()||a.size()!=b.size()) return false;
        if (a.IsScalar()) return a.as<std::string>()==b.as<std::string>();
        if (a.IsSequence()) {
            for (size_t i=0;i<a.size();++i) if (!sameConfig(a[i],b[i])) return false;
        } else if (a.IsMap()) {
            for (const auto& value:a) {
                auto key=value.first.as<std::string>();
                if (!b[key]||!sameConfig(value.second,b[key])) return false;
            }
        }
        return true;
    }
    static YAML::Node number(const YAML::Node& source) {
        try { const double value=source.as<double>(); return YAML::Node(value==0?0.0:value); }
        catch (const YAML::Exception&) { return source; }
    }
    static YAML::Node queueConfig(const YAML::Node& source) {
        if (!source.IsMap()||!source["areas"]||!source["areas"].IsSequence()) return source;
        YAML::Node result;
        result["queue_wait"]=source["queue_wait"]?source["queue_wait"]:YAML::Node(YAML::NodeType::Map);
        if (result["queue_wait"].IsMap()) {
            YAML::Node parameters(YAML::NodeType::Map);
            for (const auto& value:result["queue_wait"]) parameters[value.first.as<std::string>()]=number(value.second);
            result["queue_wait"]=parameters;
        }
        result["areas"]=YAML::Node(YAML::NodeType::Sequence);
        for (const auto& area:source["areas"]) {
            if (!area.IsMap()||!area["queue_wait"]||!area["queue_wait"].as<bool>()) continue;
            YAML::Node queue;
            for (const auto& key:{"name","start_id","end_id","stop_line"}) if (area[key]) queue[key]=area[key];
            if (queue["stop_line"].IsMap()) {
                YAML::Node line;
                if (area["stop_line"]["yaw"]) line["yaw"]=number(area["stop_line"]["yaw"]);
                auto points=area["stop_line"]["point"];
                if (points.IsSequence()) {
                    line["point"]=YAML::Node(YAML::NodeType::Sequence);
                    for (const auto& point:points) line["point"].push_back(number(point));
                } else if (points) line["point"]=points;
                queue["stop_line"]=line;
            }
            queue["queue_wait"]=true;
            result["areas"].push_back(queue);
        }
        return result;
    }
    void onConfig(const std_msgs::String::ConstPtr& msg) {
        try {
            auto updated=queueConfig(YAML::Load(msg->data));
            if (config_ready&&sameConfig(config,updated)) return;
            if (controller) { fail("Configuration changed; restart with the new route/config"); return; }
            config=updated; config_ready=true; initialize();
        } catch (const std::exception& e) {
            if (controller) fail(e.what());
            else { config_ready=false; queue_error=e.what(); ROS_ERROR("queue_wait: %s",e.what()); }
        }
    }
    void onRoute(const waypoint_manager_msgs::Route::ConstPtr& msg) {
        if (controller&&route!=msg->identities) fail("Route changed; restart with the new route/config");
        route=msg->identities; route_ready=true; initialize();
    }
    void onWaypoints(const waypoint_manager_msgs::Waypoints::ConstPtr& msg) {
        waypoint_frame_valid=sameFrame(msg->info.header.frame_id,frame);
        if (controller&&!waypoint_frame_valid) { fail("Waypoint frame differs from global frame"); return; }
        std::map<std::string,queue_wait::Waypoint> updated;
        for (const auto& w:msg->waypoints) {
            bool is_stop=false;
            for (const auto& p:w.properties) if (p.name=="stop") is_stop=(p.data=="true");
            updated[w.identity]={{w.pose.position.x,w.pose.position.y},is_stop};
        }
        bool changed=waypoints.size()!=updated.size();
        for (const auto& w:updated) {
            auto old=waypoints.find(w.first);
            if (old==waypoints.end()||old->second.position.x!=w.second.position.x||
                old->second.position.y!=w.second.position.y||old->second.stop!=w.second.stop) changed=true;
        }
        if (controller&&changed) fail("Waypoints changed; restart with the new route/config");
        waypoints=std::move(updated); waypoints_ready=true; initialize();
    }
    void onWaypoint(const waypoint_manager_msgs::Waypoint::ConstPtr& msg) {
        const bool goal_changed=!waypoint_ready||msg->identity!=current_waypoint.identity||!samePose(msg->pose,current_waypoint.pose);
        current_waypoint=*msg; waypoint_ready=true;
        if (!queue_enabled) { publishGoal(); return; }
        if (fault) return;
        if (controller) {
            controller->waypoint(msg->identity);
            if (controller->fault) { fail("Waypoint changed before operator release"); return; }
        }
        if (queueTarget()&&!was_active) {
            cancel(); if (running) stop(); was_active=true;
            scan_valid=false; scan_stamp=NAN;
            if (controller&&controller->zone>=0&&latest_scan) scan_filter->add(latest_scan);
            std_msgs::Bool held; held.data=true; active_pub.publish(held); hold_pub.publish(held);
            published_active=published_hold=true;
        }
        if (!queueTarget()&&entryHold()&&!published_hold) {
            std_msgs::Bool held; held.data=true; hold_pub.publish(held); published_hold=true;
        }
        if (running&&!queueTarget()) publishGoal(goal_changed);
    }
    void onRunning(const std_msgs::Bool::ConstPtr& msg) {
        if (running_known&&running==msg->data) return;
        const bool was_running=running; running_known=true; running=msg->data;
        if (!running) { cancel(); if (was_running) stop(); if (controller) controller->pause(); }
        else if (waypoint_ready&&!fault&&!queueTarget()) publishGoal();
    }
    void onCommand(const geometry_msgs::Twist::ConstPtr& msg) {
        if (!running||fault||!navigation_ready||queueTarget()) return;
        const auto& l=msg->linear; const auto& a=msg->angular;
        if (!std::isfinite(l.x)||!std::isfinite(l.y)||!std::isfinite(l.z)||!std::isfinite(a.x)||!std::isfinite(a.y)||!std::isfinite(a.z)) {
            stop(); return;
        }
        if (goal_pending&&*msg!=geometry_msgs::Twist{}) return;
        cmd_pub.publish(*msg);
    }
    void onScan(const sensor_msgs::LaserScan::ConstPtr& msg) {
        if (!controller||controller->zone<0) return;
        const double stamp=msg->header.stamp.toSec(),now=ros::Time::now().toSec();
        const double timeout=controller?controller->p.at("scan_timeout"):.5;
        if (msg->header.stamp.isZero()||now-stamp<-.05||now-stamp>timeout) return;
        if (std::isfinite(scan_stamp)&&scan_stamp<=now+.05&&stamp<scan_stamp) return;
        try {
            if (!std::isfinite(msg->angle_min)||!std::isfinite(msg->angle_increment)||msg->angle_increment<=0)
                throw std::runtime_error("Invalid scan angles");
            if (!std::isfinite(msg->range_min)||!std::isfinite(msg->range_max)||
                msg->range_min<0||msg->range_max<=msg->range_min)
                throw std::runtime_error("Invalid scan range bounds");
            auto t=transform(frame,msg->header.frame_id,msg->header.stamp);
            auto view=std::make_shared<queue_wait::ScanView>();
            view->origin=t; view->angle_min=msg->angle_min; view->angle_increment=msg->angle_increment;
            view->range_min=msg->range_min; view->range_max=msg->range_max; view->ranges=msg->ranges;
            std::vector<queue_wait::Point> updated;
            bool valid_measurement=false;
            for (size_t i=0;i<msg->ranges.size();++i) {
                double r=msg->ranges[i],a=msg->angle_min+i*msg->angle_increment;
                if (std::isfinite(r)&&r>=msg->range_min&&r<=msg->range_max) {
                    updated.push_back(queue_wait::transform({r*std::cos(a),r*std::sin(a)},t));
                    valid_measurement=true;
                } else if (std::isinf(r)&&r>0) valid_measurement=true;
            }
            scan=std::move(updated); scan_stamp=stamp;
            scan_view=std::move(view);
            scan_valid=valid_measurement;
        } catch (const tf::TransformException& e) {
            ROS_WARN_THROTTLE(2,"queue_wait scan: %s",e.what());
        } catch (const std::exception& e) {
            scan_valid=false; scan.clear(); scan_stamp=stamp;
            ROS_WARN_THROTTLE(2,"queue_wait scan: %s",e.what());
        }
    }
    void onMap(const std::string& key,const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        try {
            if (key=="constraint") {
                const double now=ros::Time::now().toSec();
                if (msg->header.stamp.toSec()>now+.05) return;
                auto current=maps.find(key);
                if (current!=maps.end()&&current->second.stamp<=now+.05&&
                    msg->header.stamp.toSec()<current->second.stamp) return;
            }
            const auto& o=msg->info.origin;
            auto grid=std::make_shared<queue_wait::Grid>(msg->info.width,msg->info.height,msg->info.resolution,
                queue_wait::Pose{o.position.x,o.position.y,tf::getYaw(o.orientation)},msg->data,key=="local");
            maps[key]={grid,msg->header.frame_id,msg->header.stamp.toSec()};
        } catch (const std::exception& e) { maps.erase(key); ROS_ERROR("queue_wait grid: %s",e.what()); }
    }
    void onMapUpdate(const map_msgs::OccupancyGridUpdate::ConstPtr& msg) {
        auto found=maps.find("local");
        if (found==maps.end()) return;
        auto& map=found->second; auto& grid=*map.grid;
        const double stamp=msg->header.stamp.toSec(),now=ros::Time::now().toSec();
        if (msg->header.stamp.isZero()||stamp>now+.05||(map.stamp<=now+.05&&stamp<map.stamp)) return;
        if (msg->header.frame_id!=map.frame||msg->x<0||msg->y<0||
            static_cast<uint64_t>(msg->x)+msg->width>static_cast<uint64_t>(grid.width)||
            static_cast<uint64_t>(msg->y)+msg->height>static_cast<uint64_t>(grid.height)||
            msg->data.size()!=static_cast<size_t>(msg->width)*msg->height) {
            maps.erase(found); return;
        }
        for (size_t y=0;y<msg->height;++y)
            std::copy_n(msg->data.begin()+y*msg->width,msg->width,
                        grid.data.begin()+(msg->y+y)*grid.width+msg->x);
        map.stamp=msg->header.stamp.toSec();
    }
    queue_wait::Decision decision(double now) {
        if (controller->zone<0) return controller->step(now,nullptr,NAN,nullptr,NAN,NAN,{});
        controller->static_obstacle={};
        controller->observed_free={};
        queue_wait::Pose pose{}; bool pose_valid=false; double pose_stamp=NAN,map_stamp=NAN;
        try {
            tf::StampedTransform t; tf.lookupTransform(frame,base,ros::Time(0),t);
            pose={t.getOrigin().x(),t.getOrigin().y(),tf::getYaw(t.getRotation())}; pose_stamp=t.stamp_.toSec(); pose_valid=true;
        } catch (const std::exception&) {}
        std::function<bool(queue_wait::Pose,const std::vector<queue_wait::Point>&)> collision;
        if (maps.count("local")&&maps.count("constraint")) try {
            std::vector<std::pair<std::shared_ptr<queue_wait::Grid>,queue_wait::Pose>> grids;
            for (const auto& m:maps) {
                auto offset=transform(m.second.frame,frame,ros::Time(0));
                grids.push_back({m.second.grid,offset});
                if (m.first=="constraint") {
                    auto grid=m.second.grid;
                    controller->static_obstacle=[grid,offset](queue_wait::Point point) {
                        return grid->occupiedNear(queue_wait::transform(point,offset),.05);
                    };
                }
            }
            map_stamp=maps.at("local").stamp;
            if (scan_valid&&scan_view) {
                auto view=scan_view;
                controller->observed_free=[view,grids](queue_wait::Point point) {
                    if (!view->clears(point)) return false;
                    for (const auto& grid:grids)
                        if (!grid.first->observedFree(queue_wait::transform(point,grid.second))) return false;
                    return true;
                };
            }
            collision=[grids](queue_wait::Pose predicted,const std::vector<queue_wait::Point>& f) {
                for (const auto& g:grids) {
                    auto p=queue_wait::transform({predicted.x,predicted.y},g.second);
                    if (!g.first->free({p.x,p.y,predicted.yaw+g.second.yaw},f)) return false;
                }
                return true;
            };
        } catch (const std::exception&) {}
        return controller->step(now,pose_valid?&pose:nullptr,pose_stamp,scan_valid?&scan:nullptr,scan_stamp,map_stamp,collision);
    }
    void publishDetectionRange(ros::Time now) {
        if (!detection_pub.getNumSubscribers()) return;
        visualization_msgs::Marker marker;
        marker.header.frame_id=frame; marker.header.stamp=now;
        marker.ns="queue_wait_detection"; marker.id=0;
        marker.type=visualization_msgs::Marker::LINE_LIST;
        marker.action=visualization_msgs::Marker::DELETE;
        marker.pose.orientation.w=1;
        marker.scale.x=.03; marker.color.r=1; marker.color.a=1;
        marker.lifetime=ros::Duration(.3);
        if (controller&&!fault&&controller->zone>=0) try {
            tf::StampedTransform t; tf.lookupTransform(frame,base,ros::Time(0),t);
            queue_wait::Pose pose{t.getOrigin().x(),t.getOrigin().y(),tf::getYaw(t.getRotation())};
            double age=(now-t.stamp_).toSec();
            if (queue_wait::finite(pose)&&age>=-.05&&age<=controller->p.at("pose_timeout")) {
                const auto& path=controller->drivingPath();
                const auto window=controller->obstacleWindow(pose);
                const double start=window.start,end=window.stop_end,width=window.half_width;
                auto offset=[](queue_wait::Point p,queue_wait::Point n,double d) {
                    return queue_wait::Point{p.x+n.x*d,p.y+n.y*d};
                };
                auto append=[&](queue_wait::Point p) {
                    geometry_msgs::Point point; point.x=p.x; point.y=p.y; point.z=.08;
                    marker.points.push_back(point);
                };
                auto edge=[&](queue_wait::Point a,queue_wait::Point b,bool cap=false,double along=NAN) {
                    int steps=std::max(1,static_cast<int>(std::ceil(std::hypot(b.x-a.x,b.y-a.y)/.05)));
                    auto point=[&](double k) {
                        return queue_wait::Point{a.x+(b.x-a.x)*k/steps,a.y+(b.y-a.y)*k/steps};
                    };
                    bool drawing=false; queue_wait::Point first{};
                    for (int k=0;k<=steps;++k) {
                        bool visible=false;
                        if (k<steps) {
                            auto q=path.project(point(k+.5));
                            visible=q.along>=start-1e-6&&q.along<=end+1e-6&&
                                (!std::isfinite(along)||std::abs(q.along-along)<1e-6)&&
                                (cap ? q.lateral<=width+1e-6
                                     : std::abs(q.lateral-width)<.002);
                        }
                        if (visible&&!drawing) { first=point(k); drawing=true; }
                        if (drawing&&!visible) { append(first); append(point(k)); drawing=false; }
                    }
                };
                for (size_t i=0;i<path.segments.size();++i) {
                    const auto& s=path.segments[i];
                    double low=i==0?start:std::max(start,s.offset);
                    double high=i+1==path.segments.size()?end:std::min(end,s.offset+s.length);
                    if (high<=low) continue;
                    queue_wait::Point direction{(s.b.x-s.a.x)/s.length,(s.b.y-s.a.y)/s.length};
                    queue_wait::Point normal{-direction.y,direction.x};
                    auto a=offset(s.a,direction,low-s.offset),b=offset(s.a,direction,high-s.offset);
                    edge(offset(a,normal,width),offset(b,normal,width));
                    edge(offset(a,normal,-width),offset(b,normal,-width));
                    if (low==start) edge(offset(a,normal,width),offset(a,normal,-width),true,start);
                    if (high==end) edge(offset(b,normal,width),offset(b,normal,-width),true,end);
                }
                for (size_t i=1;i<path.segments.size();++i) {
                    const auto& s=path.segments[i];
                    if (s.offset<start||s.offset>end) continue;
                    for (int k=0;k<72;++k) {
                        double a=2*std::acos(-1.0)*k/72,b=2*std::acos(-1.0)*(k+1)/72;
                        edge(offset(s.a,{std::cos(a),std::sin(a)},width),
                             offset(s.a,{std::cos(b),std::sin(b)},width),false,s.offset);
                    }
                }
                if (!marker.points.empty()) marker.action=visualization_msgs::Marker::ADD;
            }
        } catch (const tf::TransformException&) {}
        detection_pub.publish(marker);
        visualization_msgs::Marker avoidance;
        avoidance.header=marker.header; avoidance.ns="queue_wait_avoidance"; avoidance.id=1;
        avoidance.type=visualization_msgs::Marker::LINE_STRIP;
        avoidance.action=visualization_msgs::Marker::DELETE;
        avoidance.pose.orientation.w=1; avoidance.scale.x=.035;
        avoidance.color.r=1; avoidance.color.g=.6; avoidance.color.a=1;
        if (controller&&controller->zone>=0&&controller->detour) {
            for (const auto& segment:controller->detour->segments) {
                geometry_msgs::Point point; point.x=segment.a.x; point.y=segment.a.y; point.z=.1;
                avoidance.points.push_back(point);
            }
            auto end=controller->detour->segments.back().b;
            geometry_msgs::Point point; point.x=end.x; point.y=end.y; point.z=.1;
            avoidance.points.push_back(point);
            avoidance.action=visualization_msgs::Marker::ADD;
        }
        detection_pub.publish(avoidance);
    }
    void tick(const ros::TimerEvent&) {
        if (!controller&&!fault&&ros::WallTime::now()>=next_initialization) {
            next_initialization=ros::WallTime::now()+ros::WallDuration(1); initialize();
        }
        queue_wait::Decision result{"CONFIG_UNAVAILABLE",true,true};
        const double now=ros::Time::now().toSec();
        if (!fault) {
            if (!queueTarget()) {
                result={"NORMAL"};
                if (entryHold()) { result.state="QUEUE_ENTRY_ALIGNMENT"; result.hold=true; }
                if (running&&!normal_goal_sent) publishGoal();
            }
            else if (controller) result=decision(now);
            if (!navigation_ready) result={"NAVIGATION_UNAVAILABLE",true,true};
            if (!running) {
                if (controller) controller->pause();
                result.hold=result.active; result.state="OPERATOR_PAUSED";
            } else if (result.active) {
                if (!was_active) cancel();
                if (front_stop) {
                    if (controller) { controller->pause(); result=controller->hold("FRONT_STOP"); }
                    else result={"FRONT_STOP",true,true};
                }
                geometry_msgs::Twist output;
                output.linear.x=result.linear; output.angular.z=result.angular;
                cmd_pub.publish(output);
            }
        }
        was_active=result.active;
        std_msgs::Bool active,held; active.data=result.active; held.data=result.hold;
        if (active.data!=published_active) { active_pub.publish(active); published_active=active.data; }
        if (held.data!=published_hold) { hold_pub.publish(held); published_hold=held.data; }
        if (previous_state!=result.state) {
            std_msgs::String state; state.data=result.state; state_pub.publish(state);
            ROS_INFO("queue_wait: %s",state.data.c_str()); previous_state=state.data;
        }
        publishDetectionRange(ros::Time::now());
    }
    bool release(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& response) {
        if (!controller||fault||!running||front_stop) { response.message="Resume Switch Cancel and valid queue configuration first"; return true; }
        double now=ros::Time::now().toSec(); decision(now);
        response.success=controller->release(now,response.message);
        if (response.success) { std_msgs::Bool held; held.data=false; hold_pub.publish(held); published_hold=false; }
        return true;
    }
public:
    Node() {
        bool latch=false; std::string goal_topic,waypoint_topic;
        private_nh.param("latch",latch,false);
        private_nh.param("frame_id",frame,std::string("map"));
        private_nh.param("robot_base_frame",base,std::string("base_link"));
        private_nh.param("pose_topic",goal_topic,std::string("move_base_simple/goal"));
        private_nh.param("waypoint_topic",waypoint_topic,std::string("waypoint"));
        private_nh.param("use_queue_wait",queue_enabled,false);
        goal_pub=nh.advertise<geometry_msgs::PoseStamped>(goal_topic,1,latch);
        subscribers.push_back(nh.subscribe(waypoint_topic,1,&Node::onWaypoint,this));
        if (!queue_enabled) return;
        private_nh.param("cmd_vel_in",input_topic,std::string("/queue_wait/cmd_vel_in"));
        private_nh.param("cmd_vel_out",output_topic,std::string("/icart_mini/cmd_vel"));
        if (nh.resolveName(input_topic)==nh.resolveName(output_topic)) throw std::invalid_argument("cmd_vel input/output must differ");
        cmd_pub=nh.advertise<geometry_msgs::Twist>(output_topic,1);
        cancel_pub=nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel",1);
        active_pub=nh.advertise<std_msgs::Bool>("/queue_wait/active",1,true);
        hold_pub=nh.advertise<std_msgs::Bool>("/queue_wait/hold",1,true);
        state_pub=nh.advertise<std_msgs::String>("/queue_wait/state",1,true);
        detection_pub=nh.advertise<visualization_msgs::Marker>("/queue_wait/detection_range",1);
        std_msgs::Bool held; held.data=true; active_pub.publish(held); hold_pub.publish(held);
        subscribers.push_back(nh.subscribe("/waypoint_manager/queue_wait/config",1,&Node::onConfig,this));
        subscribers.push_back(nh.subscribe("route",1,&Node::onRoute,this));
        subscribers.push_back(nh.subscribe("waypoints",1,&Node::onWaypoints,this));
        scan_filter.reset(new tf::MessageFilter<sensor_msgs::LaserScan>(tf,frame,20,nh));
        scan_filter->registerCallback([this](const sensor_msgs::LaserScan::ConstPtr& msg) { onScan(msg); });
        scan_filter->registerFailureCallback([this](const sensor_msgs::LaserScan::ConstPtr&,tf::FilterFailureReason reason) {
            ROS_WARN_THROTTLE(2,"queue_wait scan: dropped before TF was available (reason %d)",static_cast<int>(reason));
        });
        subscribers.push_back(nh.subscribe<sensor_msgs::LaserScan>("/scan",1,
            [this](const sensor_msgs::LaserScan::ConstPtr& msg) {
                latest_scan=msg; if (controller&&controller->zone>=0) scan_filter->add(msg);
            }));
        subscribers.push_back(nh.subscribe(input_topic,1,&Node::onCommand,this));
        accepted_goal_sub=nh.subscribe("/move_base/current_goal",1,&Node::onAcceptedGoal,this);
        subscribers.push_back(nh.subscribe<std_msgs::Bool>("/queue_wait/front_stop",1,
            [this](const std_msgs::Bool::ConstPtr& m) { front_stop=m->data; }));
        subscribers.push_back(nh.subscribe("/waypoint_manager/waypoint_server/running",1,&Node::onRunning,this));
        subscribers.push_back(nh.subscribe<nav_msgs::OccupancyGrid>("/move_base/local_costmap/costmap",1,
            [this](const nav_msgs::OccupancyGrid::ConstPtr& m) { onMap("local",m); }));
        subscribers.push_back(nh.subscribe("/move_base/local_costmap/costmap_updates",1,&Node::onMapUpdate,this));
        subscribers.push_back(nh.subscribe<nav_msgs::OccupancyGrid>("/map_for_costmap",1,
            [this](const nav_msgs::OccupancyGrid::ConstPtr& m) { onMap("constraint",m); }));
        release_service=nh.advertiseService("/queue_wait/release",&Node::release,this);
        timer=nh.createTimer(ros::Duration(1/queue_wait::Controller::control_rate),&Node::tick,this);
    }
    ~Node() { if (queue_enabled) stop(); }
    void spin() { ros::spin(); }
};
int main(int argc,char** argv) {
    ros::init(argc,argv,"waypoint_to_posestamped_node");
    try { Node node; node.spin(); }
    catch (const std::exception& e) { ROS_FATAL("%s",e.what()); return 1; }
    return 0;
}

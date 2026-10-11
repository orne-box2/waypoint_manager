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

// The existing waypoint converter also owns autonomous velocity in queue mode.
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
struct Projection { double lateral,along; };
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
            if (d<best.lateral) best={d,s.offset+t*s.length};
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
};
struct Waypoint { Point position; bool stop; };
struct Zone {
    std::string name,stop_id; std::set<std::string> ids; Path path;
    Point line,stop_goal; double line_yaw;
};
std::vector<Zone> loadZones(const YAML::Node& config,const std::vector<std::string>& route,
                           const std::map<std::string,Waypoint>& waypoints) {
    std::vector<Zone> zones; std::set<std::string> used;
    for (const auto& area:config["areas"]) {
        if (!area["queue_wait"]||!area["queue_wait"].as<bool>()) continue;
        auto start=area["start_id"].as<std::string>(),end=area["end_id"].as<std::string>();
        if (std::count(route.begin(),route.end(),start)!=1||std::count(route.begin(),route.end(),end)!=1)
            throw std::invalid_argument("Queue start_id/end_id must each occur once in the current route");
        auto begin=std::find(route.begin(),route.end(),start)-route.begin();
        auto finish=std::find(route.begin(),route.end(),end)-route.begin();
        if (begin>=finish) throw std::invalid_argument("Queue start_id must precede end_id");
        std::set<std::string> ids;
        for (auto i=begin;i<=finish;++i) {
            if (std::count(route.begin(),route.end(),route[i])!=1||!used.insert(route[i]).second)
                throw std::invalid_argument("Queue IDs must be unique; queue areas must not overlap");
            ids.insert(route[i]);
        }
        if (!waypoints.at(end).stop) throw std::invalid_argument("Queue end waypoint needs stop: true");
        std::vector<Point> points;
        for (auto i=std::max<decltype(begin)>(0,begin-1);i<=finish;++i)
            points.push_back(waypoints.at(route[i]).position);
        auto line=area["stop_line"]["point"];
        if (!line.IsSequence()||line.size()!=2) throw std::invalid_argument("stop_line.point needs [x,y]");
        Point p{line[0].as<double>(),line[1].as<double>()};
        double yaw=area["stop_line"]["yaw"].as<double>(); Point goal=waypoints.at(end).position;
        if (!finite(p)||!std::isfinite(yaw)||!finite(goal)) throw std::invalid_argument("Non-finite stop line/goal");
        auto signedDistance=[&](Point v) { return (v.x-p.x)*std::cos(yaw)+(v.y-p.y)*std::sin(yaw); };
        if (signedDistance(points.front())>=0||signedDistance(goal)>=0)
            throw std::invalid_argument("Approach and stop goal must be before the stop line");
        zones.push_back({area["name"].as<std::string>(),end,ids,Path(points),p,goal,yaw});
    }
    if (zones.empty()) throw std::invalid_argument("No queue_wait: true areas configured");
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
        // Separating axes test the entire occupied cell against the footprint.
        // The bounding rectangle alone overestimates a rotated vehicle near walls.
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
                // A published costmap uses 99 for inscribed inflation, 100 for
                // actual obstacles. Checking inflation against the whole body
                // expands it twice; polygon collision uses lethal cells instead.
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
class Controller {
public:
    std::map<std::string,double> p{
        {"max_speed",.3},{"max_angular",.4},{"lookahead",.7},
        {"detection_distance",1},{"detection_half_width",.8},
        {"resume_distance",1.3},{"clear_duration",2},
        {"scan_timeout",.5},{"pose_timeout",.5},{"map_timeout",1.5},
        {"max_lateral_deviation",.25},{"corridor_half_width",.65},
        {"deceleration",.3},{"stop_margin",.2},{"stop_goal_radius",.2},
        {"prediction_time",1},{"cmd_timeout",.5}};
    static constexpr double control_rate=20;
    std::vector<Zone> zones; std::vector<Point> footprint;
    int zone=-1;
    bool fault=false,released=false,waiting=true,line_reached=false;
    double clear_since=NAN,last_time=NAN;
    std::string identity; Decision last{"NORMAL"};
    Controller(std::vector<Zone> z,const YAML::Node& parameters,std::vector<Point> f):zones(std::move(z)),footprint(std::move(f)) {
        for (const auto& value:parameters) {
            auto key=value.first.as<std::string>();
            if (!p.count(key)) throw std::invalid_argument("Unknown queue parameter: "+key);
            p[key]=value.second.as<double>();
        }
        for (const auto& value:p) if (!std::isfinite(value.second)||value.second<=0)
            throw std::invalid_argument("Queue parameters must be finite and positive");
        if (p["resume_distance"]<=p["detection_distance"]) throw std::invalid_argument("resume_distance must exceed detection_distance");
        if (footprint.size()<3) throw std::invalid_argument("Footprint needs at least three vertices");
        for (auto v:footprint) if (!finite(v)) throw std::invalid_argument("Non-finite footprint");
    }
    void waypoint(const std::string& id) {
        if (zone>=0&&!zones[zone].ids.count(id)) {
            if (!released) { fault=true; return; }
            zone=-1; released=false; line_reached=false;
        }
        if (zone<0) {
            for (size_t i=0;i<zones.size();++i) if (zones[i].ids.count(id)) { zone=i; break; }
            waiting=true; clear_since=NAN;
        }
        identity=id;
    }
    void pause() { waiting=true; clear_since=NAN; }
    double approachTime() const {
        return p.at("prediction_time")+p.at("max_speed")/p.at("deceleration")+.1;
    }
    struct ObstacleWindow { double start,front,end,stop_end,half_width; };
    ObstacleWindow obstacleWindow(Pose pose) const {
        const auto& path=zones[zone].path;
        double start=path.project({pose.x,pose.y}).along,front=-INFINITY;
        for (auto v:footprint) front=std::max(front,path.project(transform(v,pose)).along);
        double stop_end=front+p.at("detection_distance");
        // Keep observing beyond the red stop range for gradual approach and
        // restart hysteresis. The outer range also covers one scan timeout
        // and control cycle of travel at the configured maximum speed.
        double preview=std::max(p.at("resume_distance"),p.at("detection_distance")+p.at("max_speed")*approachTime());
        preview+=p.at("max_speed")*(p.at("scan_timeout")+1/control_rate);
        return {start,front,front+preview,stop_end,p.at("detection_half_width")};
    }
    double clearance(Pose pose,const std::vector<Point>& points) const {
        const auto& path=zones[zone].path;
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
        for (auto v:footprint) {
            auto q=transform(v,pose);
            closest=std::min(closest,(z.line.x-q.x)*std::cos(z.line_yaw)+(z.line.y-q.y)*std::sin(z.line_yaw));
        }
        return closest;
    }
    Decision hold(const std::string& reason) { last={reason,true,true}; return last; }
    Decision step(double now,const Pose* pose,double pose_stamp,const std::vector<Point>* scan,double scan_stamp,
                  double map_stamp,const std::function<bool(Pose,const std::vector<Point>&)>& collision) {
        if (zone<0) { last={"NORMAL"}; return last; }
        if (std::isfinite(last_time)&&now<last_time) pause();
        last_time=now;
        if (fault) return hold("WAYPOINT_CHANGED_WHILE_WAITING");
        auto fresh=[&](double stamp,double timeout) { return std::isfinite(stamp)&&now-stamp>=-.05&&now-stamp<=timeout; };
        if (!pose||!finite(*pose)||!fresh(pose_stamp,p["pose_timeout"])) { pause(); return hold("POSE_UNAVAILABLE"); }
        if (!scan||!fresh(scan_stamp,p["scan_timeout"])) { pause(); return hold("SCAN_UNAVAILABLE"); }
        if (!collision||!fresh(map_stamp,p["map_timeout"])) { pause(); return hold("MAP_UNAVAILABLE"); }
        if (!collision(*pose,footprint)) { pause(); return hold("CURRENT_POSE_BLOCKED"); }
        const auto& z=zones[zone]; auto q=z.path.project({pose->x,pose->y});
        if (q.lateral>p["max_lateral_deviation"]) { pause(); return hold("OUTSIDE_CORRIDOR"); }
        double gap=clearance(*pose,*scan);
        if (gap<=p["detection_distance"]) pause();
        else if (gap>=p["resume_distance"]) {
            if (!std::isfinite(clear_since)) clear_since=now;
            if (now-clear_since>=p["clear_duration"]) waiting=false;
        } else clear_since=NAN;
        double remaining=boundary(*pose);
        if (remaining<=p["stop_margin"]+.03||
            (identity==z.stop_id&&std::hypot(pose->x-z.stop_goal.x,pose->y-z.stop_goal.y)<=p["stop_goal_radius"]))
            line_reached=true;
        if (!released&&line_reached) return hold("STOP_LINE");
        if (waiting) return hold("QUEUE_WAIT");
        if (released) { last={"RELEASED_WAITING_FOR_NEXT_WAYPOINT",true,false}; return last; }
        auto target=z.path.pointAt(std::min(q.along+p["lookahead"],z.path.length));
        double error=wrap(std::atan2(target.y-pose->y,target.x-pose->x)-pose->yaw);
        double angular=std::max(-p["max_angular"],std::min(p["max_angular"],1.5*error));
        double available=std::max(0.0,std::min(remaining-p["stop_margin"],gap-p["detection_distance"]));
        double speed=std::min(p["max_speed"],std::sqrt(2*p["deceleration"]*available));
        speed=std::min(speed,available/approachTime());
        speed*=std::max(0.0,std::cos(error)); if (std::abs(error)>.6) speed=0;
        Pose prediction=*pose;
        double horizon=std::max(p["prediction_time"],speed/p["deceleration"]+1/control_rate);
        double dt=std::min(.05,.5*p["max_lateral_deviation"]/std::max(speed,.01));
        for (int i=0;i<=std::ceil(horizon/dt);++i) {
            if (z.path.project({prediction.x,prediction.y}).lateral>p["max_lateral_deviation"]||
                boundary(prediction)<p["stop_margin"]||!collision(prediction,footprint)) return hold("TRAJECTORY_BLOCKED");
            for (auto v:footprint) if (z.path.project(transform(v,prediction)).lateral>p["corridor_half_width"])
                return hold("TRAJECTORY_BLOCKED");
            prediction.x+=speed*std::cos(prediction.yaw)*dt;
            prediction.y+=speed*std::sin(prediction.yaw)*dt;
            prediction.yaw=wrap(prediction.yaw+angular*dt);
        }
        last={"QUEUE_ADVANCE",true,false,speed,angular}; return last;
    }
    bool release(double now,std::string& message) {
        if (!std::isfinite(last_time)||now-last_time>2/control_rate||last.state!="STOP_LINE") {
            message="Release requires fresh data and a stop at the stop line"; return false;
        }
        if (!std::isfinite(clear_since)||now-clear_since<p["clear_duration"]||waiting) {
            message="Front space must be continuously clear before release"; return false;
        }
        released=true; message="Released; use Next Waypoint to continue"; return true;
    }
};
}  // namespace queue_wait

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <actionlib_msgs/GoalID.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
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
    ros::Publisher goal_pub,cmd_pub,cancel_pub,active_pub,hold_pub,state_pub,detection_pub;
    std::vector<ros::Subscriber> subscribers;
    ros::Timer timer; ros::ServiceServer release_service;
    bool queue_enabled=false,running=false,was_active=false,config_ready=false;
    bool route_ready=false,waypoints_ready=false,waypoint_ready=false,scan_valid=false,fault=false;
    std::string frame,base,input_topic,output_topic,config_text,previous_state;
    YAML::Node config;
    std::unique_ptr<queue_wait::Controller> controller;
    std::vector<std::string> route;
    std::map<std::string,queue_wait::Waypoint> waypoints;
    waypoint_manager_msgs::Waypoint current_waypoint;
    std::vector<queue_wait::Point> scan; geometry_msgs::Twist raw_cmd;
    double scan_stamp=NAN,cmd_stamp=NAN,goal_stamp=NAN;
    struct Map { std::shared_ptr<queue_wait::Grid> grid; std::string frame; double stamp; };
    std::map<std::string,Map> maps;
    queue_wait::Pose transform(const std::string& target,const std::string& source,ros::Time stamp) {
        if (target==source) return {0,0,0};
        // Laser and TF messages can arrive in either order. Wait briefly for the
        // transform at the scan timestamp; never substitute an older pose.
        tf.waitForTransform(target,source,stamp,ros::Duration(.02));
        tf::StampedTransform t; tf.lookupTransform(target,source,stamp,t);
        queue_wait::Pose result{t.getOrigin().x(),t.getOrigin().y(),tf::getYaw(t.getRotation())};
        if (!queue_wait::finite(result)) throw std::runtime_error("Non-finite transform");
        return result;
    }
    void stop() { cmd_pub.publish(geometry_msgs::Twist{}); }
    void cancel() { cancel_pub.publish(actionlib_msgs::GoalID{}); }
    void publishGoal() {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id=frame; goal.header.stamp=ros::Time::now(); goal.pose=current_waypoint.pose;
        goal_stamp=goal.header.stamp.toSec(); goal_pub.publish(goal);
    }
    void fail(const std::string& reason) {
        fault=true; cancel(); stop(); if (controller) controller->fault=true;
        ROS_ERROR("queue_wait: %s",reason.c_str());
    }
    void initialize() {
        if (controller||fault||!config_ready||!route_ready||!waypoints_ready) return;
        try {
            std::string wired_topic; ros::param::get("/move_base/queue_wait_cmd_vel_topic",wired_topic);
            if (wired_topic!=input_topic) throw std::invalid_argument("Enable use_queue_wait in navigation launch too");
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
            if (waypoint_ready) { controller->waypoint(current_waypoint.identity); if (controller->zone<0) publishGoal(); }
            ROS_INFO("queue_wait: %zu configured crossing areas",controller->zones.size());
        } catch (const std::exception& e) { fail(e.what()); }
    }
    void onConfig(const std_msgs::String::ConstPtr& msg) {
        if (config_ready&&config_text!=msg->data) { fail("Configuration changed; restart with the new route/config"); return; }
        try { config=YAML::Load(msg->data); config_text=msg->data; config_ready=true; initialize(); }
        catch (const std::exception& e) { fail(e.what()); }
    }
    void onRoute(const waypoint_manager_msgs::Route::ConstPtr& msg) {
        if (controller&&route!=msg->identities) fail("Route changed; restart with the new route/config");
        route=msg->identities; route_ready=true; initialize();
    }
    void onWaypoints(const waypoint_manager_msgs::Waypoints::ConstPtr& msg) {
        if (msg->info.header.frame_id!=frame) { fail("Waypoint frame differs from global frame"); return; }
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
        current_waypoint=*msg; waypoint_ready=true;
        if (!queue_enabled) { publishGoal(); return; }
        if (!controller||fault) return;
        int previous_zone=controller->zone; controller->waypoint(msg->identity);
        if (controller->zone>=0&&previous_zone<0) {
            cancel(); stop(); was_active=true;
            std_msgs::Bool held; held.data=true; active_pub.publish(held); hold_pub.publish(held);
        }
        if (controller->fault) { fail("Waypoint changed before operator release"); return; }
        if (controller->zone<0) publishGoal();
    }
    void onRunning(const std_msgs::Bool::ConstPtr& msg) {
        running=msg->data; if (!running) { cancel(); stop(); if (controller) controller->pause(); }
    }
    void onCommand(const geometry_msgs::Twist::ConstPtr& msg) {
        const auto& l=msg->linear; const auto& a=msg->angular;
        if (!std::isfinite(l.x)||!std::isfinite(l.y)||!std::isfinite(l.z)||!std::isfinite(a.x)||!std::isfinite(a.y)||!std::isfinite(a.z)) {
            raw_cmd=geometry_msgs::Twist{}; cmd_stamp=NAN; return;
        }
        raw_cmd=*msg; cmd_stamp=ros::Time::now().toSec();
    }
    void onScan(const sensor_msgs::LaserScan::ConstPtr& msg) {
        scan_valid=false; scan.clear(); scan_stamp=msg->header.stamp.toSec();
        try {
            if (!std::isfinite(msg->angle_min)||!std::isfinite(msg->angle_increment)||msg->angle_increment<=0)
                throw std::runtime_error("Invalid scan angles");
            auto t=transform(frame,msg->header.frame_id,msg->header.stamp);
            for (size_t i=0;i<msg->ranges.size();++i) {
                double r=msg->ranges[i],a=msg->angle_min+i*msg->angle_increment;
                if (std::isfinite(r)&&r>=msg->range_min&&r<=msg->range_max)
                    scan.push_back(queue_wait::transform({r*std::cos(a),r*std::sin(a)},t));
            }
            scan_valid=!scan.empty(); // all NaN/inf is missing data, not front-clear evidence
        } catch (const std::exception& e) { ROS_WARN_THROTTLE(2,"queue_wait scan: %s",e.what()); }
    }
    void onMap(const std::string& key,const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        try {
            const auto& o=msg->info.origin;
            auto grid=std::make_shared<queue_wait::Grid>(msg->info.width,msg->info.height,msg->info.resolution,
                queue_wait::Pose{o.position.x,o.position.y,tf::getYaw(o.orientation)},msg->data,key=="local");
            maps[key]={grid,msg->header.frame_id,msg->header.stamp.toSec()};
        } catch (const std::exception& e) { maps.erase(key); ROS_ERROR("queue_wait grid: %s",e.what()); }
    }
    queue_wait::Decision decision(double now) {
        queue_wait::Pose pose{}; bool pose_valid=false; double pose_stamp=NAN,map_stamp=NAN;
        try {
            tf::StampedTransform t; tf.lookupTransform(frame,base,ros::Time(0),t);
            pose={t.getOrigin().x(),t.getOrigin().y(),tf::getYaw(t.getRotation())}; pose_stamp=t.stamp_.toSec(); pose_valid=true;
        } catch (const std::exception&) {}
        std::function<bool(queue_wait::Pose,const std::vector<queue_wait::Point>&)> collision;
        if (maps.count("local")&&maps.count("constraint")) try {
            std::vector<std::pair<std::shared_ptr<queue_wait::Grid>,queue_wait::Pose>> grids;
            for (const auto& m:maps) grids.push_back({m.second.grid,transform(m.second.frame,frame,ros::Time(0))});
            map_stamp=maps.at("local").stamp;
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
                const auto& path=controller->zones[controller->zone].path;
                const auto window=controller->obstacleWindow(pose);
                const double start=window.start,end=window.stop_end,width=window.half_width;
                auto offset=[](queue_wait::Point p,queue_wait::Point n,double d) {
                    return queue_wait::Point{p.x+n.x*d,p.y+n.y*d};
                };
                auto append=[&](queue_wait::Point p) {
                    geometry_msgs::Point point; point.x=p.x; point.y=p.y; point.z=.08;
                    marker.points.push_back(point);
                };
                // Use the same projection as clearance(), trimming overlapping
                // edges at bends. Boundary sampling has a maximum step of 5 cm.
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
                // Clamped projections around internal vertices give round corners.
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
    }
    void tick(const ros::TimerEvent&) {
        geometry_msgs::Twist output; queue_wait::Decision result{"CONFIG_UNAVAILABLE",true,true};
        double now=ros::Time::now().toSec();
        if (controller&&!fault) {
            result=decision(now);
            if (!running) { controller->pause(); result.hold=result.active; result.state="OPERATOR_PAUSED"; }
            else if (result.active) {
                if (!was_active) cancel();
                output.linear.x=result.linear; output.angular.z=result.angular;
            } else if (!was_active&&cmd_stamp>=goal_stamp&&now-cmd_stamp>=0&&
                       now-cmd_stamp<=controller->p["cmd_timeout"]) output=raw_cmd;
        }
        was_active=result.active;
        std_msgs::Bool active,held; active.data=result.active; held.data=result.hold;
        active_pub.publish(active); hold_pub.publish(held); cmd_pub.publish(output);
        std_msgs::String state; state.data=result.state; state_pub.publish(state);
        if (previous_state!=state.data) { ROS_INFO("queue_wait: %s",state.data.c_str()); previous_state=state.data; }
        publishDetectionRange(ros::Time::now());
    }
    bool release(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& response) {
        if (!controller||fault||!running) { response.message="Resume Switch Cancel and valid queue configuration first"; return true; }
        double now=ros::Time::now().toSec(); decision(now);
        response.success=controller->release(now,response.message);
        if (response.success) { std_msgs::Bool held; held.data=false; hold_pub.publish(held); }
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
        subscribers.push_back(nh.subscribe("/scan",1,&Node::onScan,this));
        subscribers.push_back(nh.subscribe(input_topic,1,&Node::onCommand,this));
        subscribers.push_back(nh.subscribe("/waypoint_manager/waypoint_server/running",1,&Node::onRunning,this));
        subscribers.push_back(nh.subscribe<nav_msgs::OccupancyGrid>("/move_base/local_costmap/costmap",1,
            [this](const nav_msgs::OccupancyGrid::ConstPtr& m) { onMap("local",m); }));
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

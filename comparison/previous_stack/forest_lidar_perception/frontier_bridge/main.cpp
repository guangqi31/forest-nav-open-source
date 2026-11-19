// 只提供前沿数据，不发送导航目标。路径、足迹膨胀和控制继续由现有 Nav2 完成。
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <explore/frontier_search.h>
#include <sstream>
class FrontierBridge : public rclcpp::Node {
  nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
public:
  FrontierBridge():Node("forest_frontier_bridge"),tf_(get_clock()),listener_(tf_) {
    sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/traversability_map",rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr m){grid_=m;});
    srv_=create_service<std_srvs::srv::Trigger>("/navigation/frontiers",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res){
        if(!grid_ || grid_->header.frame_id!="map" || grid_->data.size()!=grid_->info.width*grid_->info.height){res->message="map_not_ready";return;}
        try {
          auto t=tf_.lookupTransform("map","base_link",tf2::TimePointZero);
          const auto &i=grid_->info;
          nav2_costmap_2d::Costmap2D map(i.width,i.height,i.resolution,i.origin.position.x,i.origin.position.y,255);
          // 搜索拓扑仅区分可通行/障碍/未知；这里不修改原始代价图。
          // 输出的未知边界不能直接作为目标，Python管理器必须再筛选已知安全观察点。
          auto *data=map.getCharMap();
          for(size_t k=0;k<grid_->data.size();++k)data[k]=grid_->data[k]<0?255:(grid_->data[k]>=100?254:0);
          frontier_exploration::FrontierSearch search(&map,1.,1.,.6,get_logger());
          geometry_msgs::msg::Point pos;pos.x=t.transform.translation.x;pos.y=t.transform.translation.y;
          auto frontiers=search.searchFrom(pos);
          std::ostringstream out;out<<"{\"frame_id\":\"map\",\"frontiers\":[";
          bool first=true;
          for(const auto &f:frontiers){
            if(!first)out<<",";first=false;
            out<<"{\"size\":"<<f.size<<",\"points\":[";bool fp=true;
            // 减少服务传输量；间隔仍小于观察点到边界的退让距离。
            for(size_t k=0;k<f.points.size();k+=2){if(!fp)out<<",";fp=false;out<<"["<<f.points[k].x<<","<<f.points[k].y<<"]";}
            out<<"]}";
          }
          out<<"]}";res->success=true;res->message=out.str();
        }catch(const std::exception &e){res->message=e.what();}
      });
  }
};
int main(int argc,char **argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<FrontierBridge>());rclcpp::shutdown();}
